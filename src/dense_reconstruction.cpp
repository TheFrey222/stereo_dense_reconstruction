#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <stereo_dense_reconstruction/CamToRobotCalibParamsConfig.h>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "elas.h"
#include "popt_pp.h"
#include <yaml-cpp/yaml.h>

using namespace cv;
using namespace std;

Mat K1(3, 3, CV_64F);
Mat K2(3, 3, CV_64F);
Mat D1(1, 4, CV_64F);
Mat D2(1, 4, CV_64F);
Mat R(3, 3, CV_64F);
Mat XR(3, 3, CV_64F);
Mat XT(1, 3, CV_64F);
Mat Q(4, 4, CV_64F);

Mat lmapx, lmapy, rmapx, rmapy;
Vec3d T;
stereo_dense_reconstruction::CamToRobotCalibParamsConfig config;
FileStorage calib_file;
int debug = 0;
Size calib_img_size;

image_transport::Publisher dmap_pub;
ros::Publisher pcl_pub;

Mat composeRotationCamToRobot(float x, float y, float z) {
  Mat X = Mat::eye(3, 3, CV_64FC1);
  Mat Y = Mat::eye(3, 3, CV_64FC1);
  Mat Z = Mat::eye(3, 3, CV_64FC1);
  
  X.at<double>(1,1) = cos(x);
  X.at<double>(1,2) = -sin(x);
  X.at<double>(2,1) = sin(x);
  X.at<double>(2,2) = cos(x);

  Y.at<double>(0,0) = cos(y);
  Y.at<double>(0,2) = sin(y);
  Y.at<double>(2,0) = -sin(y);
  Y.at<double>(2,2) = cos(y);

  Z.at<double>(0,0) = cos(z);
  Z.at<double>(0,1) = -sin(z);
  Z.at<double>(1,0) = sin(z);
  Z.at<double>(1,1) = cos(z);
  
  return Z*Y*X;
}

Mat composeTranslationCamToRobot(float x, float y, float z) {
  return (Mat_<double>(3,1) << x, y, z);
}

void publishPointCloud(Mat& img_left, Mat& dmap) {
  if (debug == 1) {
    XR = composeRotationCamToRobot(config.PHI_X,config.PHI_Y,config.PHI_Z);
    XT = composeTranslationCamToRobot(config.TRANS_X,config.TRANS_Y,config.TRANS_Z);
    cout << "Rotation matrix: " << XR << endl;
    cout << "Translation matrix: " << XT << endl;
  }
  Mat V = Mat(4, 1, CV_64FC1);
  Mat pos = Mat(4, 1, CV_64FC1);
  vector< Point3d > points;
  sensor_msgs::PointCloud pc;
  sensor_msgs::ChannelFloat32 ch;
  ch.name = "rgb";
  pc.header.frame_id = "jackal";
  pc.header.stamp = ros::Time::now();
  for (int i = 0; i < img_left.cols; i++) {
    for (int j = 0; j < img_left.rows; j++) {
      int d = dmap.at<uchar>(j,i);
      // if low disparity, then ignore
      if (d < 2) {
        continue;
      }
      // V is the vector to be multiplied to Q to get
      // the 3D homogenous coordinates of the image point
      V.at<double>(0,0) = (double)(i);
      V.at<double>(1,0) = (double)(j);
      V.at<double>(2,0) = (double)d;
      V.at<double>(3,0) = 1.;
      pos = Q * V; // 3D homogeneous coordinate
      double X = pos.at<double>(0,0) / pos.at<double>(3,0);
      double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
      double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
      Mat point3d_cam = Mat(3, 1, CV_64FC1);
      point3d_cam.at<double>(0,0) = X;
      point3d_cam.at<double>(1,0) = Y;
      point3d_cam.at<double>(2,0) = Z;
      // transform 3D point from camera frame to robot frame
      Mat point3d_robot = XR * point3d_cam + XT;
      points.push_back(Point3d(point3d_robot));
      geometry_msgs::Point32 pt;
      pt.x = point3d_robot.at<double>(0,0);
      pt.y = point3d_robot.at<double>(1,0);
      pt.z = point3d_robot.at<double>(2,0);
      pc.points.push_back(pt);
      int32_t red, blue, green;
      red = img_left.at<Vec3b>(j,i)[2];
      green = img_left.at<Vec3b>(j,i)[1];
      blue = img_left.at<Vec3b>(j,i)[0];
      int32_t rgb = (red << 16 | green << 8 | blue);
      ch.values.push_back(*reinterpret_cast<float*>(&rgb));
    }
  }
  if (!dmap.empty()) {
    sensor_msgs::ImagePtr disp_msg;
    disp_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", dmap).toImageMsg();
    dmap_pub.publish(disp_msg);
  }
  pc.channels.push_back(ch);
  pcl_pub.publish(pc);
}

Mat generateDisparityMap(Mat& left, Mat& right) {
  if (left.empty() || right.empty()) 
    return left;
  const Size imsize = left.size();
  const int32_t dims[3] = {imsize.width, imsize.height, imsize.width};
  Mat leftdpf = Mat::zeros(imsize, CV_32F);
  Mat rightdpf = Mat::zeros(imsize, CV_32F);

  Elas::parameters param(Elas::MIDDLEBURY);
  param.postprocess_only_left = true;
  Elas elas(param);
  elas.process(left.data, right.data, leftdpf.ptr<float>(0), rightdpf.ptr<float>(0), dims);
  Mat dmap = Mat(calib_img_size, CV_8UC1, Scalar(0));
  leftdpf.convertTo(dmap, CV_8U, 1.);
  return dmap;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right) {
  Mat img_left_color = cv_bridge::toCvShare(msg_left, "bgr8")->image;
  Mat img_right_color = cv_bridge::toCvShare(msg_right, "bgr8")->image;
  if (img_left_color.empty() || img_right_color.empty())
    return;
  
  Mat img_left, img_right;
  cvtColor(img_left_color, img_left, CV_BGR2GRAY);
  cvtColor(img_right_color, img_right, CV_BGR2GRAY);
  
  Mat dmap = generateDisparityMap(img_left_color, img_right_color);
  publishPointCloud(img_left_color, dmap);
  
  imshow("LEFT", img_left_color);
  imshow("RIGHT", img_right_color);
  imshow("DISP", dmap);
  waitKey(30);
}

void paramsCallback(stereo_dense_reconstruction::CamToRobotCalibParamsConfig &conf, uint32_t level) {
  config = conf;
}

void readCalibration(const char* calib_file_name) {

  YAML::Node config = YAML::LoadFile(calib_file_name);

  std::vector<int> resolution = config["cam0"]["resolution"].as<std::vector<int>>();
  calib_img_size = Size(resolution[0], resolution[1]);

  std::vector<double> intr_l = config["cam0"]["intrinsics"].as<std::vector<double>>();
  K1.at<double>(0,0) = intr_l[0];
	K1.at<double>(0,1) = 0.0;
	K1.at<double>(0,2) = intr_l[2];
	K1.at<double>(1,0) = 0.0;
	K1.at<double>(1,1) = intr_l[1];
	K1.at<double>(1,2) = intr_l[3];
	K1.at<double>(2,0) = 0.0;
	K1.at<double>(2,1) = 0.0;
	K1.at<double>(2,2) = 0.0;

  std::vector<double> intr_r = config["cam1"]["intrinsics"].as<std::vector<double>>();
  K2.at<double>(0,0) = intr_r[0];
	K2.at<double>(0,1) = 0.0;
	K2.at<double>(0,2) = intr_r[2];
	K2.at<double>(1,0) = 0.0;
	K2.at<double>(1,1) = intr_r[1];
	K2.at<double>(1,2) = intr_r[3];
	K2.at<double>(2,0) = 0.0;
	K2.at<double>(2,1) = 0.0;
	K2.at<double>(2,2) = 0.0;

  std::vector<double> dist_l = config["cam0"]["distortion_coeffs"].as<std::vector<double>>();	
	D1.at<double>(0,0) = dist_l[0];
	D1.at<double>(0,1) = dist_l[1];
	D1.at<double>(0,2) = dist_l[2];
	D1.at<double>(0,3) = dist_l[3];

  std::vector<double> dist_r = config["cam1"]["distortion_coeffs"].as<std::vector<double>>();
	D2.at<double>(0,0) = dist_r[0];
	D2.at<double>(0,1) = dist_r[1];
	D2.at<double>(0,2) = dist_r[2];
	D2.at<double>(0,3) = dist_r[3];

  R.at<double>(0,0) = 1.0;
	R.at<double>(0,1) = 0.0;
	R.at<double>(0,2) = 0.0;
	R.at<double>(1,0) = 0.0;
	R.at<double>(1,1) = 1.0;
	R.at<double>(1,2) = 0.0;
	R.at<double>(2,0) = 0.0;
	R.at<double>(2,1) = 0.0;
	R.at<double>(2,2) = 1.0;

 	std::vector<double> T_0 = config["cam1"]["T_cn_cnm1"][0].as<std::vector<double>>();
  T[0] = T_0[3];
	T[1] = 0.0;
	T[2] = 0.0;

  XR.at<double>(0,0) = 1.0;
	XR.at<double>(0,1) = 0.0;
	XR.at<double>(0,2) = 0.0;
	XR.at<double>(1,0) = 0.0;
	XR.at<double>(1,1) = 1.0;
	XR.at<double>(1,2) = 0.0;
	XR.at<double>(2,0) = 0.0;
	XR.at<double>(2,1) = 0.0;
	XR.at<double>(2,2) = 1.0;

  XT.at<double>(0) = 0.0;
	XT.at<double>(1) = 0.0;
	XT.at<double>(2) = 0.0;

  Q.at<double>(0,0) = 1.0;
	Q.at<double>(0,1) = 0.0;
	Q.at<double>(0,2) = 0.0;
	Q.at<double>(0,3) = -intr_l[2];
	Q.at<double>(1,0) = 0.0;
	Q.at<double>(1,1) = 1.0;
	Q.at<double>(1,2) = 0.0;
	Q.at<double>(1,3) = -intr_l[3];
	Q.at<double>(2,0) = 0.0;
	Q.at<double>(2,1) = 0.0;
	Q.at<double>(2,2) = 1.0;
	Q.at<double>(2,3) = -intr_l[1];
  Q.at<double>(3,0) = 0.0;
	Q.at<double>(3,1) = 0.0;
	Q.at<double>(3,2) = -1.0/T_0[3];
	Q.at<double>(3,3) = 0.0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "jpp_dense_reconstruction");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  const char* left_img_topic;
  const char* right_img_topic;
  const char* calib_file_name;
  
  static struct poptOption options[] = {
    { "left_topic",'l',POPT_ARG_STRING,&left_img_topic,0,"Left image topic name","STR" },
    { "right_topic",'r',POPT_ARG_STRING,&right_img_topic,0,"Right image topic name","STR" },
    { "calib_file",'c',POPT_ARG_STRING,&calib_file_name,0,"Stereo calibration file name","STR" },
    { "debug",'d',POPT_ARG_INT,&debug,0,"Set d=1 for cam to robot frame calibration","NUM" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  
  readCalibration(calib_file_name);
  
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, left_img_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, right_img_topic, 1);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&imgCallback, _1, _2));
  
  dynamic_reconfigure::Server<stereo_dense_reconstruction::CamToRobotCalibParamsConfig> server;
  dynamic_reconfigure::Server<stereo_dense_reconstruction::CamToRobotCalibParamsConfig>::CallbackType f;

  f = boost::bind(&paramsCallback, _1, _2);
  server.setCallback(f);
  
  dmap_pub = it.advertise("/camera/left/disparity_map", 1);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud>("/camera/left/point_cloud",1);

  ros::spin();
  return 0;
}