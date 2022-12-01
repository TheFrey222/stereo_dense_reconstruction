# Dense 3D Reconstruction from Stereo

This is a ROS package for real-time 3D reconstruction from stereo images. Currently this version uses [LIBELAS](http://www.cvlibs.net/software/libelas/) for generating dense disparity maps as a baseline. The method for generation of disparity maps can be changed based on user preferences.

This package serves as a visualization tool for dense disparity maps and point clouds. Additionally, a tool for transforming point clouds to a different reference frame is also included. 

Usually, the point clouds are formed in the reference frame of the left camera. For ground robots, often the point clouds need to be transformed to a different frame *e.g.*, a reference frame with the origin at the centre of rotation of the robot projected on to the ground plane. These transformations are hard to calculate mathematically - this tool can be used to find the transformations visually.

- Author: [Sourish Ghosh](http://sourishghosh.com/)
- Author: [Andrej Studer](http://tethys-robotics.ch)

## Dependencies

- A C++ compiler (*e.g.*, [GCC](http://gcc.gnu.org/))
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [cmake](http://www.cmake.org/cmake/resources/software.html)
- [popt](http://freecode.com/projects/popt)
- [Boost](http://www.boost.org/)
- [OpenCV](https://github.com/opencv/opencv)

## Stereo Calibration

In this version, the camera streams need to be already undistorted and rectified (preferably with [Kalibr](https://github.com/ethz-asl/kalibr)). The rectified calibration file (zero distortion and so on) need to be stored in a ".yaml" file as in Kalibr.

## Compiling

Clone the repository in your workspace:

```bash
$ git clone https://github.com/TheFrey222/stereo_dense_reconstruction
```

Build package:

```bash
$ catkin_make
```

## Running Dense 3D Reconstruction

```bash
$ ./bin/dense_reconstruction [OPTION...]
```

```bash
Usage: dense_reconstruction [OPTION...]
  -l, --left_topic=STR       Left image topic name
  -r, --right_topic=STR      Right image topic name
  -c, --calib_file=STR       Stereo calibration file name
  -d, --debug=NUM            Set d=1 for cam to robot frame calibration
```

This node outputs the dense disparity map as a grayscale image on the topic `/camera/left/disparity_map` and the corresponding point cloud on the topic 
`/camera/left/point_cloud`.

## Point Cloud Transformation

The point cloud can be viewed on `rviz` by running:

```bash
$ rviz
```

To transform the point cloud to a different reference frame, the `XR` and `XT` matrices (rotation and translation) in the calibration file need to be changed. 
This can be done real-time by the running:

```bash
$ rqt_reconfigure
```

If you change the Euler Angles in `rqt_reconfigure` you should be able to see the point cloud transform. Don't forget to set `d=1` when running the 
`dense_reconstruction` node. This prints out the new transformation matrices as you transform the point cloud.

## License

This software is released under the [GNU GPL v3 license](LICENSE).
