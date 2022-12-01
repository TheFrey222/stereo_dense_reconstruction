cmake_minimum_required(VERSION 2.8.3)
project(stereo_dense_reconstruction)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-march=native \
-Wno-deprecated-declarations \
-Wno-deprecated \
-Wno-ignored-qualifiers \
-Wno-reorder \
-Wno-missing-braces \
-msse3 \
-pthread ")

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
message_generation
sensor_msgs
geometry_msgs
roscpp
rospy
std_msgs
cv_bridge
image_transport
dynamic_reconfigure
)

catkin_package()

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

find_package(Boost COMPONENTS system thread filesystem program_options REQUIRED)
set(INC_DIR ./include)
set(LIBELAS_SRC_DIR src/elas)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
  ${LIBELAS_SRC_DIR}
  ${INC_DIR}
    
)

add_executable(dense_reconstruction
  src/dense_reconstruction.cpp
  src/elas/descriptor.cpp
  src/elas/elas.cpp
  src/elas/filter.cpp
  src/elas/matrix.cpp
  src/elas/triangle.cpp)

target_link_libraries(dense_reconstruction 
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${OpenCV_LIBS} "-lpopt")