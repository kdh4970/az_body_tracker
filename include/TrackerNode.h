#ifndef __TRACKER_NODE_H__
#define __TRACKER_NODE_H__
#include <k4a/k4a.hpp>
#include <k4abt.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <azbt_msgs/bt_data.h>
#include <azbt_msgs/Elem.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include "azure_kinect_ros_driver/k4a_calibration_transform_data.h"
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"
#include <chrono>
#include <dynamic_reconfigure/server.h>
#include <az_body_tracker/az_body_trackerConfig.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <BodyTrackingHelpers.h>

#define PI 3.14159265

struct transformData
{
  float roll{0.0f},pitch{0.0f},yaw{0.0f},x{0.0f},y{0.0f},z{0.0f},scale{1.0f};
};

struct InputSettings
{
  k4a_depth_mode_t DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
  k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
  k4a_fps_t cameraFPS = K4A_FRAMES_PER_SECOND_15;
  k4a_color_resolution_t colorResolution = K4A_COLOR_RESOLUTION_720P;
  
};
using ColorPalette = std::array<Color, 8>;
static const ColorPalette BODY_COLOR_PALETTE{ { { 1.0f, 0.0f, 0.0f, 1.0f },
                                                { 0.0f, 1.0f, 0.0f, 1.0f },
                                                { 0.0f, 0.0f, 1.0f, 1.0f },
                                                { 1.0f, 1.0f, 0.0f, 1.0f },
                                                { 1.0f, 0.0f, 1.0f, 1.0f },
                                                { 0.0f, 1.0f, 1.0f, 1.0f },
                                                { 0.0f, 0.0f, 0.0f, 1.0f },
                                                { 1.0f, 1.0f, 1.0f, 1.0f } } };

class TrackerNode
{
public:
  ros::NodeHandle nh;
  ros::Publisher bt_pub, pcl_pub, body_marker_publisher;
  bool debug;
  int pub_hz;
  k4a::image point_cloud_image;
  k4a_transformation_t transformation;
  TrackerNode(ros::NodeHandle& _nh);
  ~TrackerNode();
  void pclbuffer(int depth_width, int depth_height);
  void showInfo(InputSettings& inputSettings);
  void loadParams(Eigen::Matrix4f* Extr, Eigen::Matrix4f* DepthToColor, Eigen::Matrix4f* WorldToMap);
  k4a_result_t getPointCloud(const k4a_transformation_t tf_handle , const k4a_image_t k4a_depth_frame, sensor_msgs::PointCloud2Ptr& point_cloud, ros::Time capture_time);
  void updateTransform(transformData input);
  k4a_result_t getBodyMarker(const k4abt_body_t& body, visualization_msgs::MarkerPtr marker_msg, int jointType, ros::Time capture_time);
  Eigen::Matrix4f transform;
  Eigen::Matrix4f scale;
  std::chrono::microseconds get_device_timestamp(const k4a_image_t k4a_depth_img);
  ros::Time timestampToROS(const std::chrono::microseconds& k4a_timestamp_us);
private:
  std::string pub_frame;
  void initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us);
  k4a_result_t fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2Ptr& point_cloud);
  std::chrono::nanoseconds device_to_realtime_offset_{0};
};
#endif // __TRACKER_NODE_H__