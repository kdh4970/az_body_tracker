#ifndef __TRACKER_NODE_H__
#define __TRACKER_NODE_H__
#include <k4a/k4a.hpp>
#include <k4abt.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "azbt_msgs/msg/bt.hpp"
#include "azbt_msgs/msg/bt_multi_array.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <fstream>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <BodyTrackingHelpers.h>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

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

class TrackerNode : public rclcpp::Node
{
public:
  bool debug;
  bool tfupdateRequire = true;
  k4a::image point_cloud_image;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr body_marker_publisher;
  rclcpp::Publisher<azbt_msgs::msg::BtMultiArray>::SharedPtr bt_pub;
  Eigen::Matrix4f transform;
  Eigen::Matrix4f scale;
  k4a_transformation_t transformation;
  TrackerNode();
  ~TrackerNode();
  void showInfo(InputSettings& inputSettings);
  void readTransformPreset();
  void updateTransform();
  void pclbuffer(int depth_width, int depth_height);
  k4a_result_t getPointCloud(const k4a_transformation_t tf_handle , const k4a_image_t k4a_depth_frame, sensor_msgs::msg::PointCloud2& point_cloud, rclcpp::Time capture_time);
  k4a_result_t getBodyMarker(const k4abt_body_t& body, visualization_msgs::msg::Marker &marker_msg, int jointType, rclcpp::Time capture_time);
  rclcpp::Time timestampToROS(const std::chrono::microseconds& k4a_timestamp_us);
  std::chrono::microseconds get_device_timestamp(const k4a_image_t k4a_depth_img);

private:
  transformData _t;
  std::string pub_frame;
  void initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us);
  k4a_result_t fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::msg::PointCloud2& point_cloud);
  std::chrono::nanoseconds device_to_realtime_offset_{0};
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &parameter : parameters)
    {
      if(parameter.get_name() == "debug" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL){
        debug = parameter.as_bool();
      }
      if (parameter.get_name() == "roll" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        _t.roll = parameter.as_double();
      }
      if (parameter.get_name() == "pitch" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        _t.pitch = parameter.as_double();
      }
      if (parameter.get_name() == "yaw" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        _t.yaw = parameter.as_double();
      }
      if (parameter.get_name() == "x" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        _t.x = parameter.as_double();
      }
      if (parameter.get_name() == "y" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        _t.y = parameter.as_double();
      }
      if (parameter.get_name() == "z" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        _t.z = parameter.as_double();
      }
      if (parameter.get_name() == "scale" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
        _t.scale = parameter.as_double();
      }
    }
    // if(debug)
    // {
      RCLCPP_INFO(this->get_logger(),"Updating new parameters...");
      RCLCPP_INFO(this->get_logger(),">> debug : %d",debug);
      RCLCPP_INFO(this->get_logger(),">> roll %.lf",_t.roll);
      RCLCPP_INFO(this->get_logger(),">> pitch %.lf",_t.pitch);
      RCLCPP_INFO(this->get_logger(),">> yaw %.lf",_t.yaw);
      RCLCPP_INFO(this->get_logger(),">> x %.lf",_t.x);
      RCLCPP_INFO(this->get_logger(),">> y %.lf",_t.y);
      RCLCPP_INFO(this->get_logger(),">> z %.lf",_t.z);
      RCLCPP_INFO(this->get_logger(),">> scale %.lf",_t.scale);
    // }
    tfupdateRequire = true;
    return result;
  }  
};
#endif // __TRACKER_NODE_H__