#ifndef TRACKER_NODE_H
#define TRACKER_NODE_H
#include <k4a/k4a.h>
#include <k4abt.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

struct InputSettings
{
    k4a_depth_mode_t DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
    k4a_fps_t cameraFPS = K4A_FRAMES_PER_SECOND_30;
    k4a_color_resolution_t colorResolution = K4A_COLOR_RESOLUTION_720P;
};

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode();
  void timer_callback();
  void showInfo(InputSettings& inputSettings);
private:
  uint16_t count_=0; 
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
};
#endif // TRACKER_NODE_H