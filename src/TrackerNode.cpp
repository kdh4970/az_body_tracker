#include "TrackerNode.h"

using namespace std::chrono_literals;

TrackerNode::TrackerNode(): Node("tracker_node")
{
	publisher = this->create_publisher<std_msgs::msg::String>("count", 10);
	timer = this->create_wall_timer(1000ms, std::bind(&TrackerNode::timer_callback, this));
}
void TrackerNode::timer_callback()
{
	auto message = std_msgs::msg::String();
	message.data = std::to_string(this->count_++);
	RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
	publisher->publish(message);
}

void TrackerNode::showInfo(InputSettings& inputSettings){
	if(inputSettings.DepthCameraMode == K4A_DEPTH_MODE_NFOV_UNBINNED) {
		RCLCPP_INFO(this->get_logger(), "K4A depth_mode : NFOV_UNBINNED");
		RCLCPP_INFO(this->get_logger(), "K4A camera_FPS : 30");}
	else if (inputSettings.DepthCameraMode == K4A_DEPTH_MODE_WFOV_2X2BINNED){
		RCLCPP_INFO(this->get_logger(), "K4A depth_mode : WFOV_2X2BINNED");
		RCLCPP_INFO(this->get_logger(), "K4A camera_FPS : 30");}
	else if(inputSettings.DepthCameraMode == K4A_DEPTH_MODE_WFOV_UNBINNED){
		RCLCPP_INFO(this->get_logger(), "K4A depth_mode : WFOV_UNBINNED");
		RCLCPP_INFO(this->get_logger(), "K4A camera_FPS : 15");}
	else RCLCPP_INFO(this->get_logger(), "failed to read DepthMode.");
}

	// k4a_result_t startTracker();
	// k4a_result_t startImu();

	// void stopTracker();
	// void stopImu();

	// // Get camera calibration information for the depth camera
	// void getDepthCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);

	// void getRgbCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);

	// k4a_result_t getDepthFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& depth_frame, bool rectified);

	// k4a_result_t getPointCloud(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2>& point_cloud);

	// k4a_result_t getRgbPointCloudInRgbFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2>& point_cloud);
	// k4a_result_t getRgbPointCloudInDepthFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2>& point_cloud);

