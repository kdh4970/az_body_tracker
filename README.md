# az_body_tracker  
  
## info  
This is ros2 pkg which is publish below 3 into ros2 topic by using Azure Kinect Body Tracking SDK.  
  PointCloud(sensor_msgs/msg/pointcloud2) : from Azure Kinect dpeth image  
  Marker(visualization_msgs/msg/markerarray)  : from Azure Kinect Body Tracking data  
  Custom(azbt_msgs/msg/bt_data) : to use related project-collabot 

## dependencies  
Azure Kinect SDK 1.4.1  
Azure Kinect Body Tracking SDK 1.1.2  
Azure Kinect ROS Driver  
Eigen3  
glfw3 (optional for window_controller_3d)

