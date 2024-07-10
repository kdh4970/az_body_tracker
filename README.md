# AzureKinect BodyTracker with ROS2  

### Released 240710  

## How to use  

### 0. Prerequisite  

ROS2 foxy  
Azure Kinect SDK - libk4a 1.4.1  
Azure Kinect Body Tracking SDK - libk4abt 1.1.2  

### 1. Clone the needed message repo and this repo into your ROS2 workspace.  

``` 
git clone -b foxy-devel https://github.com/microsoft/Azure_Kinect_ROS_Driver.git --single-branch
git clone -b foxy https://github.com/kdh4970/azbt_msgs.git --single_branch
git clone -b foxy https://github.com/kdh4970/az_body_tracker.git --single_branch
```  

### 2. You must change some paths for your system.  

azure_kinect_ros_driver_INCLUDE_DIRS in CMakeLists.txt  
preset_path, rviz configuration path in launch/run.launch.py  

### 3. Build.  

``` 
colcon build
``` 

### 4. Execute.

``` 
ros2 launch az_body_tracker run.launch.py
``` 