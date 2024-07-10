# AzureKinect BodyTracker with ROS1  

### Released 230926  

## How to use  

### 0. Prerequisite  

ROS1 Melodic
Azure Kinect SDK - libk4a 1.4.1
Azure Kinect Body Tracking SDK - libk4abt 1.1.2

### 1. Clone the needed message repo and this repo into your ROS workspace.  

``` 
git clone -b melodic https://github.com/microsoft/Azure_Kinect_ROS_Driver.git --single-branch
git clone https://github.com/kdh4970/azbt_msgs.git
git clone https://github.com/kdh4970/az_body_tracker.git
```  

### 2. You must change some paths for your system.  

azure_kinect_ros_driver_INCLUDE_DIRS in CMakeLists.txt  
preset_path, rviz configuration path in launch/run.launch

### 3. Build.  

``` 
catkin_make
``` 

### 4. Execute.

``` 
roslaunch az_body_tracker run.launch
``` 