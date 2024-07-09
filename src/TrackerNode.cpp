#include "TrackerNode.h"


TrackerNode::TrackerNode()
:debug(true), pub_frame("map"), Node("az_body_tracker")
{
	bt_pub = this->create_publisher<azbt_msgs::msg::BtMultiArray>("bt_result",1);
	pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("depth_pcl",1);
	body_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("joint_marker",1);

	this->declare_parameter("debug", debug);
	this->declare_parameter("publish_hz", pub_hz);
	this->declare_parameter("roll", 0.0f);
	this->declare_parameter("pitch", 0.0f);
	this->declare_parameter("yaw", 0.0f);
	this->declare_parameter("x", 0.0f);
	this->declare_parameter("y", 0.0f);
	this->declare_parameter("z", 0.0f);
	this->declare_parameter("scale", 1.0f);

	this->set_on_parameters_set_callback(std::bind(&TrackerNode::parametersCallback, this, std::placeholders::_1));

	readTransformPreset();
}

TrackerNode::~TrackerNode()
{
	k4a_transformation_destroy(transformation);
}

void TrackerNode::readTransformPreset()
{
	// may throw ament_index_cpp::PackageNotFoundError exception
	std::string presetPath = ament_index_cpp::get_package_share_directory("az_body_tracker")+"/params/preset.yaml";
    RCLCPP_INFO(this->get_logger(),"Reading Saved Preset... : %s",presetPath.c_str());
    std::ifstream readFile(presetPath);
    if(readFile.is_open()){
        RCLCPP_INFO("File reading success");
        std::string line;
        while(getline(readFile,line))
        {
            std::istringstream ss(line);
            std::string preset_id;
            float preset_val;
            ss >> preset_id >> preset_val;
            std::cout << preset_id << preset_val<<std::endl;
            if(preset_id=="roll:") {
				_t.roll = preset_val;
				rclcpp::Parameter param("roll", preset_val);
				this->set_parameter(param);
			}
			else if(preset_id=="pitch:") {
				_t.pitch = preset_val;
				rclcpp::Parameter param("pitch", preset_val);
				this->set_parameter(param);
			}
			else if(preset_id=="yaw:") {
				_t.yaw = preset_val;
				rclcpp::Parameter param("yaw", preset_val);
				this->set_parameter(param);
			}
			else if(preset_id=="x:") {
				_t.x = preset_val;
				rclcpp::Parameter param("x", preset_val);
				this->set_parameter(param);
			}
			else if(preset_id=="y:") {
				_t.y = preset_val;
				rclcpp::Parameter param("y", preset_val);
				this->set_parameter(param);
			}
			else if(preset_id=="z:") {
				_t.z = preset_val;
				rclcpp::Parameter param("z", preset_val);
				this->set_parameter(param);
			}
			else if(preset_id=="scale:") {
				_t.scale = preset_val;
				rclcpp::Parameter param("scale", preset_val);
				this->set_parameter(param);
			}
        }
        readFile.close();
    }
    else RCLCPP_ERROR(this->get_logger(),"Failed to open preset file!");
}


void TrackerNode::pclbuffer(int depth_width, int depth_height)
{
	point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, depth_width, depth_height,
                                            depth_width * 3 * (int) sizeof(uint16_t));
}

void TrackerNode::showInfo(InputSettings& inputSettings){
	if(inputSettings.DepthCameraMode == K4A_DEPTH_MODE_NFOV_UNBINNED) {
		RCLCPP_INFO(this->get_logger(),"Depth Mode : NFOV_UNBINNED");
		RCLCPP_INFO(this->get_logger(),"Depth Resolution : 640 x 576");
		RCLCPP_INFO(this->get_logger(),"Camera FPS : 30");}
		// RCLCPP_INFO(this->get_logger(), "K4A depth_mode : NFOV_UNBINNED");
		// RCLCPP_INFO(this->get_logger(), "K4A camera_FPS : 30");}
	else if (inputSettings.DepthCameraMode == K4A_DEPTH_MODE_WFOV_2X2BINNED){
		RCLCPP_INFO(this->get_logger(),"Depth Mode : WFOV_2X2BINNED");
		RCLCPP_INFO(this->get_logger(),"Depth Resolution : 512 x 512");
		RCLCPP_INFO(this->get_logger(),"Camera FPS : 30");}

		// RCLCPP_INFO(this->get_logger(), "K4A depth_mode : WFOV_2X2BINNED");
		// RCLCPP_INFO(this->get_logger(), "K4A camera_FPS : 30");}
	else if(inputSettings.DepthCameraMode == K4A_DEPTH_MODE_WFOV_UNBINNED){
		RCLCPP_INFO(this->get_logger(),"Depth Mode : WFOV_UNBINNED");
		RCLCPP_INFO(this->get_logger(),"Depth Resolution : 1024 x 1024");
		RCLCPP_INFO(this->get_logger(),"Camera FPS : 15");}
		// RCLCPP_INFO(this->get_logger(), "K4A depth_mode : WFOV_UNBINNED");
		// RCLCPP_INFO(this->get_logger(), "K4A camera_FPS : 15");}
	else RCLCPP_INFO(this->get_logger(),"failed to read DepthMode.");
}

std::chrono::microseconds TrackerNode::get_device_timestamp(const k4a_image_t k4a_depth_img)
{
	return std::chrono::microseconds(k4a_image_get_device_timestamp_usec(k4a_depth_img));
}

void TrackerNode::updateTransform(){
	
	Eigen::AngleAxisd rollAngle(_t.roll*(PI/180.0f), Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(_t.yaw*(PI/180.0f), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(_t.pitch*(PI/180.0f), Eigen::Vector3d::UnitX());
	Eigen::Matrix4f temp;
	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
	constexpr float unit_mtomm = 1.0 / 1000.0f;
	Eigen::Matrix3d rotationMatrix = q.matrix();
	temp << rotationMatrix(0), rotationMatrix(1), rotationMatrix(2), _t.x,
				rotationMatrix(3), rotationMatrix(4), rotationMatrix(5), _t.y,
				rotationMatrix(6), rotationMatrix(7), rotationMatrix(8), _t.z,
				0.0, 0.0, 0.0, 1.0;
	scale << _t.scale * unit_mtomm, 0.0, 0.0, 0.0,
			0.0, _t.scale * unit_mtomm, 0.0, 0.0,
			0.0, 0.0, _t.scale * unit_mtomm, 0.0,
			0.0, 0.0, 0.0, 1.0;
	transform = scale * temp;
}

k4a_result_t TrackerNode::getBodyMarker(const k4abt_body_t& body, visualization_msgs::msg::Marker& marker_msg, int jointType, rclcpp::Time capture_time)
{
	k4a_float3_t position = body.skeleton.joints[jointType].position;
	k4a_quaternion_t orientation = body.skeleton.joints[jointType].orientation;
	Eigen::Vector4f temp;
	marker_msg.header.frame_id = pub_frame;
	marker_msg.header.stamp = capture_time;

	// Set the lifetime to 0.25 to prevent flickering for even 5fps configurations.
	// New markers with the same ID will replace old markers as soon as they arrive.
	marker_msg.lifetime = rclcpp::Duration(0.25);
	marker_msg.id = body.id * 100 + jointType;
	marker_msg.type = visualization_msgs::Marker::SPHERE;

	Color color = BODY_COLOR_PALETTE[body.id % BODY_COLOR_PALETTE.size()];

	marker_msg.color.a = color.a;
	marker_msg.color.r = color.r;
	marker_msg.color.g = color.g;
	marker_msg.color.b = color.b;

	marker_msg.scale.x = 0.05;
	marker_msg.scale.y = 0.05;
	marker_msg.scale.z = 0.05;
	
	temp << position.v[0],
			position.v[1],
			position.v[2],
			1.0f;

	temp=transform*temp;

	marker_msg.pose.position.x = temp[0];
	marker_msg.pose.position.y = temp[1];
	marker_msg.pose.position.z = temp[2];
	marker_msg.pose.orientation.w = orientation.wxyz.w;
	marker_msg.pose.orientation.x = orientation.wxyz.x;
	marker_msg.pose.orientation.y = orientation.wxyz.y;
	marker_msg.pose.orientation.z = orientation.wxyz.z;

	return K4A_RESULT_SUCCEEDED;
}

k4a_result_t TrackerNode::getPointCloud(const k4a_transformation_t tf_handle , const k4a_image_t k4a_depth_img, sensor_msgs::msg::PointCloud2& point_cloud, rclcpp::Time capture_time)
{
	point_cloud.header.frame_id = pub_frame;
	point_cloud.header.stamp = capture_time;
	// Tranform depth image to point cloud
	// k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
    //                                                     &point_cloud_image);
    k4a_result_t result = k4a_transformation_depth_image_to_point_cloud(tf_handle, k4a_depth_img, K4A_CALIBRATION_TYPE_DEPTH, this->point_cloud_image.handle());
	if (K4A_RESULT_SUCCEEDED != result)
	{
		RCLCPP_ERROR(this->get_logger(),"Failed to transform depth image to point cloud!");
	}
	return fillPointCloud(this->point_cloud_image, point_cloud);
}

rclcpp::Time TrackerNode::timestampToROS(const std::chrono::microseconds& k4a_timestamp_us)
{
	// This will give INCORRECT timestamps until the first image.
	if (device_to_realtime_offset_.count() == 0)
	{
		initializeTimestampOffset(k4a_timestamp_us);
	}

	std::chrono::nanoseconds timestamp_in_realtime = k4a_timestamp_us + device_to_realtime_offset_;
	rclcpp::Time ros_time(timestamp_in_realtime.count(), RCL_ROS_TIME);
	return ros_time;
}

void TrackerNode::initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us)
{
  // We have no better guess than "now".
	std::chrono::nanoseconds realtime_clock = std::chrono::system_clock::now().time_since_epoch();

	device_to_realtime_offset_ = realtime_clock - k4a_device_timestamp_us;

	RCLCPP_WARN(this->get_logger(),"Initializing the device to realtime offset based on wall clock: "
					<< device_to_realtime_offset_.count() << " ns");
}

k4a_result_t TrackerNode::fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::msg::PointCloud2& point_cloud)
{
	point_cloud.height = pointcloud_image.get_height_pixels();
	point_cloud.width = pointcloud_image.get_width_pixels();
	point_cloud.is_dense = false;
	point_cloud.is_bigendian = false;
	Eigen::Vector4f temp,tf;
	const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();

	sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud);
	pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

	sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");

	pcd_modifier.resize(point_count);

	const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());

	for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
	{
    	float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

		if (z <= 0.0f)
		{
    	  *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
		}
		else
		{
			// do added
			temp << static_cast<float>(point_cloud_buffer[3 * i + 0]),
					static_cast<float>(point_cloud_buffer[3 * i + 1]),
					z,
					1.0f;

			tf = transform * temp ;
			*iter_x = tf[0];
			*iter_y = tf[1];
			*iter_z = tf[2];



		}
	}

	return K4A_RESULT_SUCCEEDED;
}