#include "TrackerNode.h"


TrackerNode::TrackerNode(ros::NodeHandle& _nh)
:debug(true),nh(_nh),pub_frame("depth_camera_link")
{
	bt_pub = nh.advertise<azbt_msgs::bt_data>("bt_result",1);
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("depth_pcl",1);
	body_marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("joint_marker",1);
}

TrackerNode::~TrackerNode()
{
	k4a_transformation_destroy(transformation);
}


void TrackerNode::pclbuffer(int depth_width, int depth_height)
{
	point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, depth_width, depth_height,
                                            depth_width * 3 * (int) sizeof(uint16_t));
}

void TrackerNode::showInfo(InputSettings& inputSettings){
	if(inputSettings.DepthCameraMode == K4A_DEPTH_MODE_NFOV_UNBINNED) {
		ROS_INFO("Depth Mode : NFOV_UNBINNED");
		ROS_INFO("Depth Resolution : 640 x 576");
		ROS_INFO("Camera FPS : 30");}
		// RCLCPP_INFO(this->get_logger(), "K4A depth_mode : NFOV_UNBINNED");
		// RCLCPP_INFO(this->get_logger(), "K4A camera_FPS : 30");}
	else if (inputSettings.DepthCameraMode == K4A_DEPTH_MODE_WFOV_2X2BINNED){
		ROS_INFO("Depth Mode : WFOV_2X2BINNED");
		ROS_INFO("Depth Resolution : 512 x 512");
		ROS_INFO("Camera FPS : 30");}

		// RCLCPP_INFO(this->get_logger(), "K4A depth_mode : WFOV_2X2BINNED");
		// RCLCPP_INFO(this->get_logger(), "K4A camera_FPS : 30");}
	else if(inputSettings.DepthCameraMode == K4A_DEPTH_MODE_WFOV_UNBINNED){
		ROS_INFO("Depth Mode : WFOV_UNBINNED");
		ROS_INFO("Depth Resolution : 1024 x 1024");
		ROS_INFO("Camera FPS : 15");}
		// RCLCPP_INFO(this->get_logger(), "K4A depth_mode : WFOV_UNBINNED");
		// RCLCPP_INFO(this->get_logger(), "K4A camera_FPS : 15");}
	else ROS_INFO("failed to read DepthMode.");
}

void TrackerNode::loadParams(Eigen::Matrix4f* Extr, Eigen::Matrix4f* DepthToColor, Eigen::Matrix4f* WorldToMap){
	if(this->nh.hasParam("/az_body_tracker/debug")){
		this->nh.getParam("/az_body_tracker/debug",this->debug);
	}
	else ROS_ERROR("Parameter debug does not exist!");

	if(this->nh.hasParam("/az_body_tracker/publish_hz")){
		this->nh.getParam("/az_body_tracker/publish_hz",this->pub_hz);
	}
	else ROS_ERROR("Parameter bt_publish_hz does not exist!");

	if(this->nh.hasParam("/az_body_tracker/WorldToColor")){
        std::vector<float> temp;
        // bool getParam(const std::string& key, std::vector<float>& vec) const;
        this->nh.getParam("/az_body_tracker/WorldToColor",temp);
        ROS_INFO("Reading Parameter '/az_body_tracker/WorldToColor'...");
        *Extr << temp[0],temp[1],temp[2],float(temp[3]),
                temp[4],temp[5],temp[6],float(temp[7]),
                temp[8],temp[9],temp[10],float(temp[11]),
                float(temp[12]),float(temp[13]),float(temp[14]),float(temp[15]);
        std::cout<< *Extr<< std::endl;
    }
    else ROS_ERROR("Parameter WorldToColor does not exist!");

	if(this->nh.hasParam("/az_body_tracker/DepthToColor")){
        std::vector<float> temp;
        // bool getParam(const std::string& key, std::vector<float>& vec) const;
        this->nh.getParam("/az_body_tracker/DepthToColor",temp);
        ROS_INFO("Reading Parameter '/az_body_tracker/DepthToColor'...");
        *DepthToColor << temp[0],temp[1],temp[2],temp[3],
                temp[4],temp[5],temp[6],temp[7],
                temp[8],temp[9],temp[10],temp[11],
                float(temp[12]),float(temp[13]),float(temp[14]),float(temp[15]);
        std::cout<< *DepthToColor<< std::endl;
    }
    else ROS_ERROR("Parameter DepthToColor does not exist!");

	if(this->nh.hasParam("/az_body_tracker/WorldToMap")){
        std::vector<float> temp;
        // bool getParam(const std::string& key, std::vector<float>& vec) const;
        this->nh.getParam("/az_body_tracker/WorldToMap",temp);
        ROS_INFO("Reading Parameter '/az_body_tracker/WorldToMap'...");
        *WorldToMap << float(temp[0]),float(temp[1]),float(temp[2]),float(temp[3]),
						float(temp[4]),float(temp[5]),float(temp[6]),float(temp[7]),
						float(temp[8]),float(temp[9]),float(temp[10]),float(temp[11]),
						float(temp[12]),float(temp[13]),float(temp[14]),float(temp[15]);
        std::cout<< *WorldToMap<< std::endl;
    }
    else ROS_ERROR("Parameter WorldToMap does not exist!");
}

std::chrono::microseconds TrackerNode::get_device_timestamp(const k4a_image_t k4a_depth_img)
{
	return std::chrono::microseconds(k4a_image_get_device_timestamp_usec(k4a_depth_img));
}

void TrackerNode::updateTransform(transformData input){
	
	Eigen::AngleAxisd rollAngle(input.roll*(PI/180.0f), Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(input.yaw*(PI/180.0f), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(input.pitch*(PI/180.0f), Eigen::Vector3d::UnitX());
	Eigen::Matrix4f temp;
	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
	constexpr float unit_mtomm = 1.0 / 1000.0f;
	Eigen::Matrix3d rotationMatrix = q.matrix();
	temp << rotationMatrix(0), rotationMatrix(1), rotationMatrix(2), input.x,
				rotationMatrix(3), rotationMatrix(4), rotationMatrix(5), input.y,
				rotationMatrix(6), rotationMatrix(7), rotationMatrix(8), input.z,
				0.0, 0.0, 0.0, 1.0;
	scale << input.scale * unit_mtomm, 0.0, 0.0, 0.0,
			0.0, input.scale * unit_mtomm, 0.0, 0.0,
			0.0, 0.0, input.scale * unit_mtomm, 0.0,
			0.0, 0.0, 0.0, 1.0;
	transform = scale * temp;
}

k4a_result_t TrackerNode::getBodyMarker(const k4abt_body_t& body, visualization_msgs::MarkerPtr marker_msg, int jointType, ros::Time capture_time)
{
	k4a_float3_t position = body.skeleton.joints[jointType].position;
	k4a_quaternion_t orientation = body.skeleton.joints[jointType].orientation;
	Eigen::Vector4f temp;
	marker_msg->header.frame_id = pub_frame;
	marker_msg->header.stamp = capture_time;

	// Set the lifetime to 0.25 to prevent flickering for even 5fps configurations.
	// New markers with the same ID will replace old markers as soon as they arrive.
	marker_msg->lifetime = ros::Duration(0.25);
	marker_msg->id = body.id * 100 + jointType;
	marker_msg->type = visualization_msgs::Marker::SPHERE;

	Color color = BODY_COLOR_PALETTE[body.id % BODY_COLOR_PALETTE.size()];

	marker_msg->color.a = color.a;
	marker_msg->color.r = color.r;
	marker_msg->color.g = color.g;
	marker_msg->color.b = color.b;

	marker_msg->scale.x = 0.05;
	marker_msg->scale.y = 0.05;
	marker_msg->scale.z = 0.05;
	
	constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
	temp << position.v[0],
			position.v[1],
			position.v[2],
			1.0f;

	temp=transform*temp;

	marker_msg->pose.position.x = temp[0];
	marker_msg->pose.position.y = temp[1];
	marker_msg->pose.position.z = temp[2];
	marker_msg->pose.orientation.w = orientation.wxyz.w;
	marker_msg->pose.orientation.x = orientation.wxyz.x;
	marker_msg->pose.orientation.y = orientation.wxyz.y;
	marker_msg->pose.orientation.z = orientation.wxyz.z;

	return K4A_RESULT_SUCCEEDED;
}

k4a_result_t TrackerNode::getPointCloud(const k4a_transformation_t tf_handle , const k4a_image_t k4a_depth_img, sensor_msgs::PointCloud2Ptr& point_cloud, ros::Time capture_time)
{
	point_cloud->header.frame_id = pub_frame;
	point_cloud->header.stamp = capture_time;
	// Tranform depth image to point cloud
	// k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
    //                                                     &point_cloud_image);
    k4a_result_t result = k4a_transformation_depth_image_to_point_cloud(tf_handle, k4a_depth_img, K4A_CALIBRATION_TYPE_DEPTH, this->point_cloud_image.handle());
	if (K4A_RESULT_SUCCEEDED != result)
	{
		ROS_ERROR("Failed to transform depth image to point cloud!");
	}
	return fillPointCloud(this->point_cloud_image, point_cloud);
}

ros::Time TrackerNode::timestampToROS(const std::chrono::microseconds& k4a_timestamp_us)
{
  // This will give INCORRECT timestamps until the first image.
	if (device_to_realtime_offset_.count() == 0)
	{
		initializeTimestampOffset(k4a_timestamp_us);
	}

	std::chrono::nanoseconds timestamp_in_realtime = k4a_timestamp_us + device_to_realtime_offset_;
	ros::Time ros_time;
	ros_time.fromNSec(timestamp_in_realtime.count());
	return ros_time;
}

void TrackerNode::initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us)
{
  // We have no better guess than "now".
	std::chrono::nanoseconds realtime_clock = std::chrono::system_clock::now().time_since_epoch();

	device_to_realtime_offset_ = realtime_clock - k4a_device_timestamp_us;

	ROS_WARN_STREAM("Initializing the device to realtime offset based on wall clock: "
					<< device_to_realtime_offset_.count() << " ns");
}

k4a_result_t TrackerNode::fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2Ptr& point_cloud)
{
	point_cloud->height = pointcloud_image.get_height_pixels();
	point_cloud->width = pointcloud_image.get_width_pixels();
	point_cloud->is_dense = false;
	point_cloud->is_bigendian = false;
	Eigen::Vector4f temp,tf;
	const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();

	sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
	pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

	sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

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