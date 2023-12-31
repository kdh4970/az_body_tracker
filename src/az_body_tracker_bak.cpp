#include <array>
#include <iostream>
#include <map>
#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>
#include "TrackerNode.h"

Eigen::Matrix4f Transform,Extr,ExtrDtoC,WorldtoMap;
bool debug=true;
sensor_msgs::PointCloud2Ptr point_cloud(new sensor_msgs::PointCloud2);
transformData t;

bool tfupdateRequire = true;
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;
uint16_t frame_count = 0;

void PrintAppUsage()
{
    printf("\n");
    printf(" Basic Navigation:\n\n");
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
    printf("\n");
    printf(" Key Shortcuts\n\n");
    printf(" ESC: quit\n");
    printf(" h: help\n");
    printf(" b: body visualization mode\n");
    printf(" k: 3d window layout\n");
    printf("\n");
}

int64_t CloseCallback(void* /*context*/)
{
    ros::shutdown();
    return 1;
}

int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        ros::shutdown();
        break;
    case GLFW_KEY_K:
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_H:
        PrintAppUsage();
        break;
    case GLFW_KEY_D:
        debug = !debug;
        break;
    }
    return 1;
}

void param_callback(az_body_tracker::az_body_trackerConfig &config, uint32_t level)
{
    t.roll=config.roll;
	t.pitch=config.pitch;
	t.yaw=config.yaw;
    t.x=config.x;
    t.y=config.y;
    t.z=config.z;
	if(debug) ROS_INFO("parameter callback >> roll %f, pitch %f, yaw %f",t.roll,t.pitch,t.yaw);
    tfupdateRequire = true;
}

void VisualizeResult(k4abt_frame_t bodyFrame, k4a_transformation_t transformation, Window3dWrapper& window3d, int depthWidth, int depthHeight, std::shared_ptr<TrackerNode> node) {

    // Obtain original capture that generates the body tracking result
    k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
    k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
    std::vector<Color> pointCloudColors(depthWidth * depthHeight, { 1.f, 1.f, 1.f, 1.f });
    if(tfupdateRequire) {node->updateTransform(t);tfupdateRequire=false;}
    // pcl 
    ros::Time capture_time = node->timestampToROS(node->get_device_timestamp(depthImage));
    k4a_result_t result = node->getPointCloud(transformation,depthImage, point_cloud, capture_time);

    if (result != K4A_RESULT_SUCCEEDED)
    {
    ROS_ERROR_STREAM("Failed to get Point Cloud");
    ros::shutdown();
    return;
    }
    

    // Read body index map and assign colors
    k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
    const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);

    
    for (int i = 0; i < depthWidth * depthHeight; i++)
    {
        uint8_t bodyIndex = bodyIndexMapBuffer[i];
        if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
        {
            uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
            pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
        }
    }
    k4a_image_release(bodyIndexMap);

    // Visualize point cloud
    window3d.UpdatePointClouds(depthImage, pointCloudColors);

    // Visualize the skeleton data
    window3d.CleanJointsAndBones();
    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    if(debug) ROS_INFO( "Detected numBodies: %d", numBodies);

    //gen msg
    azbt_msgs::bt_data msg;
    visualization_msgs::MarkerArrayPtr markerArrayPtr(new visualization_msgs::MarkerArray);

    if(numBodies == 0){
        // std_msgs::Float32 msg;
        // msg.data = 0;
        // pub.publish(msg);
        azbt_msgs::Elem elem;
        elem.body_id= 0;
        elem.length = 0;
        elem.location_x = 0;
        elem.location_y = 0;
        msg.data.push_back(elem);
    }

    for (uint32_t i = 0; i < numBodies; i++)
    {
        k4abt_body_t body;
        VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton), "Get skeleton from body frame failed!");
        body.id = k4abt_frame_get_body_id(bodyFrame, i);

        // Assign the correct color based on the body id
        Color color = g_bodyColors[body.id % g_bodyColors.size()];
        color.a = 0.4f;
        Color lowConfidenceColor = color;
        lowConfidenceColor.a = 0.1f;

        Color color2; // pelvis
        color2.r = 1.0f; color2.g = 0.0f; color2.b = 0.0f; color2.a = 1.0f;

        Color color3; // head
        color3.r = 0.0f; color3.g = 1.0f; color3.b = 0.0f; color3.a = 1.0f;

        Color color4; // foot
        color4.r = 0.0f; color4.g = 0.0f; color4.b = 1.0f; color4.a = 1.0f;

        if(debug) ROS_INFO( "Body ID: %d", body.id);
        // Visualize joints
        for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
        {
            if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
            {
                const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint%d Position: x: %f, y: %f, z: %f", joint, jointPosition.v[0], jointPosition.v[1], jointPosition.v[2]);
                // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint%d Orient: x: %f, y: %f, z: %f, w: %f", joint, jointOrientation.v[0], jointOrientation.v[1], jointOrientation.v[2], jointOrientation.v[3]);
                window3d.AddJoint(
                    jointPosition,
                    jointOrientation,
                    body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor);
            }
            
            visualization_msgs::MarkerPtr markerPtr(new visualization_msgs::Marker);
            node->getBodyMarker(body, markerPtr, i, joint, capture_time);
            markerArrayPtr->markers.push_back(*markerPtr);

        }
        // for debug 
        window3d.AddJoint(
                    body.skeleton.joints[K4ABT_JOINT_PELVIS].position,
                    body.skeleton.joints[K4ABT_JOINT_PELVIS].orientation,
                    body.skeleton.joints[K4ABT_JOINT_PELVIS].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color2 : lowConfidenceColor);
        window3d.AddJoint(
                    body.skeleton.joints[K4ABT_JOINT_HEAD].position,
                    body.skeleton.joints[K4ABT_JOINT_HEAD].orientation,
                    body.skeleton.joints[K4ABT_JOINT_HEAD].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color3 : lowConfidenceColor);
        window3d.AddJoint(
                    body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position,
                    body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].orientation,
                    body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color4 : lowConfidenceColor);
        window3d.AddJoint(
                    body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position,
                    body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].orientation,
                    body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color4 : lowConfidenceColor);
        k4a_float3_t JointHead=body.skeleton.joints[K4ABT_JOINT_HEAD].position;
        if(debug){
            // ROS_INFO("Joint PELVIS Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[0], body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[1], body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[2]);
            ROS_INFO("Joint HEAD Position: x: %f, y: %f, z: %f", JointHead.v[0], JointHead.v[1], JointHead.v[2]);
            // ROS_INFO("Joint LEFTFOOT Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position.v[0], body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position.v[1], body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position.v[2]);
            // ROS_INFO("Joint RIGHTFOOT Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position.v[0], body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position.v[1], body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position.v[2]);
        }

        Eigen::Vector4f world, DepthJointHead;
        // k4a_float3_t joint_in_color_3d;
        // transform_joint_from_depth_3d_to_color_3d(
        //     &sensor_calibration, 
        //     body.skeleton.joints[K4ABT_JOINT_HEAD].position,
        //     joint_in_color_3d);
            
        DepthJointHead << JointHead.v[0],JointHead.v[1],JointHead.v[2],1.0f;
        // world = Extr.inverse() * DepthJointHead;
        world = node->transform * DepthJointHead;
        


        // estimate body length in world coordinate : ground_to_chessboard + chessboard_to_head + head_to_top
        float world_length = 780.0f + world[2] + 150.0f;
        if(debug){
            ROS_INFO("world Head : %f, %f, %f, %f\n",world[0],world[1],world[2],world[3]);
            ROS_INFO("[body_length] : %f\n", world_length);
        }
        // publish data
        azbt_msgs::Elem elem;
        elem.body_id= body.id;
        elem.length = world_length;
        elem.location_x = 0;
        elem.location_y = 0;
        msg.data.push_back(elem);
        

        // Visualize bones
        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
        {
            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

            if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
                body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
            {
                bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                    body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor);
            }
        }
    }


    node->bt_pub.publish(msg);
    node->pcl_pub.publish(point_cloud);
    node->body_marker_publisher.publish(markerArrayPtr);
    k4a_capture_release(originalCapture);
    k4a_image_release(depthImage);
}

void PlayFromDevice(InputSettings inputSettings, std::shared_ptr<TrackerNode> node, ros::Rate rate) 
{
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!"); ROS_INFO("Open K4A Device succeeded!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = inputSettings.DepthCameraMode;
    deviceConfig.camera_fps = inputSettings.cameraFPS;
    deviceConfig.color_resolution = inputSettings.colorResolution;

    // deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!"); ROS_INFO("Start K4A cameras succeeded!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;
    node->pclbuffer(depthWidth,depthHeight);
    k4a_transformation_t transformation = k4a_transformation_create(&sensorCalibration);

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = inputSettings.processingMode;
    // trackerConfig.model_path = inputSettings.ModelPath.c_str();
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!"); ROS_INFO("Body tracker initialization succeeded!");

    // Initialize the 3d window controller
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);
    ROS_INFO("3D visualization window initialization succeeded!");
    
    ROS_INFO("Start tracking!");
    while (ros::ok())
    {
        ros::spinOnce();
        // ROS_INFO("======================= FRAME %d =======================", frame_count);
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0
        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // timeout_in_ms is set to 0. Return immediately no matter whether the sensorCapture is successfully added
            // to the queue or not.
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(sensorCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            /************* Successfully get a body tracking result, process the result here ***************/
            VisualizeResult(bodyFrame,transformation, window3d, depthWidth, depthHeight, node);
            //Release the bodyFrame
            k4abt_frame_release(bodyFrame);
        }
        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
        frame_count++;
        rate.sleep();
    }

    std::cout << "Finished body tracking processing!" << std::endl;

    window3d.Delete();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
}




int main(int argc, char** argv)
{
    ros::init(argc,argv, "az_body_tracker");
    InputSettings inputSettings;
    ros::NodeHandle nh;
    auto node = std::make_shared<TrackerNode>(nh,inputSettings);

    dynamic_reconfigure::Server<az_body_tracker::az_body_trackerConfig> server;
    dynamic_reconfigure::Server<az_body_tracker::az_body_trackerConfig>::CallbackType f;
    f=boost::bind(&param_callback,_1,_2);
    server.setCallback(f);
    ROS_INFO("node started!");
    node->loadParams(&Extr,&ExtrDtoC,&WorldtoMap); 
    // Transform = WorldtoMap * Extr.inverse() * ExtrDtoC;
    debug=node->debug;
    ros::Rate rate(node->pub_hz);

    node->showInfo(inputSettings);
    PlayFromDevice(inputSettings, node,rate);
    
    ROS_INFO("Program Stopped!\n");
    ros::shutdown();
    return 0;
}