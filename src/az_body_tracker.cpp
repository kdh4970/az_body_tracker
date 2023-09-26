#include <array>
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <map>
#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include "TrackerNode.h"

Eigen::Matrix4f Transform,Extr,ExtrDtoC,WorldtoMap;
bool debug=true;
sensor_msgs::PointCloud2Ptr point_cloud(new sensor_msgs::PointCloud2);
transformData t;
uint16_t frame_count = 0;
bool tfupdateRequire = true;
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

void kill_process(k4abt_tracker_t tracker, k4a_device_t device){
    ROS_INFO("Killing Process...");
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    ros::shutdown();
    
    // exit(0);
}

void readTransformPreset(ros::NodeHandle nh)
{
    std::string presetPath = ros::package::getPath("az_body_tracker")+"/params/preset.yaml";
    ROS_INFO("Reading Saved Preset... : %s",presetPath.c_str());
    std::ifstream readFile(presetPath);
    if(readFile.is_open()){
        ROS_INFO("File reading success");
        std::string line;
        while(getline(readFile,line))
        {
            std::istringstream ss(line);
            std::string preset_id;
            float preset_val;
            ss >> preset_id >> preset_val;
            std::cout << preset_id << preset_val<<std::endl;
            if(preset_id=="roll:") {t.roll = preset_val; nh.setParam("az_body_tracker/roll",preset_val);}
            else if(preset_id=="pitch:") {t.pitch=preset_val; nh.setParam("az_body_tracker/pitch",preset_val);}
            else if(preset_id=="yaw:") {t.yaw=preset_val; nh.setParam("az_body_tracker/yaw",preset_val);}
            else if(preset_id=="x:") {t.x=preset_val; nh.setParam("az_body_tracker/x",preset_val);}
            else if(preset_id=="y:") {t.y=preset_val; nh.setParam("az_body_tracker/y",preset_val);}
            else if(preset_id=="z:") {t.z=preset_val; nh.setParam("az_body_tracker/z",preset_val);}
            else if(preset_id=="scale:") {t.scale=preset_val; nh.setParam("az_body_tracker/scale",preset_val);}
        }
        readFile.close();
    }
    else ROS_ERROR("Open failure!");
}

void param_callback(az_body_tracker::az_body_trackerConfig &config, uint32_t level)
{
    t.roll=config.roll;
	t.pitch=config.pitch;
	t.yaw=config.yaw;
    t.x=config.x;
    t.y=config.y;
    t.z=config.z;
    t.scale=config.scale;
	if(debug) ROS_INFO("parameter callback >> roll %f, pitch %f, yaw %f",t.roll,t.pitch,t.yaw);
    tfupdateRequire = true;
}

void VisualizeResult(k4abt_frame_t bodyFrame, k4a_transformation_t transformation, int depthWidth, int depthHeight, std::shared_ptr<TrackerNode> node) {
    // Obtain original capture that generates the body tracking result
    k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
    k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
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

        // Visualize joints
        for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
        {
            if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
            {
                visualization_msgs::MarkerPtr markerPtr(new visualization_msgs::Marker);
                node->getBodyMarker(body, markerPtr, joint, capture_time);
                markerArrayPtr->markers.push_back(*markerPtr);
            }
        }
        
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
        if(debug){
            ROS_INFO("world Head : %f, %f, %f, %f\n",world[0],world[1],world[2],world[3]);
            ROS_INFO("[body_length] : %f\n", world[2]*1000);
        }
        // publish data
        azbt_msgs::Elem elem;
        elem.body_id= body.id;
        elem.length = ((world[2]*1000) / t.scale)+100;
        elem.location_x = world[0]*1000;
        elem.location_y = world[1]*1000;
        msg.data.push_back(elem);
        ROS_INFO("Target Length : %.2f",((world[2]*1000) / t.scale)+100);
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
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!"); ROS_INFO("Open K4A Device succeeded.");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = inputSettings.DepthCameraMode;
    deviceConfig.camera_fps = inputSettings.cameraFPS;
    deviceConfig.color_resolution = inputSettings.colorResolution;

    // deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!"); ROS_INFO("Start K4A cameras succeeded.");

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
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!"); ROS_INFO("Body tracker initialization succeeded.");
    

    ROS_INFO("Body Tracker Started.");
    while (ros::ok())
    {   
        bool kill_sign=false;
        ros::param::get("kill",kill_sign);
        if(kill_sign){kill_process(tracker,device);}
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
            VisualizeResult(bodyFrame,transformation, depthWidth, depthHeight, node);
            //Release the bodyFrame
            k4abt_frame_release(bodyFrame);
        }
        frame_count++;
        rate.sleep();
    }

    std::cout << "Finished body tracking processing!" << std::endl;
    
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
    auto node = std::make_shared<TrackerNode>(nh);

    dynamic_reconfigure::Server<az_body_tracker::az_body_trackerConfig> server;
    dynamic_reconfigure::Server<az_body_tracker::az_body_trackerConfig>::CallbackType f;
    f=boost::bind(&param_callback,_1,_2);
    server.setCallback(f);
    readTransformPreset(nh);
    ROS_INFO("node started!");
    node->loadParams(&Extr,&ExtrDtoC,&WorldtoMap); 
    // Transform = WorldtoMap * Extr.inverse() * ExtrDtoC;
    debug=node->debug;
    ros::Rate rate(node->pub_hz);

    node->showInfo(inputSettings);
    PlayFromDevice(inputSettings, node,rate);
    
    ros::shutdown();
    return 0;
}