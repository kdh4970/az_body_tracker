#include <array>
#include <iostream>
#include <map>
#include <vector>
#include <BodyTrackingHelpers.h>
#include <Utilities.h>

#include "TrackerNode.h"

bool debug=true;
sensor_msgs::msg::PointCloud2 point_cloud;
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
    std::cout << "Terminating the process..." << std::endl;
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    rclcpp::shutdown();
    
    // exit(0);
}

void VisualizeResult(k4abt_frame_t bodyFrame, k4a_transformation_t transformation, int depthWidth, int depthHeight, std::shared_ptr<TrackerNode> node) {
    // Obtain original capture that generates the body tracking result
    k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
    k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
    if(tfupdateRequire) {node->updateTransform();tfupdateRequire=false;}
    // pcl 
    rclcpp::Time capture_time = node->timestampToROS(node->get_device_timestamp(depthImage));
    k4a_result_t result = node->getPointCloud(transformation,depthImage, point_cloud, capture_time);

    if (result != K4A_RESULT_SUCCEEDED)
    {
    RCLCPP_ERROR(node->get_logger(),"Failed to get Point Cloud");
    rclcpp::shutdown();
    return;
    }
    
    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    if(debug) RCLCPP_INFO(node->get_logger(),"Detected numBodies: %d", numBodies);

    //gen msg
    azbt_msgs::msg::BtMultiArray msg;
    visualization_msgs::msg::MarkerArray markerArray;
    
    if(numBodies == 0){
        // std_msgs::Float32 msg;
        // msg.data = 0;
        // pub.publish(msg);
        azbt_msgs::msg::Bt elem;
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
                visualization_msgs::msg::Marker marker;
                node->getBodyMarker(body, marker, joint, capture_time);
                markerArray.markers.push_back(marker);
            }
        }
        
        k4a_float3_t JointHead=body.skeleton.joints[K4ABT_JOINT_HEAD].position;
        if(debug){
            // ROS_INFO("Joint PELVIS Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[0], body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[1], body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[2]);
            RCLCPP_INFO(node->get_logger(),"Joint HEAD Position: x: %f, y: %f, z: %f", JointHead.v[0], JointHead.v[1], JointHead.v[2]);
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
            RCLCPP_INFO(node->get_logger(),"world Head : %f, %f, %f, %f\n",world[0],world[1],world[2],world[3]);
            RCLCPP_INFO(node->get_logger(),"[body_length] : %f\n", world[2]*1000);
        }
        // publish data
        azbt_msgs::msg::Bt elem;
        elem.body_id= body.id;
        elem.length = ((world[2]*1000) / t.scale)+150;
        elem.location_x = world[0]*1000;
        elem.location_y = world[1]*1000;
        msg.data.push_back(elem);
        RCLCPP_INFO(node->get_logger(),"Target Length : %.2f",((world[2]*1000) / t.scale)+150);
    }


    node->bt_pub->publish(msg);
    node->pcl_pub->publish(point_cloud);
    node->body_marker_publisher->publish(markerArray);
    k4a_capture_release(originalCapture);
    k4a_image_release(depthImage);
}

void PlayFromDevice(InputSettings inputSettings, std::shared_ptr<TrackerNode> node, rclcpp::Rate rate) 
{
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!"); RCLCPP_INFO(node->get_logger(),"Open K4A Device succeeded.");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = inputSettings.DepthCameraMode;
    deviceConfig.camera_fps = inputSettings.cameraFPS;
    deviceConfig.color_resolution = inputSettings.colorResolution;

    // deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!"); RCLCPP_INFO(node->get_logger(),"Start K4A cameras succeeded.");

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
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!"); RCLCPP_INFO(node->get_logger(),"Body tracker initialization succeeded.");
    

    RCLCPP_INFO(node->get_logger(),"Body Tracker Started.");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok())
    {   
        bool kill_sign=false;
        ros::param::get("kill",kill_sign);
        if(kill_sign){kill_process(tracker,device);}
        executor.spin_once(std::chrono::milliseconds(0));
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
    rclcpp::init(argc,argv);
    InputSettings inputSettings;
    auto node = std::make_shared<TrackerNode>();

    RCLCPP_INFO("node started!");

    debug=node->debug;
    rclcpp::Rate rate(node->pub_hz);

    node->showInfo(inputSettings);
    PlayFromDevice(inputSettings, node,rate);
    
    rclcpp::shutdown();
    return 0;
}