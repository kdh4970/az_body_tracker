#include <array>
#include <iostream>
#include <map>
#include <vector>
#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>
#include <eigen3/Eigen/Dense>

#include "TrackerNode.h"
#include <std_msgs/Float32.h>

Eigen::Matrix4f Extr;
Eigen::Matrix4f Intr;

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool debug = true;
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

bool ParseInputSettingsFromArg(int argc, char** argv, InputSettings& inputSettings)
{
    for (int i = 1; i < argc; i++)
    {
        std::string inputArg(argv[i]);
        if (inputArg == std::string("NFOV_UNBINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        else if (inputArg == std::string("WFOV_BINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if (inputArg == std::string("WFOV_UNBINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_UNBINNED;
            inputSettings.cameraFPS = K4A_FRAMES_PER_SECOND_15;
        }
        else
        {
            printf("Error: command not understood: %s\n", inputArg.c_str());
            return false;
        }
    }
    return true;
}

bool isOutlier(const k4a_float3_t& HeadPosition, const k4a_float3_t& FootPosition){
    float distance = 0;
    distance = sqrt(pow(HeadPosition.v[0] - FootPosition.v[0], 2) + pow(HeadPosition.v[1] - FootPosition.v[1], 2) + pow(HeadPosition.v[2] - FootPosition.v[2], 2));
    if(distance >= 2000.0) return true;
    else return false;
}

float calculateBodyLength(const k4a_float3_t& HeadPosition, const k4a_float3_t& PelvisPosition, const k4a_float3_t& LetfFootPosition, const k4a_float3_t& RightFootPosition){
    float length = 0;
    if(isOutlier(HeadPosition, LetfFootPosition) || isOutlier(HeadPosition, RightFootPosition)) ROS_INFO("Outlier detected");
    else{
        std::vector<float> centerFoot {(LetfFootPosition.v[0]+RightFootPosition.v[0])/2,(LetfFootPosition.v[1]+RightFootPosition.v[1])/2,(LetfFootPosition.v[2]+RightFootPosition.v[2])/2};
        length = sqrt(pow(HeadPosition.v[0] - centerFoot[0], 2) + pow(HeadPosition.v[1] - centerFoot[1], 2) + pow(HeadPosition.v[2] - centerFoot[2], 2));
    }
    ROS_INFO("## calculated Body length: %.3fmm   %.1fcm", length, length/10);
    return length;
}

void print_body_index_map_middle_line(k4a_image_t body_index_map)
{
    uint8_t* body_index_map_buffer = k4a_image_get_buffer(body_index_map);

    // Given body_index_map pixel type should be uint8, the stride_byte should be the same as width
    // TODO: Since there is no API to query the byte-per-pixel information, we have to compare the width and stride to
    // know the information. We should replace this assert with proper byte-per-pixel query once the API is provided by
    // K4A SDK.
    assert(k4a_image_get_stride_bytes(body_index_map) == k4a_image_get_width_pixels(body_index_map));

    int middle_line_num = k4a_image_get_height_pixels(body_index_map) / 2;
    body_index_map_buffer = body_index_map_buffer + middle_line_num * k4a_image_get_width_pixels(body_index_map);

    printf("BodyIndexMap at Line %d:\n", middle_line_num);
    for (int i = 0; i < k4a_image_get_width_pixels(body_index_map); i++)
    {
        printf("%u, ", *body_index_map_buffer);
        body_index_map_buffer++;
    }
    printf("\n");
}

// Transform skeleton results from 3d depth space to 2d color image space
inline bool transform_joint_from_depth_3d_to_color_2d(
    const k4a_calibration_t* calibration, 
    k4a_float3_t joint_in_depth_space, 
    k4a_float2_t& joint_in_color_2d)
{
    int valid;
    VERIFY(k4a_calibration_3d_to_2d(
        calibration, 
        &joint_in_depth_space, 
        K4A_CALIBRATION_TYPE_DEPTH, 
        K4A_CALIBRATION_TYPE_COLOR, 
        &joint_in_color_2d, 
        &valid), "Failed to project 3d joint from depth space to 2d color image space!");

    return valid != 0;
}

// Transform body index map results from depth space to color space
void transform_body_index_map_from_depth_to_color(
    k4a_transformation_t transformation_handle,
    const k4a_image_t depth_image,
    const k4a_image_t body_index_map_in_depth_space, 
    k4a_image_t depth_image_in_color_space,
    k4a_image_t body_index_map_in_color_space)
{
    // Note:
    // 1. Depth image - In order to transform the body index map to color space, the corresponding depth image is 
    //    required to help perform this transformation in 3d.
    // 2. Interpolation type - Each pixel value for the body index map represents the body index. It is not interpolatable.
    //    The interpolation method has to be set to K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST.
    // 3. Invalid custom value - Because there is disparity between the depth camera and color camera. There might be 
    //    invalid values during the transform. We want this invalid value to be set to K4ABT_BODY_INDEX_MAP_BACKGROUND.

    VERIFY(k4a_transformation_depth_image_to_color_camera_custom(
        transformation_handle,
        depth_image,
        body_index_map_in_depth_space,
        depth_image_in_color_space,
        body_index_map_in_color_space,
        K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
        K4ABT_BODY_INDEX_MAP_BACKGROUND), "Failed to transform body index map to color space!");
}

// void VisualizeResult(k4abt_frame_t body_frame, k4a_calibration_t sensorCalibration, Window3dWrapper& window3d, int depthWidth, int depthHeight, ros::Publisher pub) {

//     // Obtain original capture that generates the body tracking result
//     k4a_capture_t originalCapture = k4abt_frame_get_capture(body_frame);
//     k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
//     std::vector<Color> pointCloudColors(depthWidth * depthHeight, { 1.f, 1.f, 1.f, 1.f });

// // // added
// //     k4a_transformation_t transformation = k4a_transformation_create(&sensorCalibration);
// //     k4a_image_t transformed_depth_image;
// //     VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,1280,720,0,&transformed_depth_image),"image create failure");
// //     VERIFY(k4a_transformation_depth_image_to_color_camera(transformation, depthImage, transformed_depth_image), "Transformation failed!");


//     // Read body index map and assign colors
//     k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(body_frame);
//     const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);

    
//     for (int i = 0; i < depthWidth * depthHeight; i++)
//     {
//         uint8_t bodyIndex = bodyIndexMapBuffer[i];
//         if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
//         {
//             uint32_t bodyId = k4abt_frame_get_body_id(body_frame, bodyIndex);
//             pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
//         }
//     }
//     k4a_image_release(bodyIndexMap);

//     // Visualize point cloud
//     window3d.UpdatePointClouds(depthImage, pointCloudColors);

//     // Visualize the skeleton data
//     window3d.CleanJointsAndBones();
//     uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame); ROS_INFO( "Detected num_bodies: %d", num_bodies);
//     if(num_bodies == 0){
//         std_msgs::Float32 msg;
//         msg.data = 0;
//         pub.publish(msg);
//     }

//     for (uint32_t i = 0; i < num_bodies; i++)
//     {
//         k4abt_body_t body;
//         VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get skeleton from body frame failed!");
//         body.id = k4abt_frame_get_body_id(body_frame, i);

//         // Assign the correct color based on the body id
//         Color color = g_bodyColors[body.id % g_bodyColors.size()];
//         color.a = 0.4f;
//         Color lowConfidenceColor = color;
//         lowConfidenceColor.a = 0.1f;

//         Color color2; // pelvis
//         color2.r = 1.0f; color2.g = 0.0f; color2.b = 0.0f; color2.a = 1.0f;

//         Color color3; // head
//         color3.r = 0.0f; color3.g = 1.0f; color3.b = 0.0f; color3.a = 1.0f;

//         Color color4; // foot
//         color4.r = 0.0f; color4.g = 0.0f; color4.b = 1.0f; color4.a = 1.0f;

//         ROS_INFO( "Body ID: %d", body.id);
//         // Visualize joints
//         for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
//         {
//             if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
//             {
//                 const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
//                 const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

//                 // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint%d Position: x: %f, y: %f, z: %f", joint, jointPosition.v[0], jointPosition.v[1], jointPosition.v[2]);
//                 // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint%d Orient: x: %f, y: %f, z: %f, w: %f", joint, jointOrientation.v[0], jointOrientation.v[1], jointOrientation.v[2], jointOrientation.v[3]);
//                 window3d.AddJoint(
//                     jointPosition,
//                     jointOrientation,
//                     body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor);
//             }
            
//         }
//         // for debug 
//         window3d.AddJoint(
//                     body.skeleton.joints[K4ABT_JOINT_PELVIS].position,
//                     body.skeleton.joints[K4ABT_JOINT_PELVIS].orientation,
//                     body.skeleton.joints[K4ABT_JOINT_PELVIS].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color2 : lowConfidenceColor);
//         window3d.AddJoint(
//                     body.skeleton.joints[K4ABT_JOINT_HEAD].position,
//                     body.skeleton.joints[K4ABT_JOINT_HEAD].orientation,
//                     body.skeleton.joints[K4ABT_JOINT_HEAD].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color3 : lowConfidenceColor);
//         window3d.AddJoint(
//                     body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position,
//                     body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].orientation,
//                     body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color4 : lowConfidenceColor);
//         window3d.AddJoint(
//                     body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position,
//                     body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].orientation,
//                     body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color4 : lowConfidenceColor);
//         if(debug){
//             ROS_INFO("Joint PELVIS Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[0], body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[1], body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[2]);
//             ROS_INFO("Joint HEAD Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[0], body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[1], body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[2]);
//             ROS_INFO("Joint LEFTFOOT Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position.v[0], body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position.v[1], body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position.v[2]);
//             ROS_INFO("Joint RIGHTFOOT Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position.v[0], body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position.v[1], body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position.v[2]);
//         }
//         float head_x = body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[0]/1000;
//         float head_y = body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[1]/1000;
//         float head_z = body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[2]/1000;
//         Eigen::Vector4f res,input;
//         input << head_x/head_z, head_y/head_z, 1.0f, 1.0f/head_z;
        
//         // world = color_extrinv * color_intrinv * depth_to_color * depth
//         res =head_z* Extr.inverse() * Intr.inverse() * input;
//         if(debug){
//             std::cout<< "transform result : " << res <<std::endl;
//         }
//         float length = calculateBodyLength(body.skeleton.joints[K4ABT_JOINT_HEAD].position, body.skeleton.joints[K4ABT_JOINT_PELVIS].position, body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position, body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position);
//         std_msgs::Float32 msg;
//         msg.data = length;
//         // std_msgs::String msg;
//         // msg.data = std::to_string(length);
//         pub.publish(msg);
        
//         // Visualize bones
//         for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
//         {
//             k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
//             k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

//             if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
//                 body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
//             {
//                 bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
//                     body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
//                 const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
//                 const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

//                 window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor);
//             }
//         }
//     }

//     k4a_capture_release(originalCapture);
//     k4a_image_release(depthImage);

// }

int main(int argc, char** argv)
{
    Extr << -0.985f, -0.162f, -0.064f, 0.117f,
            -0.029f, 0.512f, -0.859f, -0.407f,
            0.171f, -0.844f, -0.509f, 2.033f,
            0.0f, 0.0f, 0.0f, 1.0f;
    Intr << 252.372f, 0.0f, 260.575f, 0.0f,
            0.0f, 252.433f, 252.237f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;
    
    ros::init(argc,argv, "az_body_tracker");
    ros::NodeHandle nh;
    // TrackerNode node(nh);
    auto node = std::make_shared<TrackerNode>(nh);

    ros::Rate rate(30);
    ROS_INFO("node started!");
    InputSettings inputSettings;
    if(!ParseInputSettingsFromArg(argc, argv, inputSettings)) {printf("Program Stopped!\n"); exit(0);}
    node->showInfo(inputSettings);

    // Start camera. Make sure depth camera is enabled.
    k4a_device_t device = nullptr;
    
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = inputSettings.DepthCameraMode;
    deviceConfig.camera_fps = inputSettings.cameraFPS;
    deviceConfig.color_resolution = inputSettings.colorResolution;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!"); ROS_INFO("Open K4A Device succeeded!");

    

    // deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!"); ROS_INFO("Start K4A cameras succeeded!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    // k4a_calibration_extrinsics_t extr;
    // for(int i {0};i<9;i++){
    //     extr.rotation[i]=ExtrR[i];
    // }
    // for(int i {0};i<3;i++){
    //     extr.translation[i]=ExtrT[i];
    // }
    
    // sensorCalibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_DEPTH]=extr;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    // Create transformation handle to perform the body index map space transform
    k4a_transformation_t transformation = NULL;
    transformation = k4a_transformation_create(&sensorCalibration);
    if (transformation == NULL)
    {
        printf("Failed to create transformation from sensor calibration!");
        exit(1);
    }

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = inputSettings.processingMode;
    // trackerConfig.model_path = inputSettings.ModelPath.c_str();
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!"); ROS_INFO("Body tracker initialization succeeded!");

    // Preallocated the buffers to hold the depth image in color space and the body index map in color space
    int color_image_width_pixels = sensorCalibration.color_camera_calibration.resolution_width;
    int color_image_height_pixels = sensorCalibration.color_camera_calibration.resolution_height;
    k4a_image_t depth_image_in_color_space = NULL;
    VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t),
        &depth_image_in_color_space), "Failed to create empty image for the depth image in color space");

    k4a_image_t body_index_map_in_color_space = NULL;
    VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM8,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint8_t),
        &body_index_map_in_color_space), "Failed to create empty image for the body index map in color space");


    // Initialize the 3d window controller
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);
    ROS_INFO("3D visualization window initialization succeeded!");
    
    ROS_INFO("Start tracking!");
    while (ros::ok())
    {
        
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
        k4abt_frame_t body_frame = NULL;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &body_frame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            /************* Successfully get a body tracking result, process the result here ***************/
            // VisualizeResult(body_frame,sensorCalibration, window3d, depthWidth, depthHeight, node->length_pub);

            // Obtain original capture that generates the body tracking result
            // k4a_capture_t originalCapture = k4abt_frame_get_capture(body_frame);
            std::vector<Color> pointCloudColors(depthWidth * depthHeight, { 1.f, 1.f, 1.f, 1.f });

            // // added
            //     k4a_transformation_t transformation = k4a_transformation_create(&sensorCalibration);
            //     k4a_image_t transformed_depth_image;
            //     VERIFY(k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,1280,720,0,&transformed_depth_image),"image create failure");
            //     VERIFY(k4a_transformation_depth_image_to_color_camera(transformation, depthImage, transformed_depth_image), "Transformation failed!");

            uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame); ROS_INFO( "Detected num_bodies: %d", num_bodies);
            if(num_bodies == 0){
                std_msgs::Float32 msg;
                msg.data = 0;
                node->length_pub.publish(msg);
            }

             // Depth image is needed in order to perform the body index map space transform
            k4a_image_t depth_image = k4a_capture_get_depth_image(sensorCapture);
            // Read body index map and assign colors
            k4a_image_t body_index_map_in_depth_space = k4abt_frame_get_body_index_map(body_frame);
            if (body_index_map_in_depth_space != NULL)
            {
                

                transform_body_index_map_from_depth_to_color(
                    transformation,
                    depth_image,
                    body_index_map_in_depth_space,
                    depth_image_in_color_space,
                    body_index_map_in_color_space);

                // print_body_index_map_middle_line(body_index_map_in_color_space);
                
                
                
            }
            else
            {
                printf("Error: Fail to generate bodyindex map!\n");
            }
            const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(body_index_map_in_depth_space);

            
            for (int i = 0; i < depthWidth * depthHeight; i++)
            {
                uint8_t bodyIndex = bodyIndexMapBuffer[i];
                if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
                {
                    uint32_t bodyId = k4abt_frame_get_body_id(body_frame, bodyIndex);
                    pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
                }
            }
            // Visualize point cloud
                window3d.UpdatePointClouds(depth_image, pointCloudColors);

            

            // Visualize the skeleton data
            window3d.CleanJointsAndBones();

            // Transform each 3d joints from 3d depth space to 2d color image space
            for (uint32_t i = 0; i < num_bodies; i++)
            {
                printf("Person[%u]:\n", i);
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get body from body frame failed!");
                for (int joint_id = 0; joint_id < (int)K4ABT_JOINT_COUNT; joint_id++)
                {
                    k4a_float2_t joint_in_color_2d;
                    bool valid = transform_joint_from_depth_3d_to_color_2d(
                        &sensorCalibration, 
                        body.skeleton.joints[joint_id].position,
                        joint_in_color_2d);
                    if (valid)
                    {
                        printf("Joint[%d]: Pixel Location at Color Image ( %f, %f) \n",
                            joint_id, joint_in_color_2d.v[0], joint_in_color_2d.v[1]);
                    }
                    else
                    {
                        printf("Joint[%d]: Invalid Pixel Location \n", joint_id);
                    }
                }

                body.id = k4abt_frame_get_body_id(body_frame, i);

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

                ROS_INFO( "Body ID: %d", body.id);
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
                if(debug){
                    ROS_INFO("Joint PELVIS Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[0], body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[1], body.skeleton.joints[K4ABT_JOINT_PELVIS].position.v[2]);
                    ROS_INFO("Joint HEAD Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[0], body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[1], body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[2]);
                    ROS_INFO("Joint LEFTFOOT Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position.v[0], body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position.v[1], body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position.v[2]);
                    ROS_INFO("Joint RIGHTFOOT Position: x: %f, y: %f, z: %f", body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position.v[0], body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position.v[1], body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position.v[2]);
                }
                // float head_x = body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[0]/1000;
                // float head_y = body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[1]/1000;
                // float head_z = body.skeleton.joints[K4ABT_JOINT_HEAD].position.v[2]/1000;
                // Eigen::Vector4f res,input;
                // input << head_x/head_z, head_y/head_z, 1.0f, 1.0f/head_z;
                
                // // world = color_extrinv * color_intrinv * depth_to_color * depth
                // res =head_z* Extr.inverse() * Intr.inverse() * input;
                // if(debug){
                //     std::cout<< "transform result : " << res <<std::endl;
                // }
                float length = calculateBodyLength(body.skeleton.joints[K4ABT_JOINT_HEAD].position, body.skeleton.joints[K4ABT_JOINT_PELVIS].position, body.skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position, body.skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position);
                std_msgs::Float32 msg;
                msg.data = length;
                // std_msgs::String msg;
                // msg.data = std::to_string(length);
                node->length_pub.publish(msg);
                
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


            

            for (uint32_t i = 0; i < num_bodies; i++)
            {
                k4abt_body_t body;
                
                VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get skeleton from body frame failed!");
                
            }
            k4a_image_release(body_index_map_in_depth_space);
            // k4a_capture_release(originalCapture);
            //Release the body_frame
            k4abt_frame_release(body_frame);
            k4a_image_release(depth_image_in_color_space);
            k4a_image_release(body_index_map_in_depth_space);
            k4a_image_release(depth_image);
        }
        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
        frame_count++;
        rate.sleep();
    }

    std::cout << "Finished body tracking processing!" << std::endl;

    window3d.Delete();
    k4a_transformation_destroy(transformation);
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);


    
    printf("Program Stopped!\n");

    return 0;
}