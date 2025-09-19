#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() : Node("mono_node_cpp")
{
    // === INFO ===
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Monocular Node Started");

    // === Parameters (optional) ===
    this->declare_parameter("node_name_arg", "not_given");
    this->declare_parameter("voc_file_arg", "file_not_set");
    this->declare_parameter("settings_file_path_arg", "file_not_set");

    // === Default values ===
    nodeName = this->get_parameter("node_name_arg").as_string();
    vocFilePath = this->get_parameter("voc_file_arg").as_string();
    settingsFilePath = this->get_parameter("settings_file_path_arg").as_string();

    // === Hardcoded path fallback ===
    homeDir = getenv("HOME");
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        vocFilePath = packagePath + "/orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = packagePath + "/orb_slam3/config/Monocular/";
    }

    RCLCPP_INFO(this->get_logger(), "nodeName: %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "vocFilePath: %s", vocFilePath.c_str());

    // === ROS Interfaces ===
    subexperimentconfigName = "/mono_py_driver/experiment_settings";
    pubconfigackName       = "/mono_py_driver/exp_settings_ack";
    subImgMsgName          = "/mono_py_driver/img_msg";
    subTimestepMsgName     = "/mono_py_driver/timestep_msg";

    // Subscriber: config จาก Python
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
        subexperimentconfigName, 10,
        std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    // Publisher: ส่ง ACK กลับ Python
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(
        pubconfigackName, 10);

    RCLCPP_INFO(this->get_logger(), "Waiting for handshake from Python node...");
}

//* Destructor
MonocularMode::~MonocularMode()
{
    if (pAgent)
    {
        pAgent->Shutdown();
    }
    pass;
}

//* Callback: รับ config จาก Python
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg)
{
    bSettingsFromPython = true;
    experimentConfig = msg.data;

    RCLCPP_INFO(this->get_logger(), "Got experiment config: %s", experimentConfig.c_str());

    // === ส่ง ACK กลับ ===
    auto ack_msg = std_msgs::msg::String();
    ack_msg.data = "ACK";
    configAck_publisher_->publish(ack_msg);
    RCLCPP_INFO(this->get_logger(), "Sent ACK to Python node");

    // === Initialize ORB-SLAM3 ===
    initializeVSLAM(experimentConfig);
}

//* Init ORB-SLAM3
void MonocularMode::initializeVSLAM(std::string& configString)
{
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");
        rclcpp::shutdown();
    }

    std::string yaml_path = settingsFilePath + configString + ".yaml";
    RCLCPP_INFO(this->get_logger(), "Using settings file: %s", yaml_path.c_str());

    sensorType = ORB_SLAM3::System::MONOCULAR;
    enablePangolinWindow = true;
    enableOpenCVWindow = false;

    pAgent = new ORB_SLAM3::System(vocFilePath, yaml_path, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode ORB-SLAM3 initialized." << std::endl;

    // ✅ Create subscribers here, after ORB-SLAM3 is ready
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName, 10,
        std::bind(&MonocularMode::Img_callback, this, _1));

    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        subTimestepMsgName, 10,
        std::bind(&MonocularMode::Timestep_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscribers for Image + Timestep initialized after VSLAM startup");
}

//* Callback: รับ timestep
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg)
{
    timeStep = time_msg.data;
}

//* Callback: รับ image
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    if (!pAgent) {
        RCLCPP_WARN(this->get_logger(), "pAgent not initialized yet, skipping frame");
        return;
    }

    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;

        // Run ORB-SLAM3 monocular tracking
        Sophus::SE3f Tcw = pAgent->TrackMonocular(frame, timeStep);
        (void)Tcw; // TODO: publish pose if needed
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
}