#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <phoxi_camera/TutorialsConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <phoxi_camera/PhoXiSize.h>
#include <std_srvs/Empty.h>
#include <phoxi_camera/GetDeviceList.h>
#include <phoxi_camera/ConnectCamera.h>
#include <phoxi_camera/IsAcquiring.h>
#include <phoxi_camera/TriggerImage.h>
#include <phoxi_camera/GetFrame.h>
#include <phoxi_camera/GetHardwareIdentification.h>
#include <phoxi_camera/GetSupportedCapturingModes.h>

//#define PHOXI_PCL_SUPPORT

#include "PhoXi.h"
#include <string>
#include <vector>
#include <iostream>

#if defined(_WIN32)
#include <windows.h>
#elif defined (__linux__)
#include <unistd.h>
#endif


#if defined(_WIN32)
#define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
#elif defined (__linux__)
#define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl/pcl_conversions.h>

#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <opencv2/opencv.hpp>
//#include "Console/PhotoneoConsole.h"

pho::api::PPhoXi EvaluationScanner;
pho::api::PhoXiFactory Factory;
ros::Publisher pub;

void init_config(pho::api::PPhoXi &Scanner) {
    std::cout << "cinit" << std::endl;
    ros::param::set("~size_height", Scanner->Resolution->Height);
    ros::param::set("~size_width", Scanner->Resolution->Width);
    ros::param::set("~capturing_size_height", Scanner->CapturingMode->Resolution.Height);
    ros::param::set("~capturing_size_width", Scanner->CapturingMode->Resolution.Width);
    ros::param::set("~capturing_scan_multiplier", Scanner->CapturingSettings->ScanMultiplier);
    ros::param::set("~acquisition_time", Scanner->AcquisitionTime);
//    ros::param::set("~trigger_mode", Scanner->TriggerMode);
    //ros::param::set("~timeout", (int)Scanner->Timeout * -1);
    //ros::param::set("~processing_settings", Scanner->ProcessingSettings->RequiredConfidence);
//    ros::param::set("~send_point_cloud", Scanner->OutputSettings->SendPointCloud);
//    ros::param::set("~send_depth_map", Scanner->OutputSettings->SendDepthMap);
//    ros::param::set("~send_confidence_map", Scanner->OutputSettings->SendConfidenceMap);
//    ros::param::set("~send_texture", Scanner->OutputSettings->SendTexture);
}

void callback(pho::api::PPhoXi &Scanner, phoxi_camera::TutorialsConfig &config, uint32_t level) {
//void callback(phoxi_camera::TutorialsConfig &config, uint32_t level) {
    std::cout << EvaluationScanner << std::endl;
    if (EvaluationScanner == 0){
        return;
    }
    printf("%s%d%s", "\033[", 31, "m");
    std::cout << "level " << level << std::endl;
    std::bitset<32> x(level);
    std::cout << "bitmask " << x << std::endl;
    if (level & (1 << 1)) {
        Scanner->Resolution->Height = config.size_height;
    }
    if (level & (1 << 2)) {
        Scanner->Resolution->Width = config.size_width;
    }
    if (level & (1 << 3)) {
        Scanner->CapturingMode->Resolution.Height = config.capturing_size_width;
    }
    if (level & (1 << 4)) {
        Scanner->CapturingMode->Resolution.Width = config.capturing_size_width;
    }
    if (level & (1 << 5)) {
        Scanner->CapturingSettings->ScanMultiplier = config.capturing_scan_multiplier;
    }
    if (level & (1 << 6)) {
        Scanner->AcquisitionTime = config.acquisition_time;
    }
    std::cout << "koniec callback " << level << std::endl;
    printf("%s%d%s", "\033[", 0, "m");
//    if (level & (1 << 9)) {
//        Scanner->ProcessingSettings->RequiredConfidence = config.processing_settings;
//    }
//    if (level & (1 << 8)) {
//        Scanner->Resolution->width = config.size_width;
//    }
//    if (level & (1 << 9)) {
//        Scanner->Resolution->width = config.size_width;
//    }
//    if (level & (1 << 10)) {
//        Scanner->Resolution->width = config.size_width;
//    }
//    if (level & (1 << 11)) {
//        Scanner->Resolution->width = config.size_width;
//    }
//    if (level & (1 << 12)) {
//        Scanner->Resolution->width = config.size_width;
//    }
//    if (level & (1 << 13)) {
//        Scanner->Resolution->width = config.size_width;
//    }

}

std::vector<pho::api::PhoXiDeviceInformation> get_device_list(){
//    pho::api::PhoXiFactory Factory;
    Factory.StartConsoleOutput("Admin-On");
    return Factory.GetDeviceList();
}

bool get_device_list_service(phoxi_camera::GetDeviceList::Request &req,
         phoxi_camera::GetDeviceList::Response &res) {
    std::vector <pho::api::PhoXiDeviceInformation> DeviceList = get_device_list();
    res.len = DeviceList.size();
    std::cout << "dlzka " << DeviceList.size() << std::endl;
    for (int i = 0; i < DeviceList.size(); ++i) {
        res.out.push_back(DeviceList[i].HWIdentification);
    }
    return true;
}

bool connect_camera(phoxi_camera::ConnectCamera::Request &req,
                             phoxi_camera::ConnectCamera::Response &res) {
    std::vector <pho::api::PhoXiDeviceInformation> DeviceList = get_device_list();
    std::cout << "device list" << std::endl;
    for (int i = 0; i < DeviceList.size(); i++) {
        if(DeviceList[i].HWIdentification == req.name){
            std::cout << "nasiel" << std::endl;
            EvaluationScanner = Factory.Create(DeviceList[i]);
            EvaluationScanner->Connect();
            std::cout << "pripojil" << std::endl;
            if (EvaluationScanner->isConnected()) {
                //init_config(EvaluationScanner);
                std::cout << "success" << std::endl;
                res.success = true;
                return true;
            }
        }
    }
    init_config(EvaluationScanner);
    res.success = false;
    return true;
}

bool disconnect_camera(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    EvaluationScanner->Disconnect();
    return true;
}



bool is_acquiring(phoxi_camera::IsAcquiring::Request &req, phoxi_camera::IsAcquiring::Response &res) {
    if (EvaluationScanner->isAcquiring()) {
        res.is = true;
        return true;
    }
    res.is = false;
    return true;
}

bool start_acquisition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    EvaluationScanner->StartAcquisition();
    return true;
}

bool stop_acquisition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    EvaluationScanner->StopAcquisition();
    return true;
}

bool trigger_image(phoxi_camera::TriggerImage::Request &req, phoxi_camera::TriggerImage::Response &res){
    if (EvaluationScanner->TriggerImage()) {
        res.success = true;
    }
    else res.success = false;
    return true;
}

void publish_frame(pho::api::PFrame MyFrame){
    if (MyFrame) {
        if (!MyFrame->PointCloud.Empty())
            std::cout << "PointCloud: " << MyFrame->PointCloud.Size.Width << " x " <<
            MyFrame->PointCloud.Size.Height << " Type: " <<
            MyFrame->PointCloud.GetElementName() << std::endl;
        if (!MyFrame->DepthMap.Empty())
            std::cout << "DepthMap: " << MyFrame->DepthMap.Size.Width << " x " <<
            MyFrame->DepthMap.Size.Height << " Type: " << MyFrame->DepthMap.GetElementName() <<
            std::endl;
        if (!MyFrame->Texture.Empty())
            std::cout << "Texture: " << MyFrame->Texture.Size.Width << " x " <<
            MyFrame->Texture.Size.Height << " Type: " << MyFrame->Texture.GetElementName() <<
            std::endl;
        if (!MyFrame->ConfidenceMap.Empty())
            std::cout << "ConfidenceMap: " << MyFrame->ConfidenceMap.Size.Width << " x " <<
            MyFrame->ConfidenceMap.Size.Height << " Type: " <<
            MyFrame->ConfidenceMap.GetElementName() << std::endl;
        //MyFrame->SaveAsPly("Test Software" + std::to_string(k) + " , " + std::to_string(i) + ".ply");
        //pcl::PointCloud<pcl::PointXYZRGB> MyPCLCloud;
        //MyFrame->ConvertTo(MyPCLCloud);
        //pcl::PCLPointCloud2 MyPCLCloud2;
        //MyFrame->ConvertTo(MyPCLCloud2);
        //pcl::PLYWriter Writer;
        //Writer.writeBinary("Test Software PCL" + std::to_string(k) + " , " + std::to_string(i) + ".ply", MyPCLCloud2);
        pcl::PointCloud <pcl::PointXYZ> cloud;
        int h = MyFrame->PointCloud.Size.Height;
        int w = MyFrame->PointCloud.Size.Width;
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                auto &point = MyFrame->PointCloud.At(i, j);
                if (point.z > 10 && point.z < 10000)
                    cloud.push_back(pcl::PointXYZ(point.x, point.y, point.z));
                // cloud.push_back (pcl::PointXYZ (i, j, i+j));
            }
        }
        ROS_INFO("publishing");
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "map";
        pub.publish(output);
    }
}

bool get_frame(phoxi_camera::GetFrame::Request &req, phoxi_camera::GetFrame::Response &res){
    pho::api::PFrame MyFrame = EvaluationScanner->GetFrame(req.in);
    if(MyFrame){
        publish_frame(MyFrame);
        res.success = true;
    }
    else res.success = false;
    return true;
}

bool get_supported_capturing_modes(phoxi_camera::GetSupportedCapturingModes::Request &req, phoxi_camera::GetSupportedCapturingModes::Response &res){
    std::vector<pho::api::PhoXiCapturingMode> vec = EvaluationScanner->SupportedCapturingModes;
    for(int i = 0;i< vec.size();i++){
        phoxi_camera::PhoXiSize a;
        a.Width = vec[i].Resolution.Width;
        a.Height = vec[i].Resolution.Height;
        res.supported_capturing_modes.push_back(a);
    }
    return true;
}

bool get_hardware_identification(phoxi_camera::GetHardwareIdentification::Request &req, phoxi_camera::GetHardwareIdentification::Response &res){
    res.hardware_identification = EvaluationScanner->HardwareIdentification;
    return true;
}

int main(int argc, char **argv) {
    ROS_INFO("Starting pho_driver ros...");
    ros::init(argc, argv, "phoxi_camera");
    ros::NodeHandle nh("~");

    dynamic_reconfigure::Server <phoxi_camera::TutorialsConfig> server;
    dynamic_reconfigure::Server<phoxi_camera::TutorialsConfig>::CallbackType f;
    f = boost::bind(&callback, boost::ref(EvaluationScanner), _1, _2);
    server.setCallback(f);
    ros::ServiceServer service_get_device_list = nh.advertiseService("get_device_list", get_device_list_service);
    ros::ServiceServer service_connect_camera = nh.advertiseService("connect_camera", connect_camera);
    ros::ServiceServer service_is_acquiring = nh.advertiseService("is_acquiring", is_acquiring);
    ros::ServiceServer service_start_acquisition = nh.advertiseService("start_acquisition", start_acquisition);
    ros::ServiceServer service_stop_acquisition = nh.advertiseService("stop_acquisition", stop_acquisition);
    ros::ServiceServer service_trigger_image = nh.advertiseService("trigger_image", trigger_image);
    ros::ServiceServer service_get_frame = nh.advertiseService("get_frame", get_frame);
    ros::ServiceServer service_disconnect_camera = nh.advertiseService("disconnect_camera", disconnect_camera);
    ros::ServiceServer service_get_hardware_identification = nh.advertiseService("get_hardware_indentification", get_hardware_identification);
    ros::ServiceServer service_get_supported_capturing_modes = nh.advertiseService("get_supported_capturing_modes", get_supported_capturing_modes);
    ROS_INFO("Ready");
//    f = boost::bind(&callback, _1, _2);
//    server.setCallback(f);
    //LOCAL_CROSS_SLEEP(5000);
    pub = nh.advertise < pcl::PointCloud < pcl::PointXYZ >> ("output", 1);
//    int k = 0;
    ros::spin();
    while (ros::ok()) {
        ROS_INFO("Spin loop");
        LOCAL_CROSS_SLEEP(1000);
        pho::api::PhoXiFactory Factory;
        Factory.StartConsoleOutput("Admin-On");
        std::vector <pho::api::PhoXiDeviceInformation> DeviceList = Factory.GetDeviceList();
        for (int k = 0; k < DeviceList.size(); k++){
            if (DeviceList[k].HWIdentification[18] != '0' && DeviceList[k].HWIdentification[19] != '1') continue;
            ROS_INFO("found at least one");
            pho::api::PPhoXi EvaluationScanner = Factory.Create(DeviceList[k]);
            EvaluationScanner->Connect();
            if (EvaluationScanner->isAcquiring()) {
                EvaluationScanner->StopAcquisition();
            }

            if (EvaluationScanner->isConnected()) {
                ROS_INFO("Som tam");
                init_config(EvaluationScanner);
//                f = boost::bind(&callback, boost::ref(EvaluationScanner), _1, _2);
//                server.setCallback(f);
                LOCAL_CROSS_SLEEP(15000);
                EvaluationScanner->TriggerMode = pho::api::PhoXiTriggerMode::Software;
                while (EvaluationScanner->GetFrame(0)) {
                    std::cout << "Cleaning Buffer" << std::endl;
                }
                EvaluationScanner->StartAcquisition();
                if (EvaluationScanner->isAcquiring()) {
                    for (int i = 0; i < 1; i++) {
                        if (EvaluationScanner->TriggerImage()) {
                            pho::api::PFrame MyFrame = EvaluationScanner->GetFrame(pho::api::PhoXiTimeout::Infinity);
                            EvaluationScanner->AcquisitionTime = 1.0;
                            if (MyFrame) {
                                std::cout << i << std::endl;
                                if (!MyFrame->PointCloud.Empty())
                                    std::cout << "PointCloud: " << MyFrame->PointCloud.Size.Width << " x " <<
                                    MyFrame->PointCloud.Size.Height << " Type: " <<
                                    MyFrame->PointCloud.GetElementName() << std::endl;
                                if (!MyFrame->DepthMap.Empty())
                                    std::cout << "DepthMap: " << MyFrame->DepthMap.Size.Width << " x " <<
                                    MyFrame->DepthMap.Size.Height << " Type: " << MyFrame->DepthMap.GetElementName() <<
                                    std::endl;
                                if (!MyFrame->Texture.Empty())
                                    std::cout << "Texture: " << MyFrame->Texture.Size.Width << " x " <<
                                    MyFrame->Texture.Size.Height << " Type: " << MyFrame->Texture.GetElementName() <<
                                    std::endl;
                                if (!MyFrame->ConfidenceMap.Empty())
                                    std::cout << "ConfidenceMap: " << MyFrame->ConfidenceMap.Size.Width << " x " <<
                                    MyFrame->ConfidenceMap.Size.Height << " Type: " <<
                                    MyFrame->ConfidenceMap.GetElementName() << std::endl;
                                //MyFrame->SaveAsPly("Test Software" + std::to_string(k) + " , " + std::to_string(i) + ".ply");
                                //pcl::PointCloud<pcl::PointXYZRGB> MyPCLCloud;
                                //MyFrame->ConvertTo(MyPCLCloud);
                                //pcl::PCLPointCloud2 MyPCLCloud2;
                                //MyFrame->ConvertTo(MyPCLCloud2);
                                //pcl::PLYWriter Writer;
                                //Writer.writeBinary("Test Software PCL" + std::to_string(k) + " , " + std::to_string(i) + ".ply", MyPCLCloud2);
                                pcl::PointCloud <pcl::PointXYZ> cloud;
                                int h = MyFrame->PointCloud.Size.Height;
                                int w = MyFrame->PointCloud.Size.Width;
                                for (int i = 0; i < h; ++i) {
                                    for (int j = 0; j < w; ++j) {
                                        auto &point = MyFrame->PointCloud.At(i, j);
                                        if (point.z > 10 && point.z < 10000)
                                            cloud.push_back(pcl::PointXYZ(point.x, point.y, point.z));
                                        // cloud.push_back (pcl::PointXYZ (i, j, i+j));
                                    }
                                }
                                ROS_INFO("publishing");
                                sensor_msgs::PointCloud2 output;
                                pcl::toROSMsg(cloud, output);
                                output.header.frame_id = "map";
                                pub.publish(output);
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}
