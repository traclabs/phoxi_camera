#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <phoxi_camera/TutorialsConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <phoxi_camera/AddTwoInts.h>

//#define PHOXI_PCL_SUPPORT

#include "PhoXi.h"
#include <string>

#if defined(_WIN32)
#include <windows.h>
#elif defined (__linux__)
#include <unistd.h>
#endif

#include <iostream>

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
int q;
bool add(std_srvs::Empty::Request &req,
          phoxi_camera::AddTwoInts::Response &res){
//   res.sum = req.a + req.b + q;
   q += 1;
    res.sum = q;
//     ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
   return true;
}

int main(int argc, char **argv) {
    ROS_INFO("Starting pho_driver ros...");
    ros::init(argc, argv, "phoxi_camera");
    q = 1;
    ros::NodeHandle nh;
    ros::Publisher pub;

    dynamic_reconfigure::Server <phoxi_camera::TutorialsConfig> server;
    dynamic_reconfigure::Server<phoxi_camera::TutorialsConfig>::CallbackType f;
//    ros::AsyncSpinner spinner(4); // Use 4 threads
//    spinner.start();
    ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::spin();
//    f = boost::bind(&callback, _1, _2);
//    server.setCallback(f);
    //LOCAL_CROSS_SLEEP(5000);
    pub = nh.advertise < pcl::PointCloud < pcl::PointXYZ >> ("output", 1);
    int k = 0;
    while (ros::ok()) {
        ROS_INFO("Spin loop");
//        LOCAL_CROSS_SLEEP(5000);
//        continue;
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
                f = boost::bind(&callback, boost::ref(EvaluationScanner), _1, _2);
                server.setCallback(f);
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
