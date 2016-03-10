#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pho_driver/TutorialsConfig.h>
#include <sensor_msgs/PointCloud2.h>

//#define PHOXI_PCL_SUPPORT

void callback(pho_driver::TutorialsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d %f %s %s %d",
             config.int_param, config.double_param,
             config.str_param.c_str(),
             config.bool_param ? "True" : "False",
             config.size);
}

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

int main(int argc, char **argv) {
    ROS_INFO("Starting pho_driver ros...");
    ros::init(argc, argv, "pho_driver");

    ros::NodeHandle nh;
    ros::Publisher pub;

    dynamic_reconfigure::Server <pho_driver::TutorialsConfig> server;
    dynamic_reconfigure::Server<pho_driver::TutorialsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);


LOCAL_CROSS_SLEEP(5000);
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("output", 1);
    int k = 0;
        while(ros::ok()){
            ROS_INFO("Spin loop");
            LOCAL_CROSS_SLEEP(1000);
            pho::api::PhoXiFactory Factory;
            Factory.StartConsoleOutput("Admin-On");
            std::vector<pho::api::PhoXiDeviceInformation> DeviceList = Factory.GetDeviceList();
            for(int k = 0;k < DeviceList.size();k++){
                if(DeviceList[k].HWIdentification != "file") continue;
                ROS_INFO("found at least one");
                pho::api::PPhoXi EvaluationScanner = Factory.Create(DeviceList[k]);
                EvaluationScanner->Connect();
                if (EvaluationScanner->isAcquiring()) {
                    EvaluationScanner->StopAcquisition();
                }

                if (EvaluationScanner->isConnected()) {
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
                                    if (!MyFrame->PointCloud.Empty()) std::cout << "PointCloud: " << MyFrame->PointCloud.Size.Width << " x " << MyFrame->PointCloud.Size.Height << " Type: " << MyFrame->PointCloud.GetElementName() << std::endl;
                                    if (!MyFrame->DepthMap.Empty()) std::cout << "DepthMap: " << MyFrame->DepthMap.Size.Width << " x " << MyFrame->DepthMap.Size.Height << " Type: " << MyFrame->DepthMap.GetElementName() << std::endl;
                                    if (!MyFrame->Texture.Empty()) std::cout << "Texture: " << MyFrame->Texture.Size.Width << " x " << MyFrame->Texture.Size.Height << " Type: " << MyFrame->Texture.GetElementName() << std::endl;
                                    if (!MyFrame->ConfidenceMap.Empty()) std::cout << "ConfidenceMap: " << MyFrame->ConfidenceMap.Size.Width << " x " << MyFrame->ConfidenceMap.Size.Height << " Type: " << MyFrame->ConfidenceMap.GetElementName() << std::endl;
                                    //MyFrame->SaveAsPly("Test Software" + std::to_string(k) + " , " + std::to_string(i) + ".ply");
                                    //pcl::PointCloud<pcl::PointXYZRGB> MyPCLCloud;
                                    //MyFrame->ConvertTo(MyPCLCloud);
                                    //pcl::PCLPointCloud2 MyPCLCloud2;
                                    //MyFrame->ConvertTo(MyPCLCloud2);
                                    //pcl::PLYWriter Writer;
                                    //Writer.writeBinary("Test Software PCL" + std::to_string(k) + " , " + std::to_string(i) + ".ply", MyPCLCloud2);
                                    pcl::PointCloud<pcl::PointXYZ> cloud;
                                    int h = MyFrame->PointCloud.Size.Height;
                                    int w = MyFrame->PointCloud.Size.Width;
                                    for (int i = 0; i < h; ++i) {
                                        for (int j = 0; j < w; ++j) {
                                            auto& point = MyFrame->PointCloud.At(i, j);
                                            if (point.z > 10 && point.z < 10000)
                                                cloud.push_back (pcl::PointXYZ (point.x, point.y, point.z));
                                                // cloud.push_back (pcl::PointXYZ (i, j, i+j));
                                        }
                                    }
                                    ROS_INFO("publishing");
                                    sensor_msgs::PointCloud2 output;
                                    pcl::toROSMsg(cloud, output);
                                    output.header.frame_id = "map";
                                    pub.publish (output);
                                }
                            }
                        }
                    }
                }
            }
        }
    return 0;
}
