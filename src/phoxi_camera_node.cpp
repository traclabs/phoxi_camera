/*********************************************************************************************//**
* @file phoxi_camera_node.cpp
*
* Copyright (c)
* Photoneo s.r.o
* November 2016
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <dynamic_reconfigure/server.h>

#include "PhoXi.h"
#include <phoxi_camera/phoxi_cameraConfig.h>
#include <phoxi_camera/PhoXiSize.h>
#include <phoxi_camera/GetDeviceList.h>
#include <phoxi_camera/ConnectCamera.h>
#include <phoxi_camera/IsConnected.h>
#include <phoxi_camera/IsAcquiring.h>
#include <phoxi_camera/TriggerImage.h>
#include <phoxi_camera/GetFrame.h>
#include <phoxi_camera/SaveFrame.h>
#include <phoxi_camera/GetHardwareIdentification.h>
#include <phoxi_camera/GetSupportedCapturingModes.h>

//#define PHOXI_PCL_SUPPORT

#define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
//#include <opencv2/opencv.hpp>
//#include "Console/PhotoneoConsole.h"

pho::api::PPhoXi EvaluationScanner;
pho::api::PhoXiFactory Factory;
ros::Publisher pub_cloud, pub_normal_map, pub_confidence_map, pub_texture;
pho::api::PFrame CurrentFrame;
sensor_msgs::PointCloud2 output_cloud;
sensor_msgs::Image output_img;
std::string frameId;
bool triggered=false;

void init_config(pho::api::PPhoXi &Scanner) {
    std::cout << "cinit" << std::endl;
//    ros::param::set("~size_height", Scanner->Resolution->Height);
//    ros::param::set("~size_width", Scanner->Resolution->Width);

    int idx = 0;
    std::vector<pho::api::PhoXiCapturingMode> capturingModes = EvaluationScanner->SupportedCapturingModes;
    for (int i = 0; i < capturingModes.size(); ++i) {
        if(capturingModes[i] == Scanner->CapturingMode){
            idx=i;
            break;
        }
    }
    std::cout << "Capture mode: " << idx << std::endl;
    if (capturingModes.size() > idx) {
        EvaluationScanner->CapturingMode = capturingModes[idx];
    }
    ros::param::set("~vertical_resolution", idx+1);
    ros::param::set("~horizontal_resolution", idx+1);
    ros::param::set("~scan_multiplier", Scanner->CapturingSettings->ScanMultiplier);
    ros::param::set("~shutter_multiplier", Scanner->CapturingSettings->ShutterMultiplier);
//    ros::param::set("~acquisition_time", Scanner->AcquisitionTime);
    ros::param::set("~trigger_mode", (pho::api::PhoXiTriggerMode::Value)(pho::api::PhoXiTriggerMode)Scanner->TriggerMode);
    ros::param::set("~timeout", (int)(pho::api::PhoXiTimeout)Scanner->Timeout);
    ros::param::set("~confidence", Scanner->ProcessingSettings->Confidence);
    ros::param::set("~send_point_cloud", Scanner->OutputSettings->SendPointCloud);
    ros::param::set("~send_normal_map", Scanner->OutputSettings->SendNormalMap);
    ros::param::set("~send_confidence_map", Scanner->OutputSettings->SendConfidenceMap);
    ros::param::set("~send_texture", Scanner->OutputSettings->SendTexture);

}

void callback(pho::api::PPhoXi &Scanner, phoxi_camera::phoxi_cameraConfig &config, uint32_t level) {
    if (EvaluationScanner == 0){
        return;
    }
//    if (level & (1 << 1)) {
//        Scanner->Resolution->Height = config.size_height;
//    }
//    if (level & (1 << 2)) {
//        Scanner->Resolution->Width = config.size_width;
//    }
    if (level & (1 << 3)) {
        std::vector<pho::api::PhoXiCapturingMode> capturingModes = EvaluationScanner->SupportedCapturingModes;
        if (capturingModes.size() >= config.vertical_resolution) {
            EvaluationScanner->CapturingMode = capturingModes[config.vertical_resolution - 1];
            config.horizontal_resolution = config.vertical_resolution;
        }
//        Scanner->CapturingMode->Resolution.Height = config.capturing_size_width;
    }
    if (level & (1 << 4)) {
//        Scanner->CapturingMode->Resolution.Height = config.horizontal_resolution;
        std::vector<pho::api::PhoXiCapturingMode> capturingModes = EvaluationScanner->SupportedCapturingModes;
        if (capturingModes.size() >= config.horizontal_resolution) {
            EvaluationScanner->CapturingMode = capturingModes[config.horizontal_resolution - 1];
            config.vertical_resolution = config.horizontal_resolution;
        }
    }
    if (level & (1 << 5)) {
        Scanner->CapturingSettings->ScanMultiplier = config.scan_multiplier;
    }
//    if (level & (1 << 6)) {
//        Scanner->AcquisitionTime = config.acquisition_time;
//    }
    if (level & (1 << 6)) {
        Scanner->CapturingSettings->ShutterMultiplier = config.shutter_multiplier;
    }
    if (level & (1 << 7)) {
        Scanner->TriggerMode = config.trigger_mode;
    }
    if (level & (1 << 8)) {
        Scanner->Timeout = config.timeout;
    }
    if (level & (1 << 9)) {
        Scanner->ProcessingSettings->Confidence = config.confidence;
    }
    if (level & (1 << 10)) {
        Scanner->OutputSettings->SendPointCloud = config.send_point_cloud;
    }
    if (level & (1 << 11)) {
        Scanner->OutputSettings->SendNormalMap = config.send_normal_map;
    }
    if (level & (1 << 12)) {
        Scanner->OutputSettings->SendConfidenceMap = config.send_confidence_map;
    }
    if (level & (1 << 13)) {
        Scanner->OutputSettings->SendTexture = config.send_texture;
    }
}

std::vector<pho::api::PhoXiDeviceInformation> get_device_list(){
    Factory.StartConsoleOutput("Admin-On");
    return Factory.GetDeviceList();
}

bool get_device_list_service(phoxi_camera::GetDeviceList::Request &req,
         phoxi_camera::GetDeviceList::Response &res) {
    std::vector <pho::api::PhoXiDeviceInformation> DeviceList = get_device_list();
    res.len = DeviceList.size();
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
            EvaluationScanner = Factory.Create(DeviceList[i]);
            EvaluationScanner->Connect();
            if (EvaluationScanner->isConnected()) {
                init_config(EvaluationScanner);
                res.success = true;
                return true;
            }
        }
    }
    res.success = false;
    return true;
}

bool is_connected(phoxi_camera::IsConnected::Request &req,
                    phoxi_camera::IsConnected::Response &res) {
  if (EvaluationScanner==0)
    return false;
  res.connected = EvaluationScanner->isConnected();
    return true;
}

bool disconnect_camera(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  if (EvaluationScanner==0)
    return false;
  EvaluationScanner->Disconnect();
    return true;
}

bool is_acquiring(phoxi_camera::IsAcquiring::Request &req, phoxi_camera::IsAcquiring::Response &res) {
  if (EvaluationScanner==0)
    return false;  
  res.is_acquiring = EvaluationScanner->isAcquiring();
    return true;
}

bool start_acquisition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  if (EvaluationScanner==0)
    return false;
  EvaluationScanner->StartAcquisition();
  return true;
}

bool stop_acquisition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  if (EvaluationScanner==0)
    return false;
 
  EvaluationScanner->StopAcquisition();
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
        pcl::PointCloud <pcl::PointXYZI> cloud, cloud2;
        sensor_msgs::Image texture, confidence_map, normal_map;
        ros::Time       timeNow         = ros::Time::now();
        std::string     frame           = "camera";

        texture.header.stamp          = timeNow;
        texture.header.frame_id       = frame;

        confidence_map.header.stamp         = timeNow;
        confidence_map.header.frame_id      = frame;

        normal_map.header.stamp          = timeNow;
        normal_map.header.frame_id       = frame;

        texture.encoding = "32FC1";
        sensor_msgs::fillImage( texture,
                                sensor_msgs::image_encodings::TYPE_32FC1,
                                MyFrame->Texture.Size.Height, // height
                                MyFrame->Texture.Size.Width, // width
                                MyFrame->Texture.Size.Width * sizeof(float), // stepSize
                                MyFrame->Texture.operator[](0));
        confidence_map.encoding = "32FC1";
        sensor_msgs::fillImage( confidence_map,
                                sensor_msgs::image_encodings::TYPE_32FC1,
                                MyFrame->ConfidenceMap.Size.Height, // height
                                MyFrame->ConfidenceMap.Size.Width, // width
                                MyFrame->ConfidenceMap.Size.Width * sizeof(float), // stepSize
                                MyFrame->ConfidenceMap.operator[](0));
        normal_map.encoding = "32FC3";
        sensor_msgs::fillImage( normal_map,
                                sensor_msgs::image_encodings::TYPE_32FC3,
                                MyFrame->NormalMap.Size.Height, // height
                                MyFrame->NormalMap.Size.Width, // width
                                MyFrame->NormalMap.Size.Width * sizeof(float) * 3, // stepSize
                                MyFrame->NormalMap.operator[](0));
        int h = MyFrame->PointCloud.Size.Height;
        int w = MyFrame->PointCloud.Size.Width;
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                auto &point = MyFrame->PointCloud.At(i, j);
                pcl::PointXYZI p;
                p.x = point.x * 0.001d;
                p.y = point.y * 0.001d;
                p.z = point.z * 0.001d;
                p.intensity = MyFrame->Texture.At(i,j);
                cloud.push_back(p);
            }
        }

        std::cout << "publishing data" << std::endl;
        pcl::toROSMsg(cloud, output_cloud);
        output_cloud.header.frame_id = frameId;
        output_cloud.header.stamp = timeNow;
        pub_cloud.publish(output_cloud);
        pub_normal_map.publish(normal_map);
        pub_confidence_map.publish(confidence_map);
        texture.header.stamp = timeNow;
        pub_texture.publish(texture);
        output_img = texture;
    }
}

bool get_frame(phoxi_camera::GetFrame::Request &req, phoxi_camera::GetFrame::Response &res){
  if (EvaluationScanner==0)
    return false;

  if (!triggered) {
    res.success=false;
    return true;
  }

  do {
    CurrentFrame = EvaluationScanner->GetFrame(req.in);
  } while (!CurrentFrame);

  triggered=false;

  publish_frame(CurrentFrame);
  res.success = true;
  res.cloud = output_cloud;
  return true;
}

bool trigger_image(phoxi_camera::TriggerImage::Request &req, phoxi_camera::TriggerImage::Response &res){
  if (EvaluationScanner==0)
    return false;
  
  EvaluationScanner->ClearBuffer();
  res.success = EvaluationScanner->TriggerImage();
  // sometimes the first doesn't trigger
  if (!res.success)
    res.success = EvaluationScanner->TriggerImage();
  triggered=true;
  if (res.success && req.wait_until_published) {
    phoxi_camera::GetFrame srv;
    if (!get_frame(srv.request,srv.response))
      return false;
    res.success = srv.response.success;
  }
  return true;
}

bool save_frame(phoxi_camera::SaveFrame::Request &req, phoxi_camera::SaveFrame::Response &res){
//    pho::api::PFrame MyFrame = EvaluationScanner->GetFrame(req.in);
    std::cout << "Saving frame to " << req.path << std::endl;
    if(CurrentFrame){
        CurrentFrame->SaveAsPly(req.path);
        res.success = true;
    }
    else{
        std::cout << "Failed!" << std::endl;
        res.success = false;
    }
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
    std::cout << "Starting phoxi_camera ros...";
    ros::init(argc, argv, "phoxi_camera");
    ros::NodeHandle nh("~");

    dynamic_reconfigure::Server <phoxi_camera::phoxi_cameraConfig> server;
    dynamic_reconfigure::Server<phoxi_camera::phoxi_cameraConfig>::CallbackType f;

    f = boost::bind(&callback, boost::ref(EvaluationScanner), _1, _2);
    server.setCallback(f);

    nh.param<std::string>("frame_id", frameId, "map");
    ros::ServiceServer service_get_device_list = nh.advertiseService("get_device_list", get_device_list_service);
    ros::ServiceServer service_connect_camera = nh.advertiseService("connect_camera", connect_camera);
    ros::ServiceServer service_is_connected = nh.advertiseService("is_connected", is_connected);
    ros::ServiceServer service_is_acquiring = nh.advertiseService("is_acquiring", is_acquiring);
    ros::ServiceServer service_start_acquisition = nh.advertiseService("start_acquisition", start_acquisition);
    ros::ServiceServer service_stop_acquisition = nh.advertiseService("stop_acquisition", stop_acquisition);
    ros::ServiceServer service_trigger_image = nh.advertiseService("trigger_image", trigger_image);
    ros::ServiceServer service_get_frame = nh.advertiseService("get_frame", get_frame);
    ros::ServiceServer service_save_frame = nh.advertiseService("save_frame", save_frame);
    ros::ServiceServer service_disconnect_camera = nh.advertiseService("disconnect_camera", disconnect_camera);
    ros::ServiceServer service_get_hardware_identification = nh.advertiseService("get_hardware_indentification", get_hardware_identification);
    ros::ServiceServer service_get_supported_capturing_modes = nh.advertiseService("get_supported_capturing_modes", get_supported_capturing_modes);

    pub_cloud = nh.advertise < pcl::PointCloud < pcl::PointXYZ >> ("pointcloud", 1);
    pub_normal_map = nh.advertise < sensor_msgs::Image > ("normal_map", 1);
    pub_confidence_map = nh.advertise < sensor_msgs::Image > ("confidence_map", 1);
    pub_texture = nh.advertise < sensor_msgs::Image > ("texture", 1);

    std::cout << "Ready!" << std::endl;
    ros::spin();
    return 0;
}
