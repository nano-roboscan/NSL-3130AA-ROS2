#include <cstdio>
#include "cartesian_transform.hpp"
#include "interface.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <pcl/conversions.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include "roboscan_publish_node.hpp"


using namespace nanosys;
using namespace std::chrono_literals;
using namespace cv;




roboscanPublisher::roboscanPublisher() : Node("roboscan_publish_node")
{ 
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    imgDistancePub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
    imgAmplPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanAmpl", qos_profile); 
    imgGrayPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanGray", qos_profile); 
    imgDCSPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDCS", qos_profile); 
    pointcloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 

    printf("Init...\n");
    roboscanPublisher::initialise();
    roboscanPublisher::setParameters();
    roboscanPublisher::startStreaming();
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&roboscanPublisher::parametersCallback, this, std::placeholders::_1));


    printf("\nRun rqt to view the image!\n");
    
} 

roboscanPublisher::~roboscanPublisher()
{ 
}

  rcl_interfaces::msg::SetParametersResult roboscanPublisher::parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    for (const auto &param: parameters)
    {
      if (param.get_name() == "B. lensType")
      {
        lidarParam.lensType = param.as_int();
      }
      else if (param.get_name() == "C. imageType")
      {
        lidarParam.imageType = param.as_int();
      }
      else if (param.get_name() == "D. hdr_mode")
      {
        lidarParam.hdr_mode = param.as_int();
      }
      else if (param.get_name() == "E. int0")
      {
        lidarParam.int0 = param.as_int();
      }
      else if (param.get_name() == "F. int1")
      {
        lidarParam.int1 = param.as_int();
      }
      else if (param.get_name() == "G. int2")
      {
        lidarParam.int2 = param.as_int();
      }
      else if (param.get_name() == "H. intGr")
      {
        lidarParam.intGr = param.as_int();
      }
      else if (param.get_name() == "I. minAmplitude")
      {
        lidarParam.minAmplitude = param.as_int();
      }
      else if (param.get_name() == "J. modIndex")
      {
        lidarParam.modIndex = param.as_int();
      }
      else if (param.get_name() == "K. channel")
      {
        lidarParam.channel = param.as_int();
      }
      else if (param.get_name() == "L. roi_leftX")
      {
        lidarParam.roi_leftX = param.as_int();
      }
      else if (param.get_name() == "M. roi_topY")
      {
        lidarParam.roi_topY = param.as_int();
      }
      else if (param.get_name() == "N. roi_rightX")
      {
        lidarParam.roi_rightX = param.as_int();
      }
      else if (param.get_name() == "O. roi_bottomY")
      {
        lidarParam.roi_bottomY = param.as_int();
      }
      else if (param.get_name() == "P. transformAngle")
      {
        lidarParam.transformAngle = param.as_double();
      }
      else if (param.get_name() == "A. cvShow")
      {
        lidarParam.cvShow = param.as_bool();
      }
    }
    setReconfigure();
    return result;
  }

  void roboscanPublisher::setReconfigure()
  {
    //int0 = integrationTime0;

    printf("setReconfigure\n");

    interface.stopStream();    
    interface.setMinAmplitude(lidarParam.minAmplitude);
    interface.setIntegrationTime(lidarParam.int0, lidarParam.int1, lidarParam.int2, lidarParam.intGr, lidarParam.grayscaleIlluminationMode);
        
    interface.setHDRMode((uint8_t)lidarParam.hdr_mode);
    interface.setFilter(lidarParam.medianFilter, lidarParam.averageFilter, static_cast<uint16_t>(lidarParam.temporalFilterFactor * 1000), lidarParam.temporalFilterThreshold, lidarParam.edgeThreshold,
                        lidarParam.temporalEdgeThresholdLow, lidarParam.temporalEdgeThresholdHigh, lidarParam.interferenceDetectionLimit, lidarParam.useLastValue);

    interface.setAdcOverflowSaturation(lidarParam.bAdcOverflow, lidarParam.bSaturation);
    interface.setGrayscaleIlluminationMode(lidarParam.grayscaleIlluminationMode);
   
    if(lidarParam.frequencyModulation == 0) lidarParam.modIndex = 1;
    else if(lidarParam.frequencyModulation == 1)  lidarParam.modIndex = 0;
    else    lidarParam.modIndex = lidarParam.frequencyModulation;

    interface.setModulation(lidarParam.modIndex, lidarParam.channel);
    interface.setRoi(lidarParam.roi_leftX, lidarParam.roi_topY, lidarParam.roi_rightX, lidarParam.roi_bottomY);

    if(lidarParam.startStream){

      if(lidarParam.imageType == Frame::GRAYSCALE) interface.streamGrayscale();
      else if(lidarParam.imageType == Frame::DISTANCE) interface.streamDistance();
      else if(lidarParam.imageType == Frame::AMPLITUDE) interface.streamDistanceAmplitude();
      else if(lidarParam.imageType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE) interface.streamDistanceAmplitudeGrayscale();
      else if(lidarParam.imageType == Frame::DCS) interface.streamDCS();
      else interface.streamDistanceGrayscale();

    }else{
      interface.stopStream();
    }

    if(lidarParam.old_lensCenterOffsetX != lidarParam.lensCenterOffsetX || lidarParam.old_lensCenterOffsetY != lidarParam.lensCenterOffsetY || lidarParam.old_lensType != lidarParam.lensType){
      cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lidarParam.lensCenterOffsetX, lidarParam.lensCenterOffsetY, lidarParam.lensType);
      lidarParam.old_lensCenterOffsetX = lidarParam.lensCenterOffsetX;
      lidarParam.old_lensCenterOffsetY = lidarParam.lensCenterOffsetY;
      lidarParam.old_lensType = lidarParam.lensType;

    }

    //startStream = true;
    printf("setReconfigure OK!\n\n");
    waitKey(1);
    startStreaming();
  }

  void roboscanPublisher::initialise()
  {
    printf("Init roboscan_nsl3130 node\n");

    interface.stopStream();
    
    lidarParam.lensType = 2;
    lidarParam.lensCenterOffsetX = 0;
    lidarParam.lensCenterOffsetY = 0;
    lidarParam.startStream = false;
    lidarParam.imageType = 4; 
    lidarParam.hdr_mode = 0; //0 - hdr off, 1 - hdr spatial, 2 - hdr temporal
    lidarParam.int0 = 800;
    lidarParam.int1 = 100;
    lidarParam.int2 = 50;
    lidarParam.intGr = 100; //integration times
    lidarParam.frequencyModulation = 1;
    lidarParam.modIndex = 0;
    lidarParam.channel = 0;
    lidarParam.minAmplitude = 100;
    lidarParam.medianFilter = false;
    lidarParam.averageFilter = false;
    lidarParam.temporalFilterFactor = 0;
    lidarParam.temporalFilterThreshold = 0;
    lidarParam.edgeThreshold = 0;
    lidarParam.temporalEdgeThresholdLow = 0;
    lidarParam.temporalEdgeThresholdHigh = 0;
    lidarParam.interferenceDetectionLimit = 0;
    lidarParam.useLastValue = false;
    lidarParam.cartesian = true;
    lidarParam.publishPointCloud = true;
    
    lidarParam.roi_leftX = 0;
    lidarParam.roi_topY = 0;
    lidarParam.roi_rightX = 319;
    lidarParam.roi_bottomY = 239;
    lidarParam.transformAngle = 0;

    lidarParam.cvShow = false;
    //roi_height


    rclcpp::Parameter pLensType("B. lensType", lidarParam.lensType);
    rclcpp::Parameter pImageType("C. imageType", lidarParam.imageType);
    rclcpp::Parameter pHdr_mode("D. hdr_mode", lidarParam.hdr_mode);
    rclcpp::Parameter pInt0("E. int0", lidarParam.int0);
    rclcpp::Parameter pInt1("F. int1", lidarParam.int1);
    rclcpp::Parameter pInt2("G. int2", lidarParam.int2);
    rclcpp::Parameter pIntGr("H. intGr", lidarParam.intGr);
    rclcpp::Parameter pMinAmplitude("I. minAmplitude", lidarParam.minAmplitude);
    rclcpp::Parameter pModIndex("J. modIndex", lidarParam.modIndex);
    rclcpp::Parameter pChannel("K. channel", lidarParam.channel);
    rclcpp::Parameter pRoi_leftX("L. roi_leftX", lidarParam.roi_leftX);
    rclcpp::Parameter pRoi_topY("M. roi_topY", lidarParam.roi_topY);
    rclcpp::Parameter pRoi_rightX("N. roi_rightX", lidarParam.roi_rightX);
    rclcpp::Parameter pRoi_bottomY("O. roi_bottomY", lidarParam.roi_bottomY);
    rclcpp::Parameter pTransformAngle("P. transformAngle", lidarParam.transformAngle);
    rclcpp::Parameter pCvShow("A. cvShow", lidarParam.cvShow);

    this->declare_parameter<int>("B. lensType", lidarParam.lensType);
    this->declare_parameter<int>("C. imageType", lidarParam.imageType);
    this->declare_parameter<int>("D. hdr_mode", lidarParam.hdr_mode);
    this->declare_parameter<int>("E. int0", lidarParam.int0);
    this->declare_parameter<int>("F. int1", lidarParam.int1);
    this->declare_parameter<int>("G. int2", lidarParam.int2);
    this->declare_parameter<int>("H. intGr",lidarParam.intGr);
    this->declare_parameter<int>("I. minAmplitude", lidarParam.minAmplitude);
    this->declare_parameter<int>("J. modIndex", lidarParam.modIndex);
    this->declare_parameter<int>("K. channel", lidarParam.channel);
    this->declare_parameter<int>("L. roi_leftX", lidarParam.roi_leftX);
    this->declare_parameter<int>("M. roi_topY", lidarParam.roi_topY);
    this->declare_parameter<int>("N. roi_rightX", lidarParam.roi_rightX);
    this->declare_parameter<int>("O. roi_bottomY", lidarParam.roi_bottomY);
    this->declare_parameter<double>("P. transformAngle", lidarParam.transformAngle);
    this->declare_parameter<bool>("A. cvShow", lidarParam.cvShow);
    
    this->set_parameter(pLensType);
    this->set_parameter(pImageType);
    this->set_parameter(pHdr_mode);
    this->set_parameter(pInt0);
    this->set_parameter(pInt1);
    this->set_parameter(pInt2);
    this->set_parameter(pIntGr);
    this->set_parameter(pMinAmplitude);
    this->set_parameter(pModIndex);
    this->set_parameter(pChannel);
    this->set_parameter(pRoi_leftX);
    this->set_parameter(pRoi_topY);
    this->set_parameter(pRoi_rightX);
    this->set_parameter(pRoi_bottomY);
    this->set_parameter(pTransformAngle);
    this->set_parameter(pCvShow);

    //std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cameraSub");
    //cameraInfoService = node->create_service<sensor_msgs::srv::SetCameraInfo>("cameraSub", &setCameraInfo);
    connectionCameraInfo = interface.subscribeCameraInfo([&](std::shared_ptr<CameraInfo> ci) -> void { updateCameraInfo(ci); });
    connectionFrames = interface.subscribeFrame([&](std::shared_ptr<Frame> f) -> void {  updateFrame(f); });
    cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lidarParam.lensCenterOffsetX, lidarParam.lensCenterOffsetY, lidarParam.lensType); //0.02 mm - sensor pixel size

  }


  void roboscanPublisher::setParameters()
  {
 
    printf("setParameters\n");
    interface.stopStream();

    interface.setMinAmplitude(lidarParam.minAmplitude);
    interface.setIntegrationTime(lidarParam.int0, lidarParam.int1, lidarParam.int2, lidarParam.intGr, lidarParam.grayscaleIlluminationMode);
        
    interface.setHDRMode((uint8_t)lidarParam.hdr_mode);
    interface.setFilter(lidarParam.medianFilter, lidarParam.averageFilter, static_cast<uint16_t>(lidarParam.temporalFilterFactor * 1000), lidarParam.temporalFilterThreshold, lidarParam.edgeThreshold,
                        lidarParam.temporalEdgeThresholdLow, lidarParam.temporalEdgeThresholdHigh, lidarParam.interferenceDetectionLimit, lidarParam.useLastValue);

    interface.setAdcOverflowSaturation(lidarParam.bAdcOverflow, lidarParam.bSaturation);
    interface.setGrayscaleIlluminationMode(lidarParam.grayscaleIlluminationMode);
   
    
    if(lidarParam.frequencyModulation == 0) lidarParam.modIndex = 1;
    else if(lidarParam.frequencyModulation == 1)  lidarParam.modIndex = 0;
    else    lidarParam.modIndex = lidarParam.frequencyModulation;

    interface.setModulation(lidarParam.modIndex, lidarParam.channel);
    printf("modIndex = %d\n", lidarParam.modIndex);
    interface.setRoi(lidarParam.roi_leftX, lidarParam.roi_topY, lidarParam.roi_rightX, lidarParam.roi_bottomY);


    if(lidarParam.old_lensCenterOffsetX != lidarParam.lensCenterOffsetX || lidarParam.old_lensCenterOffsetY != lidarParam.lensCenterOffsetY || lidarParam.old_lensType != lidarParam.lensType){
      cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lidarParam.lensCenterOffsetX, lidarParam.lensCenterOffsetY, lidarParam.lensType);
      lidarParam.old_lensCenterOffsetX = lidarParam.lensCenterOffsetX;
      lidarParam.old_lensCenterOffsetY = lidarParam.lensCenterOffsetY;
      lidarParam.old_lensType = lidarParam.lensType;
    }
    lidarParam.cvShow = false;
    printf("setParameters OK\n");

  }


  void roboscanPublisher::startStreaming()
  {
    printf("startStream\n");
    //startStream = true;
    switch(lidarParam.imageType) {
    case Frame::GRAYSCALE:
      interface.streamGrayscale();
      printf("streaming grayscale\n");
      break;
    case Frame::DISTANCE:
      interface.streamDistance();
      printf("streaming distance\n");
      break;
    case Frame::AMPLITUDE:
      interface.streamDistanceAmplitude();
      printf("streaming distance-amplitude\n");
      break;
    case Frame::DISTANCE_AND_GRAYSCALE:
      interface.streamDistanceGrayscale();
      printf("streaming distance-grayscale\n");
      break;
    case Frame::DISTANCE_AMPLITUDE_GRAYSCALE:
      interface.streamDistanceAmplitudeGrayscale();
      printf("streaming distance-amplitude-grayscale\n");
      break;
    case Frame::DCS:
      interface.streamDCS();
      printf("streaming DCS\n");
      break;
    default:
      printf("stream break\n");
      break;
    }
  }
  int roboscanPublisher::Convert_To_RGB24( float fValue, RGB888Pixel *nRGBData, float fMinValue, float fMaxValue)
  {
    if(fValue == ADC_OVERFLOW)
    {
      nRGBData->r = 169;//R
      nRGBData->g = 14;//G
      nRGBData->b = 255;//B
    }
    else if(fValue == SATURATION)
    {
      nRGBData->r = 255;//R
      nRGBData->g = 0;//G
      nRGBData->b = 128;//B
    }
    else if(fValue == INTERFERENCE)
    {
      nRGBData->r = 0;//R
      nRGBData->g = 0;//G
      nRGBData->b = 0;//B
    }
    else if(fValue == 0) //Invalide Pixel
    {
      nRGBData->r = 0;//R
      nRGBData->g = 0;//G
      nRGBData->b = 0;//B
    }
    else if(fValue < fMinValue)
    {
      nRGBData->r = 255;//R
      nRGBData->g = 0;//G
      nRGBData->b = 0;//B
    }
    else if(fValue > fMaxValue)
    {
      nRGBData->r = 255;//R
      nRGBData->g = 0;//G
      nRGBData->b = 255;//B
    }
    else
    {
      float fColorWeight;
      fColorWeight = (fValue-fMinValue) / (fMaxValue-fMinValue);

      if( (fColorWeight <= 1.0f) && (fColorWeight > 0.8f) )
      {
        nRGBData->r = (unsigned char)(255 * ((fColorWeight - 0.8f) / 0.2f));//값에 따라 증가
        nRGBData->g = 0;
        nRGBData->b = 255;
      } 
      else if( (fColorWeight <= 0.8f) && (fColorWeight > 0.6f) )
      {
        nRGBData->r = 0;
        nRGBData->g = (unsigned char)(255 * (1.0f - (fColorWeight - 0.6f) / 0.2f));//값에 따라 감소
        nRGBData->b = 255;
      }
      else if( (fColorWeight <= 0.6f) && (fColorWeight > 0.4f) )
      {
        nRGBData->r = 0;
        nRGBData->g = 255;
        nRGBData->b = (unsigned char)(255 * ((fColorWeight - 0.4f) / 0.2f));//값에 따라 증가
      }
      else if( (fColorWeight <= 0.4f) && (fColorWeight > 0.2f) )
      {
        nRGBData->r = (unsigned char)(255 * (1.0f - (fColorWeight - 0.2f) / 0.2f));//값에 따라 감소
        nRGBData->g = 255;
        nRGBData->b = 0;
      }
      else if( (fColorWeight <= 0.2f) && (fColorWeight >= 0.0f) )
      {
        nRGBData->r = 255;
        nRGBData->g = (unsigned char)(255 * ((fColorWeight - 0.0f) / 0.2f));//값에 따라 증가
        nRGBData->b = 0;
      }
      else
      {
        nRGBData->r = 0;
        nRGBData->g = 0;
        nRGBData->b = 0;
      }
    }

    return true;
  }

  void roboscanPublisher::getGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
  {   
    if (value == SATURATION)
    {
      imageLidar.at<Vec3b>(y, x)[0] = 128;
      imageLidar.at<Vec3b>(y, x)[1] = 0;
      imageLidar.at<Vec3b>(y, x)[2] = 255; 
    }
    else if (value == ADC_OVERFLOW)
    {
      imageLidar.at<Vec3b>(y, x)[0] = 255;
      imageLidar.at<Vec3b>(y, x)[1] = 14;
      imageLidar.at<Vec3b>(y, x)[2] = 169; 
    }
    else if (value > end_range)
    {
      imageLidar.at<Vec3b>(y, x)[0] = 255;
      imageLidar.at<Vec3b>(y, x)[1] = 255;
      imageLidar.at<Vec3b>(y, x)[2] = 255; 
    }
    else if (value < 0)
    {
      imageLidar.at<Vec3b>(y, x)[0] = 0;
      imageLidar.at<Vec3b>(y, x)[1] = 0;
      imageLidar.at<Vec3b>(y, x)[2] = 0; 
    }
    else
    {
      int color = value * (255/end_range);

      //printf("color index = %d\n", color);

      imageLidar.at<Vec3b>(y, x)[0] = color;
      imageLidar.at<Vec3b>(y, x)[1] = color;
      imageLidar.at<Vec3b>(y, x)[2] = color; 
    }
}


  bool roboscanPublisher::setCameraInfo(sensor_msgs::srv::SetCameraInfo::Request& req, sensor_msgs::srv::SetCameraInfo::Response& res)
  {
    req.camera_info.width  = cameraInfo.width;
    req.camera_info.height = cameraInfo.height;
    req.camera_info.roi    = cameraInfo.roi;

    //cameraInfoPublisher->publish(req.camera_info);
  
    res.success = true;
    res.status_message = "";
    return true;
  }
 
  void roboscanPublisher::updateCameraInfo(std::shared_ptr<CameraInfo> ci)
  {
    cameraInfo.width = ci->width;
    cameraInfo.height = ci->height;
    cameraInfo.roi.x_offset = ci->roiX0;
    cameraInfo.roi.y_offset = ci->roiY0;
    cameraInfo.roi.width = ci->roiX1 - ci->roiX0;
    cameraInfo.roi.height = ci->roiY1 - ci->roiY0;
  }


  void roboscanPublisher::updateFrame(std::shared_ptr<Frame> frame)
  {
    int x, y, k, l;
    static rclcpp::Clock s_rclcpp_clock;
    auto data_stamp = s_rclcpp_clock.now();

    cv::Mat imageLidar(height, width, CV_8UC3, Scalar(255, 255, 255));

    if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::AMPLITUDE || frame->dataType == Frame::DISTANCE_AND_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
      sensor_msgs::msg::Image imgDistance;
        
      imgDistance.header.stamp = data_stamp;
      imgDistance.header.frame_id = "roboscan_frame";
      imgDistance.height = static_cast<uint32_t>(frame->height);
      imgDistance.width = static_cast<uint32_t>(frame->width);
      imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
      imgDistance.step = imgDistance.width * frame->px_size;
      imgDistance.is_bigendian = 0;
      imgDistance.data = frame->distData;
      imgDistancePub->publish(imgDistance);
    }

    if(frame->dataType == Frame::AMPLITUDE || frame->dataType == Frame::DISTANCE_AND_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){
      sensor_msgs::msg::Image imgAmpl;
        
      imgAmpl.header.stamp = data_stamp;
      imgAmpl.header.frame_id = "roboscan_frame";
      imgAmpl.height = static_cast<uint32_t>(frame->height);
      imgAmpl.width = static_cast<uint32_t>(frame->width);
      imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
      imgAmpl.step = imgAmpl.width * frame->px_size;
      imgAmpl.is_bigendian = 0;
      imgAmpl.data = frame->amplData;
      imgAmplPub->publish(imgAmpl);
    }
    
    if(frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){
      sensor_msgs::msg::Image imgGray;
      
        
      imgGray.header.stamp = data_stamp;
      imgGray.header.frame_id = "roboscan_frame";
      imgGray.height = static_cast<uint32_t>(frame->height);
      imgGray.width = static_cast<uint32_t>(frame->width);
      imgGray.encoding = sensor_msgs::image_encodings::MONO16;
      imgGray.step = imgGray.width * frame->px_size;
      imgGray.is_bigendian = 0;
      imgGray.data = frame->grayData;
      imgGrayPub->publish(imgGray);
    }

    if(frame->dataType == Frame::DCS){
      sensor_msgs::msg::Image imgDCS;
        
      imgDCS.header.stamp = data_stamp;
      imgDCS.header.frame_id = "roboscan_frame";
      imgDCS.height = static_cast<uint32_t>(frame->height) * 4;
      imgDCS.width = static_cast<uint32_t>(frame->width);
      imgDCS.encoding = sensor_msgs::image_encodings::MONO16;
      imgDCS.step = imgDCS.width * frame->px_size;
      imgDCS.is_bigendian = 0;
      imgDCS.data = frame->dcsData;
      imgDCSPub->publish(imgDCS);
    }


    if(frame->dataType == Frame::GRAYSCALE){
      uint16_t gray; 
      for(k=0, l=0, y=0; y< frame->height; y++){
        for(x=0; x< frame->width; x++, k++, l+=2){
          gray = (frame->amplData[l+1] << 8)  + frame->amplData[l];
          getGrayscaleColor(imageLidar, x, y, gray, 255);

        }
      }
    }

    if(frame->dataType != Frame::GRAYSCALE){
        

      const size_t nPixel = frame->width * frame->height;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      cloud->header.frame_id = "roboscan_frame";
      cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
      cloud->width = static_cast<uint32_t>(frame->width);
      cloud->height = static_cast<uint32_t>(frame->height);
      cloud->is_dense = false;
      cloud->points.resize(nPixel);

      uint16_t distance = 0;
      uint16_t amplitude = 0;
      double px, py, pz;

      RGB888Pixel* pTex1 = new RGB888Pixel[1];
        

      for(k=0, l=0, y=0; y< frame->height; y++){
          for(x=0; x< frame->width; x++, k++, l+=2){
            pcl::PointXYZI &p = cloud->points[k];
            if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::AMPLITUDE || frame->dataType == Frame::DISTANCE_AND_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE)
              distance = (frame->distData[l+1] << 8) + frame->distData[l];

            if(frame->dataType == Frame::DCS)
              distance = (frame->dcsData[l+1] << 8) + frame->dcsData[l];

            if(frame->dataType == Frame::AMPLITUDE)
              amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];

            
            //distance 
            if(distance == LOW_AMPLITUDE)
              distance = 0;
            

            Convert_To_RGB24((double)distance, pTex1, 0.0f, 12500.0f);
            imageLidar.at<Vec3b>(y, x)[0] = pTex1->b;
            imageLidar.at<Vec3b>(y, x)[1] = pTex1->g;
            imageLidar.at<Vec3b>(y, x)[2] = pTex1->r;
#if 0                
                if(x == 160 && y == 120)
                {
                  RCLCPP_INFO(this->get_logger(), "distance : %d", distance);
                }
#endif
            if (distance > 0 && distance < 64000){

              if(lidarParam.cartesian){
                cartesianTransform.transformPixel(x, y, distance, px, py, pz, lidarParam.transformAngle);
                  p.x = static_cast<float>(pz / 1000.0); //mm -> m
                  p.y = static_cast<float>(px / 1000.0);
                  p.z = static_cast<float>(-py / 1000.0);

                  if(frame->dataType == Frame::AMPLITUDE) p.intensity = static_cast<float>(amplitude);
                  else p.intensity = static_cast<float>(pz / 1000.0);

                   
                  }else{
                    p.x = distance / 1000.0;
                    p.y = -(160-x) / 100.0;
                    p.z = (120-y) / 100.0;
                    if(frame->dataType == Frame::AMPLITUDE) p.intensity =  static_cast<float>(amplitude);
                    else p.intensity = static_cast<float>(distance / 1000.0);
                  }
                
              }else{
                p.x = std::numeric_limits<float>::quiet_NaN();
                p.y = std::numeric_limits<float>::quiet_NaN();
                p.z = std::numeric_limits<float>::quiet_NaN();
            }         

          }
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = data_stamp;
        msg.header.frame_id = "roboscan_frame";
        pointcloudPub->publish(msg);  

        delete[] pTex1;        
      }
      if(frame->dataType != Frame::GRAYSCALE){

      }

      if(lidarParam.cvShow == true)
      {
        imshow("NSL-31310AA IMAGE", imageLidar);
      }
      else cv::destroyAllWindows();

      waitKey(1);
  }



int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<roboscanPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
