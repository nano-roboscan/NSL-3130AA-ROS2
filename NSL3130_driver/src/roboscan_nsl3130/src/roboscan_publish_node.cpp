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

//Lidar Fixed Parameters
#define LOW_AMPLITUDE               64001
#define ADC_OVERFLOW                64002
#define SATURATION                  64003
#define BAD_PIXEL                   64004
#define INFERENCE                   64007
#define EDGE_FILTERED               64008


using namespace nanosys;
using namespace std::chrono_literals;
using namespace cv;

//set Parameters~
int imageType; //image and aquisition type: 0 - grayscale, 1 - distance, 2 - distance_amplitude
int lensType;  //0- wide field, 1- standard field, 2 - narrow field
int old_lensType;
bool medianFilter;
bool averageFilter;
double temporalFilterFactor;
int temporalFilterThreshold;
int edgeThreshold;
int temporalEdgeThresholdLow;
int temporalEdgeThresholdHigh;
int interferenceDetectionLimit;
bool startStream;
bool useLastValue;
bool publishPointCloud;
bool cartesian;
int channel;
int frequencyModulation;
int int0, int1, int2, intGr; //integration times
int hdr_mode; //0 - hdr off, 1 - hdr spatial, 2 - hdr temporal
int minAmplitude;
int lensCenterOffsetX = 0;
int lensCenterOffsetY = 0;
int old_lensCenterOffsetX = 0;
int old_lensCenterOffsetY = 0;
uint8_t modIndex = 0;


int roi_leftX = 0;
int roi_topY = 0;
int roi_rightX = 319;
int roi_bottomY = 239;
// ~set Parmeters

const int width   = 320;
const int width2  = 160;
const int height  = 240;
const int height2 = 120;
const double sensorPixelSizeMM = 0.02; //camera sensor pixel size 20x20 um

uint8_t grayscaleIlluminationMode = 0;
uint8_t bAdcOverflow = 1;
uint8_t bSaturation = 1;

uint32_t frameSeq;

boost::signals2::connection connectionFrames;
boost::signals2::connection connectionCameraInfo;

rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr cameraInfoService;

Interface interface;
CartesianTransform cartesianTransform;
sensor_msgs::msg::CameraInfo cameraInfo;


static rclcpp::Clock s_rclcpp_clock;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgDistancePub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgAmplPub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgGrayPub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgDCSPub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPub;


class roboscanPublisher : public rclcpp::Node { 

public:
  typedef struct _RGB888Pixel
  {
    unsigned char r;
    unsigned char g;
    unsigned char b;
  } RGB888Pixel;

  roboscanPublisher() : Node("roboscan_publish_node")
  { 
    
    initialise();
    setParameters();
    startStreaming();
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&roboscanPublisher::parametersCallback, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "++Start roboscanPublisher()....");
    
  } 

  rcl_interfaces::msg::SetParametersResult parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    for (const auto &param: parameters)
    {
      if (param.get_name() == "hdr_mode")
      {
        hdr_mode = param.as_int();
      }
      else if (param.get_name() == "int0")
      {
        int0 = param.as_int();
      }
      else if (param.get_name() == "int1")
      {
        int1 = param.as_int();
      }
      else if (param.get_name() == "int2")
      {
        int2 = param.as_int();
      }
      else if (param.get_name() == "intGr")
      {
        intGr = param.as_int();
      }
      else if (param.get_name() == "minAmplitude")
      {
        minAmplitude = param.as_int();
      }
      else if (param.get_name() == "modIndex")
      {
        modIndex = param.as_int();
      }
      else if (param.get_name() == "channel")
      {
        channel = param.as_int();
      }
      else if (param.get_name() == "roi_leftX")
      {
        roi_leftX = param.as_int();
      }
      else if (param.get_name() == "roi_topY")
      {
        roi_topY = param.as_int();
      }
      else if (param.get_name() == "roi_rightX")
      {
        roi_rightX = param.as_int();
      }
      else if (param.get_name() == "roi_bottomY")
      {
        roi_bottomY = param.as_int();
      }
    }
    setReconfigure();
    return result;
  }

  void setReconfigure()
  {
    //int0 = integrationTime0;

    printf("setReconfigure\n");

    interface.stopStream();    
    interface.setMinAmplitude(minAmplitude);
    interface.setIntegrationTime(int0, int1, int2, intGr);
        
    interface.setHDRMode((uint8_t)hdr_mode);
    interface.setFilter(medianFilter, averageFilter, static_cast<uint16_t>(temporalFilterFactor * 1000), temporalFilterThreshold, edgeThreshold,
                        temporalEdgeThresholdLow, temporalEdgeThresholdHigh, interferenceDetectionLimit, useLastValue);

    interface.setAdcOverflowSaturation(bAdcOverflow, bSaturation);
    interface.setGrayscaleIlluminationMode(grayscaleIlluminationMode);
   
    uint8_t modIndex;
    if(frequencyModulation == 0) modIndex = 1;
    else if(frequencyModulation == 1)  modIndex = 0;
    else    modIndex = frequencyModulation;

    interface.setModulation(modIndex, channel);
    interface.setRoi(roi_leftX, roi_topY, roi_rightX, roi_bottomY);

    if(startStream){

      if(imageType == Frame::GRAYSCALE) interface.streamGrayscale();
      else if(imageType == Frame::DISTANCE) interface.streamDistance();
      else if(imageType == Frame::AMPLITUDE) interface.streamDistanceAmplitude();
      else if(imageType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE) interface.streamDistanceAmplitudeGrayscale();
      else if(imageType == Frame::DCS) interface.streamDCS();
      else interface.streamDistanceGrayscale();

    }else{
      interface.stopStream();
    }

    if(old_lensCenterOffsetX != lensCenterOffsetX || old_lensCenterOffsetY != lensCenterOffsetY || old_lensType != lensType){
      cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType);
      old_lensCenterOffsetX = lensCenterOffsetX;
      old_lensCenterOffsetY = lensCenterOffsetY;
      old_lensType = lensType;

    }
    startStream = true;
    printf("setReconfigure OK\n\n");

  }

  void initialise()
  {
    printf("Init roboscan_nsl3130 node\n");

    lensType = 2;
    lensCenterOffsetX = 0;
    lensCenterOffsetY = 0;
    startStream = false;
    imageType = 4; 
    hdr_mode = 0; //0 - hdr off, 1 - hdr spatial, 2 - hdr temporal
    int0 = 800;
    int1 = 100;
    int2 = 50;
    intGr = 100; //integration times
    frequencyModulation = 1;
    modIndex = 0;
    channel = 0;
    minAmplitude = 100;
    medianFilter = false;
    averageFilter = false;
    temporalFilterFactor = 0;
    temporalFilterThreshold = 0;
    edgeThreshold = 0;
    temporalEdgeThresholdLow = 0;
    temporalEdgeThresholdHigh = 0;
    interferenceDetectionLimit = 0;
    useLastValue = false;
    cartesian = true;
    publishPointCloud = true;
    
    roi_leftX = 0;
    roi_topY = 0;
    roi_rightX = 319;
    roi_bottomY = 239;
    //roi_height


    rclcpp::Parameter pLensType("lensType", lensType);
    rclcpp::Parameter pImageType("imageType", imageType);
    rclcpp::Parameter pHdr_mode("hdr_mode", hdr_mode);
    rclcpp::Parameter pInt0("int0", int0);
    rclcpp::Parameter pInt1("int1", int1);
    rclcpp::Parameter pInt2("int2", int2);
    rclcpp::Parameter pIntGr("intGr", intGr);
    rclcpp::Parameter pMinAmplitude("minAmplitude", minAmplitude);
    rclcpp::Parameter pModIndex("modIndex", modIndex);
    rclcpp::Parameter pChannel("channel", channel);
    rclcpp::Parameter pRoi_leftX("roi_leftX", roi_leftX);
    rclcpp::Parameter pRoi_topY("roi_topY", roi_topY);
    rclcpp::Parameter pRoi_rightX("roi_rightX", roi_rightX);
    rclcpp::Parameter pRoi_bottomY("roi_bottomY", roi_bottomY);

    this->declare_parameter<int>("lensType", lensType);
    this->declare_parameter<int>("imageType", imageType);
    this->declare_parameter<int>("hdr_mode", hdr_mode);
    this->declare_parameter<int>("int0", int0);
    this->declare_parameter<int>("int1", int1);
    this->declare_parameter<int>("int2", int2);
    this->declare_parameter<int>("intGr", intGr);
    this->declare_parameter<int>("minAmplitude", minAmplitude);
    this->declare_parameter<int>("modIndex", modIndex);
    this->declare_parameter<int>("channel", channel);
    this->declare_parameter<int>("roi_leftX", roi_leftX);
    this->declare_parameter<int>("roi_topY", roi_topY);
    this->declare_parameter<int>("roi_rightX", roi_rightX);
    this->declare_parameter<int>("roi_bottomY", roi_bottomY);
    
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

    //std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cameraSub");
    //cameraInfoService = node->create_service<sensor_msgs::srv::SetCameraInfo>("cameraSub", &setCameraInfo);
    connectionCameraInfo = interface.subscribeCameraInfo([&](std::shared_ptr<CameraInfo> ci) -> void { updateCameraInfo(ci); });
    connectionFrames = interface.subscribeFrame([&](std::shared_ptr<Frame> f) -> void {  updateFrame(f); });
    cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType); //0.02 mm - sensor pixel size

  }


  void setParameters()
  {
 
    printf("setParameters\n");
    interface.stopStream();

    interface.setMinAmplitude(minAmplitude);
    interface.setIntegrationTime(int0, int1, int2, intGr);
        
    interface.setHDRMode((uint8_t)hdr_mode);
    interface.setFilter(medianFilter, averageFilter, static_cast<uint16_t>(temporalFilterFactor * 1000), temporalFilterThreshold, edgeThreshold,
                        temporalEdgeThresholdLow, temporalEdgeThresholdHigh, interferenceDetectionLimit, useLastValue);

    interface.setAdcOverflowSaturation(bAdcOverflow, bSaturation);
    interface.setGrayscaleIlluminationMode(grayscaleIlluminationMode);
   
    
    if(frequencyModulation == 0) modIndex = 1;
    else if(frequencyModulation == 1)  modIndex = 0;
    else    modIndex = frequencyModulation;

    interface.setModulation(modIndex, channel);
    printf("modIndex = %d\n", modIndex);
    //interface.setRoi(roi_leftX, roi_topY, roi_rightX, roi_bottomY);


    if(old_lensCenterOffsetX != lensCenterOffsetX || old_lensCenterOffsetY != lensCenterOffsetY || old_lensType != lensType){
      cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType);
      old_lensCenterOffsetX = lensCenterOffsetX;
      old_lensCenterOffsetY = lensCenterOffsetY;
      old_lensType = lensType;
    }
    printf("setParameters OK\n");
  }

  void startStreaming()
  {
    printf("startStream\n");
    //startStream = true;
    switch(imageType) {
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
  int Convert_To_RGB24( float fValue, RGB888Pixel *nRGBData, float fMinValue, float fMaxValue)
  {
    if(fValue == 0) //Invalide Pixel
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

  bool setCameraInfo(sensor_msgs::srv::SetCameraInfo::Request& req, sensor_msgs::srv::SetCameraInfo::Response& res)
  {
    req.camera_info.width  = cameraInfo.width;
    req.camera_info.height = cameraInfo.height;
    req.camera_info.roi    = cameraInfo.roi;

    //cameraInfoPublisher->publish(req.camera_info);
  
    res.success = true;
    res.status_message = "";
    return true;
  }
 
  void updateCameraInfo(std::shared_ptr<CameraInfo> ci)
  {
    cameraInfo.width = ci->width;
    cameraInfo.height = ci->height;
    cameraInfo.roi.x_offset = ci->roiX0;
    cameraInfo.roi.y_offset = ci->roiY0;
    cameraInfo.roi.width = ci->roiX1 - ci->roiX0;
    cameraInfo.roi.height = ci->roiY1 - ci->roiY0;
  }


  void updateFrame(std::shared_ptr<Frame> frame)
  {
    int x, y, k, l;
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto data_stamp = s_rclcpp_clock.now();

    if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::AMPLITUDE || frame->dataType == Frame::DISTANCE_AND_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
      sensor_msgs::msg::Image imgDistance;
      imgDistancePub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
        
      imgDistance.header.stamp = s_rclcpp_clock.now();
      imgDistance.header.frame_id = std::to_string(frame->frame_id);
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
      imgAmplPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanAmpl", qos_profile); 
        
      imgAmpl.header.stamp = s_rclcpp_clock.now();
      imgAmpl.header.frame_id = std::to_string(frame->frame_id);
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
      imgGrayPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanGray", qos_profile); 
        
      imgGray.header.stamp = s_rclcpp_clock.now();
      imgGray.header.frame_id = std::to_string(frame->frame_id);
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
      imgDCSPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDCS", qos_profile); 
        
      imgDCS.header.stamp = s_rclcpp_clock.now();
      imgDCS.header.frame_id = std::to_string(frame->frame_id);
      imgDCS.height = static_cast<uint32_t>(frame->height) * 4;
      imgDCS.width = static_cast<uint32_t>(frame->width);
      imgDCS.encoding = sensor_msgs::image_encodings::MONO16;
      imgDCS.step = imgDCS.width * frame->px_size;
      imgDCS.is_bigendian = 0;
      imgDCS.data = frame->dcsData;
      imgDCSPub->publish(imgDCS);
    }

    if(frame->dataType != Frame::GRAYSCALE){
        
      pointcloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 

      const size_t nPixel = frame->width * frame->height;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      cloud->header.frame_id = "map";
      cloud->header.stamp = pcl_conversions::toPCL(s_rclcpp_clock.now());
      cloud->width = static_cast<uint32_t>(frame->width);
      cloud->height = static_cast<uint32_t>(frame->height);
      cloud->is_dense = false;
      cloud->points.resize(nPixel);

      uint16_t distance, amplitude;
      double px, py, pz;

      RGB888Pixel* pTex1 = new RGB888Pixel[1];
      cv::Mat imageLidar(height, width, CV_8UC3, Scalar(255, 255, 255));
        

      for(k=0, l=0, y=0; y< frame->height; y++){
          for(x=0; x< frame->width; x++, k++, l+=2){
            pcl::PointXYZI &p = cloud->points[k];
            distance = (frame->distData[l+1] << 8) + frame->distData[l];

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
            if (distance > 0 && distance < 65000){

              if(cartesian){
                cartesianTransform.transformPixel(x, y, distance, px, py, pz);
                  p.x = static_cast<float>(px / 1000.0); //mm -> m
                  p.y = static_cast<float>(py / 1000.0);
                  p.z = static_cast<float>(pz / 1000.0);

                  if(frame->dataType == Frame::AMPLITUDE) p.intensity = static_cast<float>(amplitude);
                  else p.intensity = static_cast<float>(pz / 1000.0);

                  }else{
                    p.x = x / 100.0;
                    p.y = y / 100.0;
                    p.z = distance / 1000.0;
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
        msg.header.frame_id = "/map";
        pointcloudPub->publish(msg);  

        imshow("NSL-31310AA DISTANCE", imageLidar);
        waitKey(1);
        delete[] pTex1;
        
      }
  }


private:
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};


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

