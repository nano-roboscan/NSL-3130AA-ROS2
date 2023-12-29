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
using namespace std;

#define WIN_NAME "NSL-3130AA IMAGE"
#define  MAX_LEVELS  9
#define NUM_COLORS     		30000

std::atomic<int> x_start = -1, y_start = -1;


static void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	std::ignore = flags;
	std::ignore = user_data;
	
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		x_start = x;
		y_start = y;
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
	}
}

roboscanPublisher::roboscanPublisher() : 
	Node("roboscan_publish_node")
#ifdef image_transfer_function
	,nodeHandle(std::shared_ptr<roboscanPublisher>(this, [](auto *) {}))
	,imageTransport(nodeHandle)
	,imagePublisher(imageTransport.advertise("roboscanImage", 1000))
#endif	
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

	int numSteps = NUM_COLORS;
	unsigned char red, green, blue;

	for(int i=0;  i< numSteps; i++)
	{
	  createColorMapPixel(numSteps, i, red, green, blue);
	  colorVector.push_back(Vec3b(blue, green, red));
	}

	reconfigure = false;
	mouseXpos = -1;
	mouseYpos = -1;
	runThread = true;
    publisherThread.reset(new boost::thread(boost::bind(&roboscanPublisher::thread_callback, this)));
    printf("\nRun rqt to view the image!\n");
    
} 

roboscanPublisher::~roboscanPublisher()
{
	runThread = false;
	publisherThread->join();

    printf("\nEnd roboscanPublisher()!\n");
}

void roboscanPublisher::thread_callback()
{
	cv::destroyAllWindows();
	while(runThread){

		if( reconfigure ){
			reconfigure = false;
			setReconfigure();
		}
				
		if( backupFrame.size() > 0 ){
			publishFrame(backupFrame.front());
			backupFrame.erase(backupFrame.begin());
		}

		waitKey(1);
	}

	while( backupFrame.size() > 0 ){
		backupFrame.erase(backupFrame.begin());
	}

	cv::destroyAllWindows();
	printf("end thread_callback\n");
}

double roboscanPublisher::interpolate( double x, double x0, double y0, double x1, double y1){

    if( x1 == x0 ){
        return y0;
    } else {
        return ((x-x0)*(y1-y0)/(x1-x0) + y0);
    }

}

void roboscanPublisher::createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue)
{
    double k = 1;
    double BIT0 = -0.125 * k - 0.25;
    double BIT1 = BIT0 + 0.25 * k;
    double BIT2 = BIT1 + 0.25 * k;
    double BIT3 = BIT2 + 0.25 * k;

    double G0 = BIT1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;

    double R0 = BIT2;
    double R1 = R0 + 0.25 * k;
    double R2 = R1 + 0.25 * k;
    double R3 = R2 + 0.25 * k + 0.25;

    double i = (double)indx/(double)numSteps - 0.25 * k;

    if( i>= R0 && i < R1 ){
        red = interpolate(i, R0, 0, R1, 255);
    } else if((i >= R1) && (i < R2)){
        red = 255;
    } else if((i >= R2) && (i < R3)) {
        red = interpolate(i, R2, 255, R3, 0);
    } else {
        red = 0;
    }

    if( i>= G0 && i < G1 ){
        green = interpolate(i, G0, 0, G1, 255);
    } else if((i>=G1)&&(i<G2)){
        green = 255;
    } else if((i >= G2)&&(i < G3)){
        green = interpolate(i, G2, 255, G3, 0);
    } else {
        green = 0;
    }


    if( i>= BIT0 && i < BIT1 ){
        blue = interpolate(i, BIT0, 0, BIT1, 255);
    } else if((i >= BIT1)&&(i < BIT2)){
        blue = 255;
    } else if((i >= BIT2)&&(i < BIT3)) {
        blue = interpolate(i, BIT2, 255, BIT3, 0);
    } else{
        blue = 0;
    }

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
			int imgType = param.as_int();
			if( lidarParam.imageType != imgType ){
				lidarParam.imageType = imgType;
				lidarParam.changedCvShow = true;
			}
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
			lidarParam.frequencyModulation = param.as_int();
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
		else if (param.get_name() == "Q. cutPixels")
		{
			lidarParam.cutPixels = param.as_int();
		}
		else if (param.get_name() == "Q. cutPixels")
		{
			lidarParam.cutPixels = param.as_int();
		}
		else if (param.get_name() == "R. medianFilter")
		{
			lidarParam.medianFilter = param.as_bool();
		}
		else if (param.get_name() == "S. averageFilter")
		{
			lidarParam.averageFilter = param.as_bool();
		}
		else if (param.get_name() == "T. temporalFilterFactor")
		{
			lidarParam.temporalFilterFactor = param.as_double();
		}
		else if (param.get_name() == "T. temporalFilterFactorThreshold")
		{
			lidarParam.temporalFilterThreshold = param.as_int();
		}
		else if (param.get_name() == "U. edgeFilterThreshold")
		{
			lidarParam.edgeFilterThreshold = param.as_int();
		}
		/*
		else if (param.get_name() == "W. temporalEdgeThresholdLow")
		{
			lidarParam.temporalEdgeThresholdLow = param.as_int();
		}
		else if (param.get_name() == "X. temporalEdgeThresholdHigh")
		{
			lidarParam.temporalEdgeThresholdHigh = param.as_int();
		}
		*/
		else if (param.get_name() == "V. interferenceDetectionLimit")
		{
			lidarParam.interferenceDetectionLimit = param.as_int();
		}
		else if (param.get_name() == "V. useLastValue")
		{
			lidarParam.useLastValue = param.as_bool();
		}
		else if (param.get_name() == "A. cvShow")
		{
			
			bool showCv = param.as_bool();
			if( lidarParam.cvShow != showCv ){
				lidarParam.cvShow = showCv;
				lidarParam.changedCvShow = true;
			}
			
		}
		else if (param.get_name() == "W. dualBeam")
		{
			lidarParam.dualBeam = param.as_int();
			if( lidarParam.dualBeam > 2 ) lidarParam.dualBeam = 0;
		}
		else if (param.get_name() == "X. grayscale LED")
		{
			lidarParam.grayscaleIlluminationMode = param.as_bool();
		}
		else if (param.get_name() == "Y. PointColud EDGE")
		{
			lidarParam.pointCloudEdgeFilter = param.as_bool();
		}
		else if (param.get_name() == "Z. MaxDistance")
		{
			lidarParam.maxDistance = param.as_int();
		}
		else if (param.get_name() == "0. IP Addr")
		{
			string tmpIp = param.as_string();
			if( tmpIp != lidarParam.ipAddr ) {
				printf("changed IP addr %s -> %s\n", lidarParam.ipAddr.c_str(), tmpIp.c_str());

				lidarParam.changedIpInfo = true;
				lidarParam.ipAddr = tmpIp;
			}
		}
		else if (param.get_name() == "1. Net Mask")
		{
			string tmpIp = param.as_string();
			if( tmpIp != lidarParam.netMask ) {
				printf("changed Netmask addr %s -> %s\n", lidarParam.ipAddr.c_str(), tmpIp.c_str());
				lidarParam.changedIpInfo = true;
				lidarParam.netMask= tmpIp;
			}
		}
		else if (param.get_name() == "2. GW Addr")
		{
			string tmpIp = param.as_string();
			if( tmpIp != lidarParam.gwAddr ) {
				printf("changed Gw addr %s -> %s\n", lidarParam.ipAddr.c_str(), tmpIp.c_str());
				lidarParam.changedIpInfo = true;
				lidarParam.gwAddr= tmpIp;
			}
		}
	}

	reconfigure = true;
	return result;
}

void roboscanPublisher::setReconfigure()
{

	printf("setReconfigure\n");

	if( lidarParam.changedIpInfo ){
		lidarParam.changedIpInfo = false;
		interface.setIpAddr( lidarParam.ipAddr, lidarParam.netMask, lidarParam.gwAddr);
	}
	
	interface.stopStream();    
	interface.setMinAmplitude(lidarParam.minAmplitude);
	interface.setIntegrationTime(lidarParam.int0, lidarParam.int1, lidarParam.int2, lidarParam.intGr, lidarParam.grayscaleIlluminationMode);
	    
	interface.setHDRMode((uint8_t)lidarParam.hdr_mode);
	interface.setFilter(lidarParam.medianFilter, lidarParam.averageFilter, static_cast<uint16_t>(lidarParam.temporalFilterFactor * 1000), lidarParam.temporalFilterThreshold, lidarParam.edgeFilterThreshold,
	                    lidarParam.temporalEdgeThresholdLow, lidarParam.temporalEdgeThresholdHigh, lidarParam.interferenceDetectionLimit, lidarParam.useLastValue);

	interface.setAdcOverflowSaturation(lidarParam.bAdcOverflow, lidarParam.bSaturation);
	interface.setGrayscaleIlluminationMode(lidarParam.grayscaleIlluminationMode);
	interface.setDualBeam(lidarParam.dualBeam, true);

	if(lidarParam.frequencyModulation == 0) lidarParam.modIndex = 1;
	else if(lidarParam.frequencyModulation == 1)  lidarParam.modIndex = 0;
	else if(lidarParam.frequencyModulation == 2)  lidarParam.modIndex = 2;
	else    lidarParam.modIndex = 3;

	interface.setModulation(lidarParam.modIndex, lidarParam.channel);
	interface.setRoi(lidarParam.roi_leftX, lidarParam.roi_topY, lidarParam.roi_rightX, lidarParam.roi_bottomY);

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

	setWinName();

}

void roboscanPublisher::setWinName()
{
	bool changedCvShow = lidarParam.changedCvShow;
	lidarParam.changedCvShow = false;
	
	if( changedCvShow ){
		cv::destroyAllWindows();
	}
	
	if( lidarParam.cvShow == false || changedCvShow == false ) return;
	
	if( lidarParam.imageType == 0 ){
		sprintf(winName,"%s(Gray)", WIN_NAME);
	}
	else if( lidarParam.imageType == 1 ){
		sprintf(winName,"%s(Dist)", WIN_NAME);
	}
	else if( lidarParam.imageType == 2 ){
		sprintf(winName,"%s(Dist/Ampl)", WIN_NAME);
	}
	else if( lidarParam.imageType == 3 ){
		sprintf(winName,"%s(DCS)", WIN_NAME);
	}
	else if( lidarParam.imageType == 4 ){
		sprintf(winName,"%s(Dist/Gray)", WIN_NAME);
	}
	else if( lidarParam.imageType == 5 ){
		sprintf(winName,"%s(Dist/Ampl/Gray)", WIN_NAME);
	}

	
	cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
	cv::setWindowProperty(winName, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(winName, callback_mouse_click, NULL);
}
void roboscanPublisher::initialise()
{
	printf("Init roboscan_nsl3130 node\n");


	interface.stopStream();

	lidarParam.lensType = 2;
	lidarParam.lensCenterOffsetX = 0;
	lidarParam.lensCenterOffsetY = 0;
	lidarParam.startStream = false;
	lidarParam.imageType = 2; 
	lidarParam.hdr_mode = 0; //0 - hdr off, 1 - hdr spatial, 2 - hdr temporal
	lidarParam.int0 = 1500;
	lidarParam.int1 = 100;
	lidarParam.int2 = 50;
	lidarParam.intGr = 100; //integration times
	lidarParam.frequencyModulation = 0;
	lidarParam.modIndex = 0;
	lidarParam.channel = 0;
	lidarParam.minAmplitude = 100;
	lidarParam.medianFilter = false;
	lidarParam.averageFilter = false;
	lidarParam.temporalFilterFactor = 0.3;
	lidarParam.temporalFilterThreshold = 200;
	lidarParam.edgeFilterThreshold = 0;
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
	lidarParam.cutPixels = 0;

	lidarParam.cvShow = false;
	lidarParam.pointCloudEdgeFilter = false;
	lidarParam.dualBeam = 2;

	lidarParam.ipAddr = "192.168.0.220";
	lidarParam.netMask = "255.255.255.0";
	lidarParam.gwAddr = "192.168.0.1";
	lidarParam.maxDistance = 12500;
	//roi_height

	setWinName();

	rclcpp::Parameter pIPAddr("0. IP Addr", lidarParam.ipAddr);
//	rclcpp::Parameter pNetMask("1. Net Mask", lidarParam.netMask);
//	rclcpp::Parameter pGWAddr("2. GW Addr", lidarParam.gwAddr);

	rclcpp::Parameter pLensType("B. lensType", lidarParam.lensType);
	rclcpp::Parameter pImageType("C. imageType", lidarParam.imageType);
	rclcpp::Parameter pHdr_mode("D. hdr_mode", lidarParam.hdr_mode);
	rclcpp::Parameter pInt0("E. int0", lidarParam.int0);
	rclcpp::Parameter pInt1("F. int1", lidarParam.int1);
	rclcpp::Parameter pInt2("G. int2", lidarParam.int2);
	rclcpp::Parameter pIntGr("H. intGr", lidarParam.intGr);
	rclcpp::Parameter pMinAmplitude("I. minAmplitude", lidarParam.minAmplitude);
	rclcpp::Parameter pModIndex("J. modIndex", lidarParam.frequencyModulation);
	rclcpp::Parameter pChannel("K. channel", lidarParam.channel);
	rclcpp::Parameter pRoi_leftX("L. roi_leftX", lidarParam.roi_leftX);
	rclcpp::Parameter pRoi_topY("M. roi_topY", lidarParam.roi_topY);
	rclcpp::Parameter pRoi_rightX("N. roi_rightX", lidarParam.roi_rightX);
	rclcpp::Parameter pRoi_bottomY("O. roi_bottomY", lidarParam.roi_bottomY);
	rclcpp::Parameter pTransformAngle("P. transformAngle", lidarParam.transformAngle);
	rclcpp::Parameter pCutpixels("Q. cutPixels", lidarParam.cutPixels);
	rclcpp::Parameter pMedianFilter("R. medianFilter", lidarParam.medianFilter);
	rclcpp::Parameter pAverageFilter("S. averageFilter", lidarParam.averageFilter);
	rclcpp::Parameter pTemporalFilterFactor("T. temporalFilterFactor", lidarParam.temporalFilterFactor);
	rclcpp::Parameter pTemporalFilterThreshold("T. temporalFilterFactorThreshold", lidarParam.temporalFilterThreshold);
	rclcpp::Parameter pEdgeFilterThreshold("U. edgeFilterThreshold", lidarParam.edgeFilterThreshold);
	//rclcpp::Parameter pTemporalEdgeThresholdLow("W temporalEdgeThresholdLow", lidarParam.temporalEdgeThresholdLow);
	//rclcpp::Parameter pTemporalEdgeThresholdHigh("X temporalEdgeThresholdHigh", lidarParam.temporalEdgeThresholdHigh);
	rclcpp::Parameter pInterferenceDetectionLimit("V. interferenceDetectionLimit", lidarParam.interferenceDetectionLimit);
	rclcpp::Parameter pUseLastValue("V. useLastValue", lidarParam.useLastValue);

	rclcpp::Parameter pCvShow("A. cvShow", lidarParam.cvShow);
	rclcpp::Parameter pDualBeam("W. dualBeam", lidarParam.dualBeam);
	rclcpp::Parameter pGrayLED("X. grayscale LED", lidarParam.grayscaleIlluminationMode);
	rclcpp::Parameter pPCEdgeFilter("Y. PointColud EDGE", lidarParam.pointCloudEdgeFilter);
	rclcpp::Parameter pMaxDistance("Z. MaxDistance", lidarParam.maxDistance);

	this->declare_parameter<string>("0. IP Addr", lidarParam.ipAddr);
//	this->declare_parameter<string>("1. Net Mask", lidarParam.netMask);
//	this->declare_parameter<string>("2. GW Addr", lidarParam.gwAddr);

	this->declare_parameter<int>("B. lensType", lidarParam.lensType);
	this->declare_parameter<int>("C. imageType", lidarParam.imageType);
	this->declare_parameter<int>("D. hdr_mode", lidarParam.hdr_mode);
	this->declare_parameter<int>("E. int0", lidarParam.int0);
	this->declare_parameter<int>("F. int1", lidarParam.int1);
	this->declare_parameter<int>("G. int2", lidarParam.int2);
	this->declare_parameter<int>("H. intGr",lidarParam.intGr);
	this->declare_parameter<int>("I. minAmplitude", lidarParam.minAmplitude);
	this->declare_parameter<int>("J. modIndex", lidarParam.frequencyModulation);
	this->declare_parameter<int>("K. channel", lidarParam.channel);
	this->declare_parameter<int>("L. roi_leftX", lidarParam.roi_leftX);
	this->declare_parameter<int>("M. roi_topY", lidarParam.roi_topY);
	this->declare_parameter<int>("N. roi_rightX", lidarParam.roi_rightX);
	this->declare_parameter<int>("O. roi_bottomY", lidarParam.roi_bottomY);
	this->declare_parameter<double>("P. transformAngle", lidarParam.transformAngle);
	this->declare_parameter<int>("Q. cutPixels", lidarParam.transformAngle);
	this->declare_parameter<bool>("R. medianFilter", lidarParam.medianFilter);
	this->declare_parameter<bool>("S. averageFilter", lidarParam.averageFilter);
	this->declare_parameter<double>("T. temporalFilterFactor", lidarParam.temporalFilterFactor);
	this->declare_parameter<int>("T. temporalFilterFactorThreshold", lidarParam.temporalFilterThreshold);
	this->declare_parameter<int>("U. edgeFilterThreshold", lidarParam.edgeFilterThreshold);
	//this->declare_parameter<int>("W temporalEdgeThresholdLow", lidarParam.temporalEdgeThresholdLow);
	//this->declare_parameter<int>("X temporalEdgeThresholdHigh", lidarParam.temporalEdgeThresholdHigh);
	this->declare_parameter<int>("V. interferenceDetectionLimit", lidarParam.interferenceDetectionLimit);
	this->declare_parameter<bool>("V. useLastValue", lidarParam.useLastValue);

	this->declare_parameter<bool>("A. cvShow", lidarParam.cvShow);
	this->declare_parameter<int>("W. dualBeam", lidarParam.dualBeam);
	this->declare_parameter<bool>("X. grayscale LED", lidarParam.grayscaleIlluminationMode);
	this->declare_parameter<bool>("Y. PointColud EDGE", lidarParam.pointCloudEdgeFilter);
	this->declare_parameter<int>("Z. MaxDistance", lidarParam.maxDistance);

	this->set_parameter(pIPAddr);
//	this->set_parameter(pNetMask);
//	this->set_parameter(pGWAddr);

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
	this->set_parameter(pCutpixels);
	this->set_parameter(pMedianFilter);
	this->set_parameter(pAverageFilter);
	this->set_parameter(pTemporalFilterFactor);
	this->set_parameter(pTemporalFilterThreshold);
	this->set_parameter(pEdgeFilterThreshold);
	//this->set_parameter(pTemporalEdgeThresholdLow);
	//this->set_parameter(pTemporalEdgeThresholdHigh);
	this->set_parameter(pInterferenceDetectionLimit);
	this->set_parameter(pUseLastValue);

	this->set_parameter(pCvShow);
	this->set_parameter(pDualBeam);
	this->set_parameter(pGrayLED);
	this->set_parameter(pPCEdgeFilter);
	this->set_parameter(pMaxDistance);
	
	

	//std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cameraSub");
	//cameraInfoService = node->create_service<sensor_msgs::srv::SetCameraInfo>("cameraSub", &setCameraInfo);
	connectionCameraInfo = interface.subscribeCameraInfo([&](std::shared_ptr<CameraInfo> ci) -> void { updateCameraInfo(ci); });
	connectionFrames = interface.subscribeFrame([&](Frame* f) -> void {  updateFrame(f); });
	cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lidarParam.lensCenterOffsetX, lidarParam.lensCenterOffsetY, lidarParam.lensType); //0.02 mm - sensor pixel size

}


void roboscanPublisher::setParameters()
{

	printf("setParameters\n");
	interface.setIpAddr( lidarParam.ipAddr, lidarParam.netMask, lidarParam.gwAddr);
	interface.stopStream();

	interface.setMinAmplitude(lidarParam.minAmplitude);
	interface.setIntegrationTime(lidarParam.int0, lidarParam.int1, lidarParam.int2, lidarParam.intGr, lidarParam.grayscaleIlluminationMode);
	    
	interface.setHDRMode((uint8_t)lidarParam.hdr_mode);
	interface.setFilter(lidarParam.medianFilter, lidarParam.averageFilter, static_cast<uint16_t>(lidarParam.temporalFilterFactor * 1000), lidarParam.temporalFilterThreshold, lidarParam.edgeFilterThreshold,
	                    lidarParam.temporalEdgeThresholdLow, lidarParam.temporalEdgeThresholdHigh, lidarParam.interferenceDetectionLimit, lidarParam.useLastValue);

	interface.setAdcOverflowSaturation(lidarParam.bAdcOverflow, lidarParam.bSaturation);
	interface.setGrayscaleIlluminationMode(lidarParam.grayscaleIlluminationMode);
	interface.setDualBeam(lidarParam.dualBeam, true);

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
	lidarParam.pointCloudEdgeFilter = false;
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
	case Frame::DISTANCE_AMPLITUDE:
	  interface.streamDistanceAmplitude();
	  printf("streaming distance-amplitude\n");
	  break;
	case Frame::DISTANCE_GRAYSCALE:
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

int roboscanPublisher::Convert_To_RGB24( float fValue, RGB888Pixel &nRGBData, float fMinValue, float fMaxValue)
{
	if(fValue == ADC_OVERFLOW)
	{
		nRGBData.r = 169;//R
		nRGBData.g = 14;//G
		nRGBData.b = 255;//B
	}
	else if(fValue == SATURATION)
	{
		nRGBData.r = 255;//R
		nRGBData.g = 0;//G
		nRGBData.b = 128;//B
	}
	else if(fValue == INTERFERENCE || fValue == LOW_AMPLITUDE || fValue == EDGE_FILTERED )
	{
		nRGBData.r = 0;//R
		nRGBData.g = 0;//G
		nRGBData.b = 0;//B
	}
	else if(fValue == 0) //Invalide Pixel
	{
		nRGBData.r = 0;//R
		nRGBData.g = 0;//G
		nRGBData.b = 0;//B
	}
	else if(fValue < fMinValue)
	{
		nRGBData.r = 0;//R
		nRGBData.g = 0;//G
		nRGBData.b = 0;//B
	}
	else if(fValue > fMaxValue)
	{
		nRGBData.r = 0;//R
		nRGBData.g = 0;//G
		nRGBData.b = 0;//B
	}
	else
	{
		float fColorWeight;
		fColorWeight = (fValue-fMinValue) / (fMaxValue-fMinValue);

		if( (fColorWeight <= 1.0f) && (fColorWeight > 0.8f) )
		{
			nRGBData.r = (unsigned char)(255 * ((fColorWeight - 0.8f) / 0.2f));//값에 따라 증가
			nRGBData.g = 0;
			nRGBData.b = 255;
		} 
		else if( (fColorWeight <= 0.8f) && (fColorWeight > 0.6f) )
		{
			nRGBData.r = 0;
			nRGBData.g = (unsigned char)(255 * (1.0f - (fColorWeight - 0.6f) / 0.2f));//값에 따라 감소
			nRGBData.b = 255;
		}
		else if( (fColorWeight <= 0.6f) && (fColorWeight > 0.4f) )
		{
			nRGBData.r = 0;
			nRGBData.g = 255;
			nRGBData.b = (unsigned char)(255 * ((fColorWeight - 0.4f) / 0.2f));//값에 따라 증가
		}
		else if( (fColorWeight <= 0.4f) && (fColorWeight > 0.2f) )
		{
			nRGBData.r = (unsigned char)(255 * (1.0f - (fColorWeight - 0.2f) / 0.2f));//값에 따라 감소
			nRGBData.g = 255;
			nRGBData.b = 0;
		}
		else if( (fColorWeight <= 0.2f) && (fColorWeight >= 0.0f) )
		{
			nRGBData.r = 255;
			nRGBData.g = (unsigned char)(255 * ((fColorWeight - 0.0f) / 0.2f));//값에 따라 증가
			nRGBData.b = 0;
		}
		else
		{
			nRGBData.r = 0;
			nRGBData.g = 0;
			nRGBData.b = 0;
		}
	}

	return true;
}

void roboscanPublisher::setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{
	if( value == LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == SATURATION)
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
	else if(value == BAD_PIXEL)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<Vec3b>(y, x) = colorVector.at(0);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > end_range)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = value * (NUM_COLORS / end_range);
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = 0;
		}
		else if( index >= (int)colorVector.size() ){
			index = colorVector.size()-1;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

}


void roboscanPublisher::setGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
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

int roboscanPublisher::setDistanceColor(cv::Mat &imageLidar, int x, int y, int value )
{	
	if( value == LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == SATURATION)
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
	else if(value == INTERFERENCE)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == EDGE_FILTERED)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == BAD_PIXEL)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<Vec3b>(y, x) = colorVector.at(colorVector.size()-1);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > lidarParam.maxDistance)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = colorVector.size() - (value*(NUM_COLORS/lidarParam.maxDistance));
		if( index < 0 ){
			printf("error1 index = %d\n", index);
			index = colorVector.size()-1;
		}
		else if( index > (int)colorVector.size() ){
			printf("error2 index = %d\n", index);
			index = 0;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

	return value;
}

cv::Mat roboscanPublisher::addDistanceInfo(cv::Mat distMat, Frame *frame)
{
	int xpos = mouseXpos;
	int ypos = mouseYpos;
	
	if( (ypos > 0 && ypos < frame->height)){
		// mouseXpos, mouseYpos
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		cv::line(distMat, cv::Point(xpos-10, ypos), cv::Point(xpos+10, ypos), cv::Scalar(255, 255, 0), 2);
		cv::line(distMat, cv::Point(xpos, ypos-10), cv::Point(xpos, ypos+10), cv::Scalar(255, 255, 0), 2);

		if( xpos >= frame->width*2 ){
			xpos -= frame->width*2;
		}
		else if( xpos >= frame->width ){
			xpos -= frame->width;
		}

		std::string dist_caption;

		int real_xpos = xpos;
		int real_dist = frame->dist2BData[ypos*frame->width + real_xpos];
		if( real_dist > PIXEL_VALID_DATA ){

			if( real_dist == ADC_OVERFLOW )
				dist_caption = cv::format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( real_dist == SATURATION )
				dist_caption = cv::format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( real_dist == BAD_PIXEL )
				dist_caption = cv::format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( real_dist == INTERFERENCE )
				dist_caption = cv::format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( real_dist == EDGE_FILTERED )
				dist_caption = cv::format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist_caption = cv::format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			if( frame->dataType == Frame::DISTANCE_AMPLITUDE ) dist_caption = cv::format("X:%d,Y:%d %dmm/%dlsb", xpos, ypos, frame->dist2BData[ypos*frame->width + real_xpos], frame->ampl2BData[ypos*frame->width + real_xpos]);
			else if( frame->dataType == Frame::DISTANCE_GRAYSCALE ) dist_caption = cv::format("X:%d,Y:%d %dmm/%dlsb", xpos, ypos, frame->dist2BData[ypos*frame->width + real_xpos], frame->gray2BData[ypos*frame->width + real_xpos]);
			else if( frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ) dist_caption = cv::format("X:%d,Y:%d %dmm/%dlsb/%dlsb", xpos, ypos, frame->dist2BData[ypos*frame->width + real_xpos], frame->ampl2BData[ypos*frame->width + real_xpos], frame->gray2BData[ypos*frame->width + real_xpos]);
			else if( frame->dataType == Frame::GRAYSCALE )	dist_caption = cv::format("X:%d,Y:%d %dlsb", xpos, ypos, frame->gray2BData[ypos*frame->width + real_xpos]);
			else	dist_caption = cv::format("X:%d,Y:%d %dmm", xpos, ypos, frame->dist2BData[ypos*frame->width + real_xpos]);
		}

		putText(infoImage, dist_caption.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));
		cv::vconcat(distMat, infoImage, distMat);
	}
	else{
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));
		cv::vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}

cv::Mat roboscanPublisher::addDCSInfo(cv::Mat distMat, Frame *frame)
{
	int xpos = mouseXpos;
	int ypos = mouseYpos;
	
	if( (ypos > 0 && ypos < frame->height*2)){
		// mouseXpos, mouseYpos
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		cv::line(distMat, cv::Point(xpos-10, ypos), cv::Point(xpos+10, ypos), cv::Scalar(255, 255, 0), 2);
		cv::line(distMat, cv::Point(xpos, ypos-10), cv::Point(xpos, ypos+10), cv::Scalar(255, 255, 0), 2);

		if( xpos >= frame->width ){
			xpos -= frame->width;
		}

		if( ypos >= frame->height ){
			ypos -= frame->height;
		}

		std::string dist_caption;

		int real_xpos = xpos;
		int real_dist = frame->dcs2BData[ypos*frame->width + real_xpos];
		if( real_dist > PIXEL_VALID_DATA ){

			if( real_dist == ADC_OVERFLOW )
				dist_caption = cv::format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( real_dist == SATURATION )
				dist_caption = cv::format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( real_dist == BAD_PIXEL )
				dist_caption = cv::format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( real_dist == INTERFERENCE )
				dist_caption = cv::format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( real_dist == EDGE_FILTERED )
				dist_caption = cv::format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist_caption = cv::format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			dist_caption = cv::format("X:%d,Y:%d %d/%d/%d/%d", xpos, ypos
										, frame->dcs2BData[ypos*frame->width + real_xpos]
										, frame->dcs2BData[(frame->width*frame->height) + ypos*frame->width + real_xpos]
										, frame->dcs2BData[(frame->width*frame->height * 2) + ypos*frame->width + real_xpos]
										, frame->dcs2BData[(frame->width*frame->height * 3) + ypos*frame->width + real_xpos]);
		}

		putText(infoImage, dist_caption.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));
		cv::vconcat(distMat, infoImage, distMat);
	}
	else{
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));
		cv::vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}


bool roboscanPublisher::edgeDetection(Triple const& v0, Triple const& v1, Triple const& v2, double threshold, double distanceLimit)
{
    if( (abs(v0.z - v1.z) < distanceLimit) && (abs(v1.z - v2.z) < distanceLimit) && (abs(v2.z - v0.z) < distanceLimit) )
        return false;

    Triple u, v, n, nc;
    double dotproduct;
    u = v1 - v0;
    v = v2 - v0;

    n = normalizedcross(u,v);
    nc = (v0 + v1 + v2)/3;
    nc.normalize();
    dotproduct = dotProduct(n, nc);

    if(abs(dotproduct) < threshold)
        return true;
    else
        return false;
}

void roboscanPublisher::publishFrame(Frame *frame)
{
	int x, y, k, l, pc;
	static rclcpp::Clock s_rclcpp_clock;
	auto data_stamp = s_rclcpp_clock.now();

//	std::chrono::system_clock::time_point update_start = std::chrono::system_clock::now();

	cv::Mat dcs1(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// distance
	cv::Mat dcs2(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// amplitude
	cv::Mat dcs3(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// garycale
	cv::Mat dcs4(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));

	if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::DISTANCE_AMPLITUDE || frame->dataType == Frame::DISTANCE_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
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

	if(frame->dataType == Frame::DISTANCE_AMPLITUDE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){
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

	if(frame->dataType == Frame::GRAYSCALE || frame->dataType == Frame::DISTANCE_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){
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
			for(x=frame->width-1; x >= 0; x--, k++, l+=2){
				gray = (frame->grayData[l+1] << 8)	+ frame->grayData[l];
				setGrayscaleColor(dcs3, x, y, gray, 2048); // 2048, 255
			}
		}
	}
	else if(frame->dataType == Frame::DCS){
		uint16_t dcsData1; 
		uint16_t dcsData2; 
		uint16_t dcsData3; 
		uint16_t dcsData4;

		int frameSize = frame->width * frame->height * 2;
		for(k=0, l=0, y=0; y< frame->height; y++){
			for(x=0; x < frame->width; x++, k++, l+=2){
				dcsData1 = (frame->dcsData[l+1] << 8)	+ frame->dcsData[l];
				dcsData2 = (frame->dcsData[l+1+frameSize] << 8)	+ frame->dcsData[l+frameSize];
				dcsData3 = (frame->dcsData[l+1+frameSize*2] << 8)	+ frame->dcsData[l+frameSize*2];
				dcsData4 = (frame->dcsData[l+1+frameSize*3] << 8)	+ frame->dcsData[l+frameSize*3];
				setGrayscaleColor(dcs1, x, y, dcsData1, 4096); // 2048, 255
				setGrayscaleColor(dcs2, x, y, dcsData2, 4096); // 2048, 255
				setGrayscaleColor(dcs3, x, y, dcsData3, 4096); // 2048, 255
				setGrayscaleColor(dcs4, x, y, dcsData4, 4096); // 2048, 255
			}
		}
	}
	else
	{
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
		uint16_t grayscale = 0;
		double px, py, pz;

		RGB888Pixel pTex1;


		for(k=0, l=0, y=0; y< frame->height; y++)
		{
			for(x=0, pc = frame->width-1; x < frame->width; x++, k++, l+=2, pc--)
			{
				pcl::PointXYZI &p = cloud->points[k];

				distance = (frame->distData[l+1] << 8) + frame->distData[l];
				amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];
				grayscale = (frame->grayData[l+1] << 8)  + frame->grayData[l];

				if( !(y > -x + lidarParam.cutPixels
						&& y > x - (319-lidarParam.cutPixels)
						&& y < x + (239-lidarParam.cutPixels)
						&& y < -x + lidarParam.cutPixels + (239-lidarParam.cutPixels) + (319-lidarParam.cutPixels)))
				{
					distance = LOW_AMPLITUDE;
					amplitude = LOW_AMPLITUDE;
				}

				Convert_To_RGB24((double)distance, pTex1, 0.0f, lidarParam.maxDistance);
				dcs1.at<Vec3b>(y, x)[0] = pTex1.b;
				dcs1.at<Vec3b>(y, x)[1] = pTex1.g;
				dcs1.at<Vec3b>(y, x)[2] = pTex1.r;

				if(frame->dataType == Frame::DISTANCE_AMPLITUDE){
					setAmplitudeColor(dcs2, x, y, amplitude, 2897);
				}
				else if( frame->dataType == Frame::DISTANCE_GRAYSCALE ){
					setGrayscaleColor(dcs3, x, y, grayscale, 2048);	// 2048, 255
				}
				else if( frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
					setAmplitudeColor(dcs2, x, y, amplitude, 2897);	// 2048, 255
					setGrayscaleColor(dcs3, x, y, grayscale, 2048);	// 2048, 255
				}

#if 0
				if(x == 160 && y == 120)
				{
					RCLCPP_INFO(this->get_logger(), "distance : %d", distance);
				}
#endif
				if (distance > 0 && distance < lidarParam.maxDistance)
				{
					if(lidarParam.cartesian)
					{
						if(lidarParam.pointCloudEdgeFilter
							&& pc < frame->width-1 
							&& y < frame->height-1)
						{
							
							uint16_t d1 = distance;
							uint16_t d2 = (frame->distData[l+1+2] << 8) + frame->distData[l+2]; //*(uint16_t *)&resultBuffer[(index+1) * bufferSize];
							uint16_t d3 = (frame->distData[l+1+frame->width*2] << 8) + frame->distData[l+frame->width*2]; //*(uint16_t *)&resultBuffer[(index+width) * bufferSize];

							if( d1>0 && d1<lidarParam.maxDistance && d2>0 && d2<lidarParam.maxDistance && d3>0 && d3<lidarParam.maxDistance ){
								Triple v0, v1, v2;
								cartesianTransform.transformPixel(pc, y, d1, v0.x, v0.y, v0.z, lidarParam.transformAngle);
								cartesianTransform.transformPixel(pc+1, y, d2, v1.x, v1.y, v1.z, lidarParam.transformAngle);
								cartesianTransform.transformPixel(pc, y+1, d3, v2.x, v2.y, v2.z, lidarParam.transformAngle);

								if(edgeDetection(v0, v1, v2, 0.2, 30)){
									p.x = std::numeric_limits<float>::quiet_NaN();
									p.y = std::numeric_limits<float>::quiet_NaN();
									p.z = std::numeric_limits<float>::quiet_NaN();
									p.intensity = std::numeric_limits<float>::quiet_NaN();
									continue;
								}
							}
						}
					
						cartesianTransform.transformPixel(pc, y, distance, px, py, pz, lidarParam.transformAngle);
						p.x = static_cast<float>(pz / 1000.0); //mm -> m
						p.y = static_cast<float>(px / 1000.0);
						p.z = static_cast<float>(-py / 1000.0);

						if(frame->dataType == Frame::DISTANCE_AMPLITUDE
							|| frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ) p.intensity = static_cast<float>(amplitude);
						else p.intensity = static_cast<float>(pz / 1000.0);
					}
					else
					{
						p.x = distance / 1000.0;
						p.y = -(160-x) / 100.0;
						p.z = (120-y) / 100.0;
						if(frame->dataType == Frame::DISTANCE_AMPLITUDE
							|| frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ) p.intensity = static_cast<float>(amplitude);
						else p.intensity = static_cast<float>(distance / 1000.0);
					} 
				}
				else
				{
					p.x = std::numeric_limits<float>::quiet_NaN();
					p.y = std::numeric_limits<float>::quiet_NaN();
					p.z = std::numeric_limits<float>::quiet_NaN();
					p.intensity = std::numeric_limits<float>::quiet_NaN();
				}		  

			}
		}


		sensor_msgs::msg::PointCloud2 msg;
		pcl::toROSMsg(*cloud, msg);
		msg.header.stamp = data_stamp;
		msg.header.frame_id = "roboscan_frame";
		pointcloudPub->publish(msg);  
	}
	
	getMouseEvent(mouseXpos, mouseYpos);
		
	if( frame->dataType == Frame::DISTANCE ){
		dcs1 = addDistanceInfo(dcs1, frame);
	}
	else if( frame->dataType == Frame::DISTANCE_AMPLITUDE ){
		cv::hconcat(dcs1, dcs2, dcs1);
		dcs1 = addDistanceInfo(dcs1, frame);
	}
	else if( frame->dataType == Frame::DISTANCE_GRAYSCALE ){
		cv::hconcat(dcs1, dcs3, dcs1);
		dcs1 = addDistanceInfo(dcs1, frame);
	}
	else if( frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
		cv::hconcat(dcs1, dcs2, dcs1);
		cv::hconcat(dcs1, dcs3, dcs1);
		dcs1 = addDistanceInfo(dcs1, frame);
	}
	if( frame->dataType == Frame::GRAYSCALE ){
		dcs1 = addDistanceInfo(dcs3, frame);
	}
	else if(frame->dataType == Frame::DCS){
		cv::hconcat(dcs1, dcs2, dcs1);
		cv::hconcat(dcs3, dcs4, dcs3);
		cv::vconcat(dcs1, dcs3, dcs1);
		dcs1 = addDCSInfo(dcs1, frame);
	}

	if(lidarParam.cvShow == true)
	{
		imshow(winName, dcs1);
	}

//	std::chrono::system_clock::time_point update_end = std::chrono::system_clock::now();
//	std::chrono::milliseconds timeCnt = std::chrono::duration_cast<std::chrono::milliseconds>(update_end - update_start);
//	printf("update-color time = %ld ms\n", timeCnt.count());
#ifdef image_transfer_function
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv_ptr->header.stamp = data_stamp;
	cv_ptr->header.frame_id = "roboscan_frame";
	cv_ptr->image = dcs1;
	cv_ptr->encoding = "bgr8";

	imagePublisher.publish(cv_ptr->toImageMsg());
#endif
	delete frame;
}

void roboscanPublisher::updateFrame(Frame * frame)
{
	backupFrame.push_back(frame);
}


void roboscanPublisher::getMouseEvent( int &mouse_xpos, int &mouse_ypos )
{
	mouse_xpos = x_start;
	mouse_ypos = y_start;
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
