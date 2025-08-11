#include <cstdio>
#include "cartesian_transform.hpp"
#include "interface.hpp"
#include <chrono>
#include <functional>
#include <future>
#include <filesystem>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>
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
#include "roboscan_nsl3130/msg/custom_msg.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include "yaml-cpp/yaml.h"
#include "roboscan_publish_node.hpp"

using namespace nanosys;
using namespace std::chrono_literals;
using namespace cv;
using namespace std;

#define WIN_NAME "NSL-3130AA IMAGE"
#define  MAX_LEVELS  9
#define NUM_COLORS     		30000

#define LEFTX_MAX	124	
#define RIGHTX_MIN	131
#define RIGHTX_MAX	319	
#define X_INTERVAL	4

#define LEFTY_MAX	116	
#define RIGHTY_MIN	123
#define RIGHTY_MAX	239	
#define Y_INTERVAL	2

std::atomic<int> x_start = -1, y_start = -1;

std::future<void> result;

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
	lidarParam.publishName = "roboscan_frame";
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    imgDistancePub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
    imgAmplPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanAmpl", qos_profile); 
    imgGrayPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanGray", qos_profile); 
    imgDCSPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDCS", qos_profile); 
    pointcloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 
	area0Pub = this->create_publisher<visualization_msgs::msg::Marker>("area0", qos_profile);
	area1Pub = this->create_publisher<visualization_msgs::msg::Marker>("area1", qos_profile);
	area2Pub = this->create_publisher<visualization_msgs::msg::Marker>("area2", qos_profile);
	area3Pub = this->create_publisher<visualization_msgs::msg::Marker>("area3", qos_profile);
	areaMsgpub = this->create_publisher<roboscan_nsl3130::msg::CustomMsg>("areaMsg", qos_profile);

	

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
			int x1_tmp = param.as_int();

			if(x1_tmp % X_INTERVAL ) x1_tmp+=X_INTERVAL-(x1_tmp % X_INTERVAL );
			if(x1_tmp > LEFTX_MAX ) x1_tmp = LEFTX_MAX;

			lidarParam.roi_leftX = x1_tmp;

		}
		else if (param.get_name() == "N. roi_rightX")
		{
			int x2_tmp = param.as_int();
			
			if((x2_tmp-RIGHTX_MIN) % X_INTERVAL)	x2_tmp-=((x2_tmp-RIGHTX_MIN) % X_INTERVAL);
			if(x2_tmp < RIGHTX_MIN ) x2_tmp = RIGHTX_MIN;
			if(x2_tmp > RIGHTX_MAX ) x2_tmp = RIGHTX_MAX;
			
			lidarParam.roi_rightX = x2_tmp;
		}
		else if (param.get_name() == "M. roi_topY")
		{
			int y1_tmp = param.as_int();
			
			if(y1_tmp % Y_INTERVAL )	y1_tmp++;
			if(y1_tmp > LEFTY_MAX ) y1_tmp = LEFTY_MAX;
			
			lidarParam.roi_topY = y1_tmp;
			
			int y2_tmp = RIGHTY_MAX - y1_tmp;
			lidarParam.roi_bottomY = y2_tmp;
		}
		else if (param.get_name() == "O. roi_bottomY")
		{
			int y2_tmp = param.as_int();
			
			if(y2_tmp % Y_INTERVAL == 0 )	y2_tmp++;
			if(y2_tmp < RIGHTY_MIN ) y2_tmp = RIGHTY_MIN;
			if(y2_tmp > RIGHTY_MAX ) y2_tmp = RIGHTY_MAX;
			
			lidarParam.roi_bottomY = y2_tmp;
			
			int y1_tmp = RIGHTY_MAX - y2_tmp;
			lidarParam.roi_topY = y1_tmp;
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
		else if (param.get_name() == "1. Frame Id")
		{
			lidarParam.publishName = param.as_string();
		}

		else if (param.get_name() == "2-1. image Format Mono16")
		{
			lidarParam.FormatMono16 = param.as_bool();
			if (param.as_bool())
			{
				lidarParam.selected_format = "2-1. image Format Mono16";
			}

		}
		else if (param.get_name() == "2-2. image Format Mono8")
		{
			lidarParam.FormatMono8 = param.as_bool();
			if (param.as_bool())
			{
				lidarParam.selected_format = "2-2. image Format Mono8";
			}

		}
		else if (param.get_name() == "2-3. image Format 16UC1")
		{
			lidarParam.Format16UC1 = param.as_bool();
			if (param.as_bool())
			{
				lidarParam.selected_format = "2-3. image Format 16UC1";
			}
		}
		else if (param.get_name() == "2-4. image Format RGB8")
		{
			lidarParam.FormatRGB8 = param.as_bool();
			if (param.as_bool())
			{
				lidarParam.selected_format = "2-4. image Format RGB8";
			}
		}
		// Area 0
		else if (param.get_name() == "area0. Enabled")
		{
			lidarParam.areaBtn[0] = param.as_bool();
		}
		else if (param.get_name() == "area0. minimum_detection_point")
		{
			lidarParam.minPoint[0] = param.as_int();
		}
		else if (param.get_name() == "area0. length_scale")
		{
			lidarParam.areaScaleX[0] = param.as_double();
		}
		else if (param.get_name() == "area0. width_scale")
		{
			lidarParam.areaScaleY[0] = param.as_double();
		}
		else if (param.get_name() == "area0. height_scale")
		{
			lidarParam.areaScaleZ[0] = param.as_double();
		}
		else if (param.get_name() == "area0. length_position")
		{
			lidarParam.areaPosX[0] = param.as_double();
		}
		else if (param.get_name() == "area0. width_position")
		{
			lidarParam.areaPosY[0] = param.as_double();
		}
		else if (param.get_name() == "area0. height_position")
		{
			lidarParam.areaPosZ[0] = param.as_double();
		}

		// Area 1
		else if (param.get_name() == "area1. Enabled")
		{
			lidarParam.areaBtn[1] = param.as_bool();
		}
		else if (param.get_name() == "area1. minimum_detection_point")
		{
			lidarParam.minPoint[1] = param.as_int();
		}
		else if (param.get_name() == "area1. length_scale")
		{
			lidarParam.areaScaleX[1] = param.as_double();
		}
		else if (param.get_name() == "area1. width_scale")
		{
			lidarParam.areaScaleY[1] = param.as_double();
		}
		else if (param.get_name() == "area1. height_scale")
		{
			lidarParam.areaScaleZ[1] = param.as_double();
		}
		else if (param.get_name() == "area1. length_position")
		{
			lidarParam.areaPosX[1] = param.as_double();
		}
		else if (param.get_name() == "area1. width_position")
		{
			lidarParam.areaPosY[1] = param.as_double();
		}
		else if (param.get_name() == "area1. height_position")
		{
			lidarParam.areaPosZ[1] = param.as_double();
		}

		// Area 2
		else if (param.get_name() == "area2. Enabled")
		{
			lidarParam.areaBtn[2] = param.as_bool();
		}
		else if (param.get_name() == "area2. minimum_detection_point")
		{
			lidarParam.minPoint[2] = param.as_int();
		}
		else if (param.get_name() == "area2. length_scale")
		{
			lidarParam.areaScaleX[2] = param.as_double();
		}
		else if (param.get_name() == "area2. width_scale")
		{
			lidarParam.areaScaleY[2] = param.as_double();
		}
		else if (param.get_name() == "area2. height_scale")
		{
			lidarParam.areaScaleZ[2] = param.as_double();
		}
		else if (param.get_name() == "area2. length_position")
		{
			lidarParam.areaPosX[2] = param.as_double();
		}
		else if (param.get_name() == "area2. width_position")
		{
			lidarParam.areaPosY[2] = param.as_double();
		}
		else if (param.get_name() == "area2. height_position")
		{
			lidarParam.areaPosZ[2] = param.as_double();
		}

		// Area 3
		else if (param.get_name() == "area3. Enabled")
		{
			lidarParam.areaBtn[3] = param.as_bool();
		}
		else if (param.get_name() == "area3. minimum_detection_point")
		{
			lidarParam.minPoint[3] = param.as_int();
		}
		else if (param.get_name() == "area3. length_scale")
		{
			lidarParam.areaScaleX[3] = param.as_double();
		}
		else if (param.get_name() == "area3. width_scale")
		{
			lidarParam.areaScaleY[3] = param.as_double();
		}
		else if (param.get_name() == "area3. height_scale")
		{
			lidarParam.areaScaleZ[3] = param.as_double();
		}
		else if (param.get_name() == "area3. length_position")
		{
			lidarParam.areaPosX[3] = param.as_double();
		}
		else if (param.get_name() == "area3. width_position")
		{
			lidarParam.areaPosY[3] = param.as_double();
		}
		else if (param.get_name() == "area3. height_position")
		{
			lidarParam.areaPosZ[3] = param.as_double();
		}

	}

	lidarParam.FormatChange = true;
	reconfigure = true;
	return result;
}

// Save & Load Test
void roboscanPublisher::paramDump(const std::string & filename)
{
	lidarParam.paramSave = true;

    std::string package_path = ament_index_cpp::get_package_share_directory("roboscan_nsl3130");

    std::string full_path = package_path + "/" + filename;

    std::string command = "ros2 param dump /roboscan_publish_node > " + full_path;

    int result = std::system(command.c_str());

    if (result == 0) {
        try {
            std::ifstream in(full_path);
            std::ofstream out(full_path + ".tmp");

            std::string line;
            bool skip_block = false;

            while (std::getline(in, line)) {
                if (line.find("A:") != std::string::npos) {
                    continue;
                }
                if (line.find("cvShow") != std::string::npos) {
                    continue;
                }

                if (line.find("qos_overrides:") != std::string::npos) {
                    skip_block = true;
                    continue;
                }

                if (skip_block && !line.empty() && line[0] != ' ' && line.find(":") != std::string::npos) {
                    skip_block = false;
                }

                if (!skip_block) {
                    out << line << '\n';
                }
            }

            in.close();
            out.close();

            std::filesystem::rename(full_path + ".tmp", package_path + "/rqt.yaml");
            std::filesystem::remove(full_path);

            std::cout << "save successful!!!!!!." << std::endl;

        } catch (const std::filesystem::filesystem_error& e) {
            std::filesystem::remove(full_path);
            std::cerr << "save failed!! " << e.what() << std::endl;
        }
    } else {
        std::cerr << "Command failed!!" << std::endl;
        std::filesystem::remove(full_path);
    }
	lidarParam.paramSave = false;
}


void roboscanPublisher::paramLoad() // file exist check
{
	std::string yaml_file = ament_index_cpp::get_package_share_directory("roboscan_nsl3130") + "/rqt.yaml";

	std::string command = "ros2 param load /roboscan_publish_node " + yaml_file + " > /dev/null";

	if (std::filesystem::exists(yaml_file)) {
		int result = std::system(command.c_str());

		if (result == 0) {
			std::cout << "Load successful!!!!!" << std::endl;
		} else {
			std::cerr << "Load failed!!" << std::endl;
		}
	} else {
		std::cerr << "not exist yaml file: " << yaml_file << std::endl;
	}
}


void roboscanPublisher::setReconfigure()
{ 
	printf("setReconfigure\n");

	if( lidarParam.changedIpInfo ){
		interface.stopStream();    
		lidarParam.changedIpInfo = false;
		interface.setIpAddr( lidarParam.ipAddr, lidarParam.netMask, lidarParam.gwAddr);
		paramDump("rqt_temp.yaml");
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

	area0Box.header.frame_id = lidarParam.publishName;
	area1Box.header.frame_id = lidarParam.publishName;
	area2Box.header.frame_id = lidarParam.publishName;
	area3Box.header.frame_id = lidarParam.publishName;


	// Area 0
	if(lidarParam.areaBtn[0])
	{
		area0Box.action = visualization_msgs::msg::Marker::ADD;
		area0Box.scale.x = lidarParam.areaScaleX[0];
		area0Box.scale.y = lidarParam.areaScaleY[0];
		area0Box.scale.z = lidarParam.areaScaleZ[0];
	
		area0Box.pose.position.x = lidarParam.areaScaleX[0] / 2.0 + lidarParam.areaPosX[0];
		area0Box.pose.position.y = lidarParam.areaPosY[0];
		area0Box.pose.position.z = lidarParam.areaPosZ[0];

		lidarParam.x_min[0] = lidarParam.areaPosX[0] , lidarParam.x_max[0] = lidarParam.areaPosX[0] + lidarParam.areaScaleX[0] ;
		lidarParam.y_min[0] = lidarParam.areaPosY[0] - lidarParam.areaScaleY[0] / 2.0, lidarParam.y_max[0] = lidarParam.areaPosY[0] + lidarParam.areaScaleY[0] / 2.0;
		lidarParam.z_min[0] = lidarParam.areaPosZ[0] - lidarParam.areaScaleZ[0] / 2.0, lidarParam.z_max[0] = lidarParam.areaPosZ[0] + lidarParam.areaScaleZ[0] / 2.0;
	}
	
	else
	{
		area0Box.action = visualization_msgs::msg::Marker::DELETE;
		lidarParam.pointCount[0] = 0;
		lidarParam.pointDetect[0] = false;
	}


	// Area 1
	if(lidarParam.areaBtn[1])
	{
		area1Box.action = visualization_msgs::msg::Marker::ADD;
		area1Box.scale.x = lidarParam.areaScaleX[1];
		area1Box.scale.y = lidarParam.areaScaleY[1];
		area1Box.scale.z = lidarParam.areaScaleZ[1];

		area1Box.pose.position.x = lidarParam.areaScaleX[1] / 2.0 + lidarParam.areaPosX[1];
		area1Box.pose.position.y = lidarParam.areaPosY[1];
		area1Box.pose.position.z = lidarParam.areaPosZ[1];

		lidarParam.x_min[1] = lidarParam.areaPosX[1], lidarParam.x_max[1] = lidarParam.areaPosX[1] + lidarParam.areaScaleX[1];
		lidarParam.y_min[1] = lidarParam.areaPosY[1] - lidarParam.areaScaleY[1] / 2.0, lidarParam.y_max[1] = lidarParam.areaPosY[1] + lidarParam.areaScaleY[1] / 2.0;
		lidarParam.z_min[1] = lidarParam.areaPosZ[1] - lidarParam.areaScaleZ[1] / 2.0, lidarParam.z_max[1] = lidarParam.areaPosZ[1] + lidarParam.areaScaleZ[1] / 2.0;

		
	}
	else
	{
		area1Box.action = visualization_msgs::msg::Marker::DELETE;
		lidarParam.pointCount[1] = 0;
		lidarParam.pointDetect[1] = false;
	}

	// Area 2
	if(lidarParam.areaBtn[2])
	{
		area2Box.action = visualization_msgs::msg::Marker::ADD;
		area2Box.scale.x = lidarParam.areaScaleX[2];
		area2Box.scale.y = lidarParam.areaScaleY[2];
		area2Box.scale.z = lidarParam.areaScaleZ[2];

		area2Box.pose.position.x = lidarParam.areaScaleX[2] / 2.0 + lidarParam.areaPosX[2];
		area2Box.pose.position.y = lidarParam.areaPosY[2];
		area2Box.pose.position.z = lidarParam.areaPosZ[2];

		lidarParam.x_min[2] = lidarParam.areaPosX[2], lidarParam.x_max[2] = lidarParam.areaPosX[2] + lidarParam.areaScaleX[2];
		lidarParam.y_min[2] = lidarParam.areaPosY[2] - lidarParam.areaScaleY[2] / 2.0, lidarParam.y_max[2] = lidarParam.areaPosY[2] + lidarParam.areaScaleY[2] / 2.0;
		lidarParam.z_min[2] = lidarParam.areaPosZ[2] - lidarParam.areaScaleZ[2] / 2.0, lidarParam.z_max[2] = lidarParam.areaPosZ[2] + lidarParam.areaScaleZ[2] / 2.0;

	}
	else
	{
		area2Box.action = visualization_msgs::msg::Marker::DELETE;
		lidarParam.pointCount[2] = 0;
		lidarParam.pointDetect[2] = false;
	}

	// Area 3
	if(lidarParam.areaBtn[3])
	{
		area3Box.action = visualization_msgs::msg::Marker::ADD;
		area3Box.scale.x = lidarParam.areaScaleX[3];
		area3Box.scale.y = lidarParam.areaScaleY[3];
		area3Box.scale.z = lidarParam.areaScaleZ[3];

		area3Box.pose.position.x = lidarParam.areaScaleX[3] / 2.0 + lidarParam.areaPosX[3];
		area3Box.pose.position.y = lidarParam.areaPosY[3];
		area3Box.pose.position.z = lidarParam.areaPosZ[3];

		lidarParam.x_min[3] = lidarParam.areaPosX[3], lidarParam.x_max[3] = lidarParam.areaPosX[3] + lidarParam.areaScaleX[3];
		lidarParam.y_min[3] = lidarParam.areaPosY[3] - lidarParam.areaScaleY[3] / 2.0, lidarParam.y_max[3] = lidarParam.areaPosY[3] + lidarParam.areaScaleY[3] / 2.0;
		lidarParam.z_min[3] = lidarParam.areaPosZ[3] - lidarParam.areaScaleZ[3] / 2.0, lidarParam.z_max[3] = lidarParam.areaPosZ[3] + lidarParam.areaScaleZ[3] / 2.0;

	}
	else
	{
		area3Box.action = visualization_msgs::msg::Marker::DELETE;
		lidarParam.pointCount[0] = 0;
		lidarParam.pointDetect[0] = false;
	}

	//startStream = true;
	printf("setReconfigure OK!\n\n");
	waitKey(1);
	startStreaming();

	setWinName();


	if(!lidarParam.paramLoad && !lidarParam.paramSave)
	{
		result = std::async(std::launch::async, [&]() {
			paramDump("rqt_temp.yaml");
		  });
	}
	lidarParam.paramLoad = false;

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

	//defalut
	lidarParam.lensType = 2;
	lidarParam.FormatMono16 = true;
	lidarParam.FormatMono8 = false;
	lidarParam.Format16UC1 = false;
	lidarParam.FormatRGB8 = false;
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
	lidarParam.publishName = "roboscan_frame";
	lidarParam.selected_format = "2-1. image Format Mono16";
	lidarParam.maxDistance = 12500;

	lidarParam.paramSave = false;

	//roi_height

	lidarParam.areaBtn[0] = true;
	lidarParam.areaBtn[1] = true;
	lidarParam.areaBtn[2] = true;
	lidarParam.areaBtn[3] = true;

	lidarParam.areaScaleX[0] = 0.5;
	lidarParam.areaScaleX[1] = 0.5;
	lidarParam.areaScaleX[2] = 0.5;
	lidarParam.areaScaleX[3] = 0.5;

	lidarParam.areaScaleY[0] = 1;
	lidarParam.areaScaleY[1] = 1;
	lidarParam.areaScaleY[2] = 1;
	lidarParam.areaScaleY[3] = 1;

	lidarParam.areaScaleZ[0] = 1;
	lidarParam.areaScaleZ[1] = 1;
	lidarParam.areaScaleZ[2] = 1;
	lidarParam.areaScaleZ[3] = 1;

	lidarParam.areaPosX[0] = 0;
	lidarParam.areaPosX[1] = 0.5;
	lidarParam.areaPosX[2] = 1;
	lidarParam.areaPosX[3] = 1.5;

	lidarParam.areaPosY[0] = 0;
	lidarParam.areaPosY[1] = 0;
	lidarParam.areaPosY[2] = 0;
	lidarParam.areaPosY[3] = 0;

	lidarParam.areaPosZ[0] = 0;
	lidarParam.areaPosZ[1] = 0;
	lidarParam.areaPosZ[2] = 0;
	lidarParam.areaPosZ[3] = 0;

	lidarParam.minPoint[0] = 80;
	lidarParam.minPoint[1] = 80;
	lidarParam.minPoint[2] = 80;
	lidarParam.minPoint[3] = 80;

	this->declare_parameter<string>("0. IP Addr", lidarParam.ipAddr);
//	this->declare_parameter<string>("1. Net Mask", lidarParam.netMask);
//	this->declare_parameter<string>("2. GW Addr", lidarParam.gwAddr);
    this->declare_parameter<string>("1. Frame Id", lidarParam.publishName);
	this->declare_parameter<bool>("2-1. image Format Mono16", lidarParam.FormatMono16); 
	this->declare_parameter<bool>("2-2. image Format Mono8", lidarParam.FormatMono8); 
	this->declare_parameter<bool>("2-3. image Format 16UC1", lidarParam.Format16UC1); 
	this->declare_parameter<bool>("2-4. image Format RGB8", lidarParam.FormatRGB8); 

	this->declare_parameter<uint16_t>("B. lensType", lidarParam.lensType);
	this->declare_parameter<uint16_t>("C. imageType", lidarParam.imageType);
	this->declare_parameter<uint16_t>("D. hdr_mode", lidarParam.hdr_mode);
	this->declare_parameter<uint16_t>("E. int0", lidarParam.int0);
	this->declare_parameter<uint16_t>("F. int1", lidarParam.int1);
	this->declare_parameter<uint16_t>("G. int2", lidarParam.int2);
	this->declare_parameter<uint16_t>("H. intGr",lidarParam.intGr);
	this->declare_parameter<uint16_t>("I. minAmplitude", lidarParam.minAmplitude);
	this->declare_parameter<uint16_t>("J. modIndex", lidarParam.frequencyModulation);
	this->declare_parameter<uint16_t>("K. channel", lidarParam.channel);
	this->declare_parameter<uint16_t>("L. roi_leftX", lidarParam.roi_leftX);
	this->declare_parameter<uint16_t>("M. roi_topY", lidarParam.roi_topY);
	this->declare_parameter<uint16_t>("N. roi_rightX", lidarParam.roi_rightX);
//	this->declare_parameter<int>("O. roi_bottomY", lidarParam.roi_bottomY);
	this->declare_parameter<double>("P. transformAngle", lidarParam.transformAngle);
	this->declare_parameter<int>("Q. cutPixels", lidarParam.transformAngle);
	this->declare_parameter<bool>("R. medianFilter", lidarParam.medianFilter);
	this->declare_parameter<bool>("S. averageFilter", lidarParam.averageFilter);
	this->declare_parameter<double>("T. temporalFilterFactor", lidarParam.temporalFilterFactor);
	this->declare_parameter<uint16_t>("T. temporalFilterFactorThreshold", lidarParam.temporalFilterThreshold);
	this->declare_parameter<uint16_t>("U. edgeFilterThreshold", lidarParam.edgeFilterThreshold);
	//this->declare_parameter<int>("W temporalEdgeThresholdLow", lidarParam.temporalEdgeThresholdLow);
	//this->declare_parameter<int>("X temporalEdgeThresholdHigh", lidarParam.temporalEdgeThresholdHigh);
	this->declare_parameter<uint16_t>("V. interferenceDetectionLimit", lidarParam.interferenceDetectionLimit);
	this->declare_parameter<bool>("V. useLastValue", lidarParam.useLastValue);

	this->declare_parameter<bool>("A. cvShow", lidarParam.cvShow);
	this->declare_parameter<uint16_t>("W. dualBeam", lidarParam.dualBeam);
	this->declare_parameter<bool>("X. grayscale LED", lidarParam.grayscaleIlluminationMode);
	this->declare_parameter<bool>("Y. PointColud EDGE", lidarParam.pointCloudEdgeFilter);
	this->declare_parameter<int>("Z. MaxDistance", lidarParam.maxDistance);


	// Area 0
	this->declare_parameter<bool>("area0. Enabled", lidarParam.areaBtn[0]);
	this->declare_parameter<int>("area0. minimum_detection_point", lidarParam.minPoint[0]);

	this->declare_parameter<double>("area0. length_scale", lidarParam.areaScaleX[0]);
	this->declare_parameter<double>("area0. width_scale", lidarParam.areaScaleY[0]);
	this->declare_parameter<double>("area0. height_scale", lidarParam.areaScaleZ[0]);

	this->declare_parameter<double>("area0. length_position", lidarParam.areaPosX[0]);
	this->declare_parameter<double>("area0. width_position", lidarParam.areaPosY[0]);
	this->declare_parameter<double>("area0. height_position", lidarParam.areaPosZ[0]);

	// Area 1
	this->declare_parameter<bool>("area1. Enabled", lidarParam.areaBtn[1]);
	this->declare_parameter<int>("area1. minimum_detection_point", lidarParam.minPoint[1]);

	this->declare_parameter<double>("area1. length_scale", lidarParam.areaScaleX[1]);
	this->declare_parameter<double>("area1. width_scale", lidarParam.areaScaleY[1]);
	this->declare_parameter<double>("area1. height_scale", lidarParam.areaScaleZ[1]);

	this->declare_parameter<double>("area1. length_position", lidarParam.areaPosX[1]);
	this->declare_parameter<double>("area1. width_position", lidarParam.areaPosY[1]);
	this->declare_parameter<double>("area1. height_position", lidarParam.areaPosZ[1]);

	// Area 2
	this->declare_parameter<bool>("area2. Enabled", lidarParam.areaBtn[2]);
	this->declare_parameter<int>("area2. minimum_detection_point", lidarParam.minPoint[2]);

	this->declare_parameter<double>("area2. length_scale", lidarParam.areaScaleX[2]);
	this->declare_parameter<double>("area2. width_scale", lidarParam.areaScaleY[2]);
	this->declare_parameter<double>("area2. height_scale", lidarParam.areaScaleZ[2]);

	this->declare_parameter<double>("area2. length_position", lidarParam.areaPosX[2]);
	this->declare_parameter<double>("area2. width_position", lidarParam.areaPosY[2]);
	this->declare_parameter<double>("area2. height_position", lidarParam.areaPosZ[2]);

	// Area 3
	this->declare_parameter<bool>("area3. Enabled", lidarParam.areaBtn[3]);
	this->declare_parameter<int>("area3. minimum_detection_point", lidarParam.minPoint[3]);

	this->declare_parameter<double>("area3. length_scale", lidarParam.areaScaleX[3]);
	this->declare_parameter<double>("area3. width_scale", lidarParam.areaScaleY[3]);
	this->declare_parameter<double>("area3. height_scale", lidarParam.areaScaleZ[3]);

	this->declare_parameter<double>("area3. length_position", lidarParam.areaPosX[3]);
	this->declare_parameter<double>("area3. width_position", lidarParam.areaPosY[3]);
	this->declare_parameter<double>("area3. height_position", lidarParam.areaPosZ[3]);


	//roi_height

	this->get_parameter_or<std::string>("0. IP Addr", lidarParam.ipAddr, "192.168.0.220");
	//this->get_parameter_or<std::string>("1. Frame Id", lidarParam.publishName, "roboscan_frame");
	this->get_parameter_or<bool>("2-1. image Format Mono16", lidarParam.FormatMono16, true); 
	this->get_parameter_or<bool>("2-2. image Format Mono8", lidarParam.FormatMono8, false); 
	this->get_parameter_or<bool>("2-3. image Format 16UC1", lidarParam.Format16UC1, false); 
	this->get_parameter_or<bool>("2-4. image Format RGB8", lidarParam.FormatRGB8, false); 

	this->get_parameter_or<uint16_t>("B. lensType", lidarParam.lensType, 2);
	this->get_parameter_or<uint16_t>("C. imageType",  lidarParam.imageType, 2);
	this->get_parameter_or<uint16_t>("D. hdr_mode", lidarParam.hdr_mode, 0); //0 - hdr off, 1 - hdr spatial, 2 - hdr temporal
	this->get_parameter_or<uint16_t>("E. int0", lidarParam.int0, 1500);
	this->get_parameter_or<uint16_t>("F. int1", lidarParam.int1, 100);
	this->get_parameter_or<uint16_t>("G. int2", lidarParam.int2, 50);
	this->get_parameter_or<uint16_t>("H. intGr",lidarParam.intGr, 100); //integration times
	this->get_parameter_or<uint16_t>("I. minAmplitude", lidarParam.minAmplitude, 100);
	this->get_parameter_or<uint16_t>("J. modIndex", lidarParam.frequencyModulation, 0);
	this->get_parameter_or<uint16_t>("K. channel", lidarParam.channel, 0);
	this->get_parameter_or<uint16_t>("L. roi_leftX", lidarParam.roi_leftX, 0);
	this->get_parameter_or<uint16_t>("M. roi_topY", lidarParam.roi_topY, 0);
	this->get_parameter_or<uint16_t>("N. roi_rightX", lidarParam.roi_rightX, 319);
	//this->get_parameter_or<uint16_t>("O. roi_bottomY", lidarParam.roi_bottomY, 239);
	this->get_parameter_or<double>("P. transformAngle", lidarParam.transformAngle, 0);
	this->get_parameter_or<bool>("R. medianFilter", lidarParam.medianFilter, false);
	this->get_parameter_or<bool>("S. averageFilter", lidarParam.averageFilter, false);
	this->get_parameter_or<double>("T. temporalFilterFactor", lidarParam.temporalFilterFactor, 0.3);
	this->get_parameter_or<uint16_t>("T. temporalFilterFactorThreshold", lidarParam.temporalFilterThreshold, 200);
	this->get_parameter_or<uint16_t>("U. edgeFilterThreshold", lidarParam.edgeFilterThreshold, 0);
	//this->get_parameter_or<uint16_t>("W temporalEdgeThresholdLow", lidarParam.temporalEdgeThresholdLow, 0);
	//this->get_parameter_or<uint16_t>("X temporalEdgeThresholdHigh", lidarParam.temporalEdgeThresholdHigh, 0);
	this->get_parameter_or<uint16_t>("V. interferenceDetectionLimit", lidarParam.interferenceDetectionLimit, 0);
	this->get_parameter_or<bool>("V. useLastValue", lidarParam.useLastValue, false);
	this->get_parameter_or<uint16_t>("W. dualBeam", lidarParam.dualBeam, 2);
	this->get_parameter_or<bool>("X. grayscale LED", lidarParam.grayscaleIlluminationMode, false);
	this->get_parameter_or<bool>("Y. PointColud EDGE", lidarParam.pointCloudEdgeFilter, false);
	this->get_parameter_or<int>("Z. MaxDistance", lidarParam.maxDistance, 12500);


	// Area 0
	this->get_parameter_or<bool>("area0. Enabled", lidarParam.areaBtn[0], true);
	this->get_parameter_or<int>("area0. minimum_detection_point", lidarParam.minPoint[0], 80);

	this->get_parameter_or<double>("area0. length_scale", lidarParam.areaScaleX[0], 1);
	this->get_parameter_or<double>("area0. width_scale", lidarParam.areaScaleY[0], 1);
	this->get_parameter_or<double>("area0. height_scale", lidarParam.areaScaleZ[0], 1);

	this->get_parameter_or<double>("area0. length_position", lidarParam.areaPosX[0], 0);
	this->get_parameter_or<double>("area0. width_position", lidarParam.areaPosY[0], 0);
	this->get_parameter_or<double>("area0. height_position", lidarParam.areaPosZ[0], 0);

	// Area 1
	this->get_parameter_or<bool>("area1. Enabled", lidarParam.areaBtn[1], true);
	this->get_parameter_or<int>("area1. minimum_detection_point", lidarParam.minPoint[1], 80);

	this->get_parameter_or<double>("area1. length_scale", lidarParam.areaScaleX[1], 1);
	this->get_parameter_or<double>("area1. width_scale", lidarParam.areaScaleY[1], 1);
	this->get_parameter_or<double>("area1. height_scale", lidarParam.areaScaleZ[1], 1);

	this->get_parameter_or<double>("area1. length_position", lidarParam.areaPosX[1], 0);
	this->get_parameter_or<double>("area1. width_position", lidarParam.areaPosY[1], 1);
	this->get_parameter_or<double>("area1. height_position", lidarParam.areaPosZ[1], 0);

	// Area 2
	this->get_parameter_or<bool>("area2. Enabled", lidarParam.areaBtn[2], true);
	this->get_parameter_or<int>("area2. minimum_detection_point", lidarParam.minPoint[2], 80);

	this->get_parameter_or<double>("area2. length_scale", lidarParam.areaScaleX[2], 1);
	this->get_parameter_or<double>("area2. width_scale", lidarParam.areaScaleY[2], 1);
	this->get_parameter_or<double>("area2. height_scale", lidarParam.areaScaleZ[2], 1);

	this->get_parameter_or<double>("area2. length_position", lidarParam.areaPosX[2], 0);
	this->get_parameter_or<double>("area2. width_position", lidarParam.areaPosY[2], -1);
	this->get_parameter_or<double>("area2. height_position", lidarParam.areaPosZ[2], 0);

	// Area 3
	this->get_parameter_or<bool>("area3. Enabled", lidarParam.areaBtn[3], true);
	this->get_parameter_or<int>("area3. minimum_detection_point", lidarParam.minPoint[3], 80);

	this->get_parameter_or<double>("area3. length_scale", lidarParam.areaScaleX[3], 1);
	this->get_parameter_or<double>("area3. width_scale", lidarParam.areaScaleY[3], 1);
	this->get_parameter_or<double>("area3. height_scale", lidarParam.areaScaleZ[3], 1);

	this->get_parameter_or<double>("area3. length_position", lidarParam.areaPosX[3], 1);
	this->get_parameter_or<double>("area3. width_position", lidarParam.areaPosY[3], 0);
	this->get_parameter_or<double>("area3. height_position", lidarParam.areaPosZ[3], 0);


	setWinName();

	rclcpp::Parameter pIPAddr("0. IP Addr", lidarParam.ipAddr);
	rclcpp::Parameter FrameID("1. Frame Id", lidarParam.publishName);
//	rclcpp::Parameter pNetMask("1. Net Mask", lidarParam.netMask);
//	rclcpp::Parameter pGWAddr("2. GW Addr", lidarParam.gwAddr);
	rclcpp::Parameter pimageFormatMono16("2-1. image Format Mono16", lidarParam.FormatMono16); 
	rclcpp::Parameter pimageFormatMono8("2-2. image Format Mono8", lidarParam.FormatMono8); 
	rclcpp::Parameter pimageFormat16UC1("2-3. image Format 16UC1", lidarParam.Format16UC1); 
	rclcpp::Parameter pimageFormatRGB8("2-4. image Format RGB8", lidarParam.FormatRGB8); 

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
	//rclcpp::Parameter pRoi_bottomY("O. roi_bottomY", lidarParam.roi_bottomY);
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

	// Area 0 parameter
	rclcpp::Parameter area0btn("area0. Enabled", lidarParam.areaBtn[0]);
	rclcpp::Parameter area0minPoint("area0. minimum_detection_point", lidarParam.minPoint[0]);

	rclcpp::Parameter area0ScaleX("area0. length_scale", lidarParam.areaScaleX[0]);
	rclcpp::Parameter area0ScaleY("area0. width_scale", lidarParam.areaScaleY[0]);
	rclcpp::Parameter area0ScaleZ("area0. height_scale", lidarParam.areaScaleZ[0]);

	rclcpp::Parameter area0PosX("area0. length_position", lidarParam.areaPosX[0]);
	rclcpp::Parameter area0PosY("area0. width_position", lidarParam.areaPosY[0]);
	rclcpp::Parameter area0PosZ("area0. height_position", lidarParam.areaPosZ[0]);

	// Area 1 parameter
	rclcpp::Parameter area1btn("area1. Enabled", lidarParam.areaBtn[1]);
	rclcpp::Parameter area1minPoint("area1. minimum_detection_point", lidarParam.minPoint[1]);

	rclcpp::Parameter area1ScaleX("area1. length_scale", lidarParam.areaScaleX[1]);
	rclcpp::Parameter area1ScaleY("area1. width_scale", lidarParam.areaScaleY[1]);
	rclcpp::Parameter area1ScaleZ("area1. height_scale", lidarParam.areaScaleZ[1]);

	rclcpp::Parameter area1PosX("area1. length_position", lidarParam.areaPosX[1]);
	rclcpp::Parameter area1PosY("area1. width_position", lidarParam.areaPosY[1]);
	rclcpp::Parameter area1PosZ("area1. height_position", lidarParam.areaPosZ[1]);

	// Area 2 parameter
	rclcpp::Parameter area2btn("area2. Enabled", lidarParam.areaBtn[2]);
	rclcpp::Parameter area2minPoint("area2. minimum_detection_point", lidarParam.minPoint[2]);

	rclcpp::Parameter area2ScaleX("area2. length_scale", lidarParam.areaScaleX[2]);
	rclcpp::Parameter area2ScaleY("area2. width_scale", lidarParam.areaScaleY[2]);
	rclcpp::Parameter area2ScaleZ("area2. height_scale", lidarParam.areaScaleZ[2]);

	rclcpp::Parameter area2PosX("area2. length_position", lidarParam.areaPosX[2]);
	rclcpp::Parameter area2PosY("area2. width_position", lidarParam.areaPosY[2]);
	rclcpp::Parameter area2PosZ("area2. height_position", lidarParam.areaPosZ[2]);

	// Area 3 parameter
	rclcpp::Parameter area3btn("area3. Enabled", lidarParam.areaBtn[3]);
	rclcpp::Parameter area3minPoint("area3. minimum_detection_point", lidarParam.minPoint[3]);

	rclcpp::Parameter area3ScaleX("area3. length_scale", lidarParam.areaScaleX[3]);
	rclcpp::Parameter area3ScaleY("area3. width_scale", lidarParam.areaScaleY[3]);
	rclcpp::Parameter area3ScaleZ("area3. height_scale", lidarParam.areaScaleZ[3]);

	rclcpp::Parameter area3PosX("area3. length_position", lidarParam.areaPosX[3]);
	rclcpp::Parameter area3PosY("area3. width_position", lidarParam.areaPosY[3]);
	rclcpp::Parameter area3PosZ("area3. height_position", lidarParam.areaPosZ[3]);



	this->set_parameter(pIPAddr);
	this->set_parameter(FrameID);
//	this->set_parameter(pNetMask);
//	this->set_parameter(pGWAddr);

	this->set_parameter(pimageFormatMono16);
	this->set_parameter(pimageFormatMono8);
	this->set_parameter(pimageFormat16UC1);
	this->set_parameter(pimageFormatRGB8);


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
//	this->set_parameter(pRoi_bottomY);
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

	// Area 0
	this->set_parameter(area0btn);
	this->set_parameter(area0minPoint);
	this->set_parameter(area0ScaleX);
	this->set_parameter(area0ScaleY);
	this->set_parameter(area0ScaleZ);
	this->set_parameter(area0PosX);
	this->set_parameter(area0PosY);
	this->set_parameter(area0PosZ);

	// Area 1
	this->set_parameter(area1btn);
	this->set_parameter(area1minPoint);
	this->set_parameter(area1ScaleX);
	this->set_parameter(area1ScaleY);
	this->set_parameter(area1ScaleZ);
	this->set_parameter(area1PosX);
	this->set_parameter(area1PosY);
	this->set_parameter(area1PosZ);

	// Area 2
	this->set_parameter(area2btn);
	this->set_parameter(area2minPoint);
	this->set_parameter(area2ScaleX);
	this->set_parameter(area2ScaleY);
	this->set_parameter(area2ScaleZ);
	this->set_parameter(area2PosX);
	this->set_parameter(area2PosY);
	this->set_parameter(area2PosZ);

	// Area 3
	this->set_parameter(area3btn);
	this->set_parameter(area3minPoint);
	this->set_parameter(area3ScaleX);
	this->set_parameter(area3ScaleY);
	this->set_parameter(area3ScaleZ);
	this->set_parameter(area3PosX);
	this->set_parameter(area3PosY);
	this->set_parameter(area3PosZ);

	

	//std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cameraSub");
	//cameraInfoService = node->create_service<sensor_msgs::srv::SetCameraInfo>("cameraSub", &setCameraInfo);
	connectionCameraInfo = interface.subscribeCameraInfo([&](std::shared_ptr<CameraInfo> ci) -> void { updateCameraInfo(ci); });
	connectionFrames = interface.subscribeFrame([&](Frame* f) -> void {  updateFrame(f); });
	cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lidarParam.lensCenterOffsetX, lidarParam.lensCenterOffsetY, lidarParam.lensType); //0.02 mm - sensor pixel size

	//area0box
    area0Box.header.frame_id = lidarParam.publishName;
    area0Box.ns = "Markers_Box_" + std::to_string(0);
    area0Box.id = 0;
    area0Box.type = visualization_msgs::msg::Marker::CUBE;
    area0Box.action = visualization_msgs::msg::Marker::ADD;
    area0Box.pose.position.x = lidarParam.areaScaleX[0] / 2.0 + lidarParam.areaPosX[0];
    area0Box.pose.position.y = lidarParam.areaPosY[0];
    area0Box.pose.position.z = lidarParam.areaPosZ[0];
    area0Box.pose.orientation.x = 0.0;
    area0Box.pose.orientation.y = 0.0;
    area0Box.pose.orientation.z = 0.0; 
    area0Box.pose.orientation.w = 1.0;
    area0Box.scale.x = lidarParam.areaScaleX[0];
    area0Box.scale.y = lidarParam.areaScaleY[0];
    area0Box.scale.z = lidarParam.areaScaleZ[0];

    area0Box.color.r = 1.0f;
    area0Box.color.g = 0.0f;    
    area0Box.color.b = 0.0f;
    area0Box.color.a = 0.3f;

	lidarParam.x_min[0] = lidarParam.areaPosX[0] , lidarParam.x_max[0] = lidarParam.areaPosX[0] + lidarParam.areaScaleX[0] ;
	lidarParam.y_min[0] = lidarParam.areaPosY[0] - lidarParam.areaScaleY[0] / 2.0, lidarParam.y_max[0] = lidarParam.areaPosY[0] + lidarParam.areaScaleY[0] / 2.0;
	lidarParam.z_min[0] = lidarParam.areaPosZ[0] - lidarParam.areaScaleZ[0] / 2.0, lidarParam.z_max[0] = lidarParam.areaPosZ[0] + lidarParam.areaScaleZ[0] / 2.0;

	//area1box	
	area1Box.header.frame_id = lidarParam.publishName;
    area1Box.ns = "Markers_Box_" + std::to_string(1);
    area1Box.id = 1;
    area1Box.type = visualization_msgs::msg::Marker::CUBE;
    area1Box.action = visualization_msgs::msg::Marker::ADD;
	area1Box.pose.position.x = lidarParam.areaScaleX[1] / 2.0 + lidarParam.areaPosX[1];
	area1Box.pose.position.y = lidarParam.areaPosY[1];
	area1Box.pose.position.z = lidarParam.areaPosZ[1];	
    area1Box.pose.orientation.x = 0.0;
    area1Box.pose.orientation.y = 0.0;
    area1Box.pose.orientation.z = 0.0; 
    area1Box.pose.orientation.w = 1.0;
    area1Box.scale.x = lidarParam.areaScaleX[1];
    area1Box.scale.y = lidarParam.areaScaleY[1];
    area1Box.scale.z = lidarParam.areaScaleZ[1];

    area1Box.color.r = 1.0f;
    area1Box.color.g = 1.0f;    
    area1Box.color.b = 0.0f;
    area1Box.color.a = 0.3f;

	lidarParam.x_min[1] = lidarParam.areaPosX[1], lidarParam.x_max[1] = lidarParam.areaPosX[1] + lidarParam.areaScaleX[1];
	lidarParam.y_min[1] = lidarParam.areaPosY[1] - lidarParam.areaScaleY[1] / 2.0, lidarParam.y_max[1] = lidarParam.areaPosY[1] + lidarParam.areaScaleY[1] / 2.0;
	lidarParam.z_min[1] = lidarParam.areaPosZ[1] - lidarParam.areaScaleZ[1] / 2.0, lidarParam.z_max[1] = lidarParam.areaPosZ[1] + lidarParam.areaScaleZ[1] / 2.0;


	//area2box	
	area2Box.header.frame_id = lidarParam.publishName;
    area2Box.ns = "Markers_Box_" + std::to_string(2);
    area2Box.id = 2;
    area2Box.type = visualization_msgs::msg::Marker::CUBE;
    area2Box.action = visualization_msgs::msg::Marker::ADD;
	area2Box.pose.position.x = lidarParam.areaScaleX[2] / 2.0 + lidarParam.areaPosX[2];
	area2Box.pose.position.y = lidarParam.areaPosY[2];
	area2Box.pose.position.z = lidarParam.areaPosZ[2];
    area2Box.pose.orientation.x = 0.0;
    area2Box.pose.orientation.y = 0.0;
    area2Box.pose.orientation.z = 0.0; 
    area2Box.pose.orientation.w = 1.0;
    area2Box.scale.x = lidarParam.areaScaleX[2];
    area2Box.scale.y = lidarParam.areaScaleY[2];
    area2Box.scale.z = lidarParam.areaScaleZ[2];

    area2Box.color.r = 0.5f;
    area2Box.color.g = 1.0f;    
    area2Box.color.b = 0.0f;
    area2Box.color.a = 0.3f;

	lidarParam.x_min[2] = lidarParam.areaPosX[2], lidarParam.x_max[2] = lidarParam.areaPosX[2] + lidarParam.areaScaleX[2];
	lidarParam.y_min[2] = lidarParam.areaPosY[2] - lidarParam.areaScaleY[2] / 2.0, lidarParam.y_max[2] = lidarParam.areaPosY[2] + lidarParam.areaScaleY[2] / 2.0;
	lidarParam.z_min[2] = lidarParam.areaPosZ[2] - lidarParam.areaScaleZ[2] / 2.0, lidarParam.z_max[2] = lidarParam.areaPosZ[2] + lidarParam.areaScaleZ[2] / 2.0;


	//area3box	
	area3Box.header.frame_id = lidarParam.publishName;
    area3Box.ns = "Markers_Box_" + std::to_string(3);
    area3Box.id = 3;
    area3Box.type = visualization_msgs::msg::Marker::CUBE;
    area3Box.action = visualization_msgs::msg::Marker::ADD;
	area3Box.pose.position.x = lidarParam.areaScaleX[3] / 2.0 + lidarParam.areaPosX[3];
	area3Box.pose.position.y = lidarParam.areaPosY[3];
	area3Box.pose.position.z = lidarParam.areaPosZ[3];
    area3Box.pose.orientation.x = 0.0;
    area3Box.pose.orientation.y = 0.0;
    area3Box.pose.orientation.z = 0.0; 
    area3Box.pose.orientation.w = 1.0;
    area3Box.scale.x = lidarParam.areaScaleX[3];
    area3Box.scale.y = lidarParam.areaScaleY[3];
    area3Box.scale.z = lidarParam.areaScaleZ[3];

    area3Box.color.r = 0.0f;
    area3Box.color.g = 1.0f;    
    area3Box.color.b = 0.0f;
    area3Box.color.a = 0.3f;

	lidarParam.x_min[3] = lidarParam.areaPosX[3], lidarParam.x_max[3] = lidarParam.areaPosX[3] + lidarParam.areaScaleX[3];
	lidarParam.y_min[3] = lidarParam.areaPosY[3] - lidarParam.areaScaleY[3] / 2.0, lidarParam.y_max[3] = lidarParam.areaPosY[3] + lidarParam.areaScaleY[3] / 2.0;
	lidarParam.z_min[3] = lidarParam.areaPosZ[3] - lidarParam.areaScaleZ[3] / 2.0, lidarParam.z_max[3] = lidarParam.areaPosZ[3] + lidarParam.areaScaleZ[3] / 2.0;


	
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


	area0Box.header.frame_id = lidarParam.publishName;
	area1Box.header.frame_id = lidarParam.publishName;
	area2Box.header.frame_id = lidarParam.publishName;
	area3Box.header.frame_id = lidarParam.publishName;


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

int roboscanPublisher::AmplitudeColor24( float fValue, RGB888Pixel &nRGBData, const std::vector<cv::Vec3b> &colorVector, float fMaxValue)
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
	else if(fValue < 0)
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
        int index = static_cast<int>(fValue * (colorVector.size() / fMaxValue));

        if (index < 0) index = 0;
        if (index >= static_cast<int>(colorVector.size())) index = colorVector.size() - 1;

        const cv::Vec3b &bgr = colorVector.at(index);
        nRGBData.r = bgr[2];
        nRGBData.g = bgr[1];
        nRGBData.b = bgr[0];
    }
	return true;
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


void roboscanPublisher::ImageFormatChange()
{

	if (lidarParam.FormatChange)
	{
		lidarParam.FormatMono16 = (lidarParam.selected_format == "2-1. image Format Mono16");
		lidarParam.FormatMono8  = (lidarParam.selected_format == "2-2. image Format Mono8");
		lidarParam.Format16UC1  = (lidarParam.selected_format == "2-3. image Format 16UC1");
		lidarParam.FormatRGB8   = (lidarParam.selected_format == "2-4. image Format RGB8");
	
		this->set_parameters({
			rclcpp::Parameter("2-1. image Format Mono16", lidarParam.FormatMono16),
			rclcpp::Parameter("2-2. image Format Mono8",  lidarParam.FormatMono8),
			rclcpp::Parameter("2-3. image Format 16UC1",  lidarParam.Format16UC1),
			rclcpp::Parameter("2-4. image Format RGB8",   lidarParam.FormatRGB8)
		});
	
		lidarParam.FormatChange = false;
	} 

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

	if (!lidarParam.FormatMono16 && !lidarParam.FormatMono8 && !lidarParam.Format16UC1 && !lidarParam.FormatRGB8)
    {
		lidarParam.FormatChange = true;
		lidarParam.selected_format = "2-1. image Format Mono16";
		ImageFormatChange();
    }


	if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::DISTANCE_AMPLITUDE || frame->dataType == Frame::DISTANCE_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE ){
		if(lidarParam.FormatMono8) //mono 8
		{	

			ImageFormatChange();

			imgDistance.header.stamp = data_stamp;
			imgDistance.header.frame_id = lidarParam.publishName;
			imgDistance.height = static_cast<uint32_t>(frame->height);
			imgDistance.width = static_cast<uint32_t>(frame->width);
			imgDistance.encoding = sensor_msgs::image_encodings::MONO8;
			imgDistance.step = imgDistance.width * 1;
			imgDistance.is_bigendian = 0;

			const size_t nPixel = frame->width * frame->height;
			uint16_t distance = 0;
			imgDistance.data.resize(nPixel);

			for (size_t i = 0, l = 0; i < nPixel; ++i, l += 2) {
				distance = (frame->distData[l + 1] << 8) | frame->distData[l];
				imgDistance.data[i] = static_cast<uint8_t>(distance >> 8);
			}

			/* 정규화
			for (size_t i = 0, l = 0; i < nPixel; ++i, l += 2) {
				distance = (frame->distData[l + 1] << 8) + frame->distData[l];
				uint8_t data_value = static_cast<uint8_t>(std::min(255.0, (distance / (double)lidarParam.maxDistance) * 255.0));
				imgDistance.data[i] = data_value;
			}
			*/

			imgDistancePub->publish(imgDistance);
		}
		else if(lidarParam.Format16UC1) //16UC1
		{
			ImageFormatChange();

			imgDistance.header.stamp = data_stamp;
			imgDistance.header.frame_id = lidarParam.publishName;
			imgDistance.height = static_cast<uint32_t>(frame->height);
			imgDistance.width = static_cast<uint32_t>(frame->width);
			imgDistance.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
			imgDistance.step = imgDistance.width * frame->px_size;
			imgDistance.is_bigendian = 0;
			imgDistance.data = frame->distData;
			imgDistancePub->publish(imgDistance);
		}
		else if(lidarParam.FormatRGB8) //RGB8
		{		
			ImageFormatChange();

			imgDistance.header.stamp = data_stamp;
			imgDistance.header.frame_id = lidarParam.publishName;
			imgDistance.height = static_cast<uint32_t>(frame->height);
			imgDistance.width = static_cast<uint32_t>(frame->width);
			imgDistance.encoding = sensor_msgs::image_encodings::RGB8;
			imgDistance.step = imgDistance.width * 3;
			imgDistance.is_bigendian = 0;

			const size_t nPixel = frame->width * frame->height;
			uint16_t distance = 0;
			imgDistance.data.resize(nPixel * 3);

			RGB888Pixel color;

			for (size_t i = 0, l = 0; i < nPixel; ++i, l += 2) {
				distance = (frame->distData[l + 1] << 8) + frame->distData[l];
			
				Convert_To_RGB24((double)distance, color, 0.0f, lidarParam.maxDistance);
			
				imgDistance.data[3 * i + 0] = color.r; // R
				imgDistance.data[3 * i + 1] = color.g; // G
				imgDistance.data[3 * i + 2] = color.b; // B
			}
			imgDistancePub->publish(imgDistance);
		}
		else if(lidarParam.FormatMono16) //mono16
		{
			ImageFormatChange();

			imgDistance.header.stamp = data_stamp;
			imgDistance.header.frame_id = lidarParam.publishName;
			imgDistance.height = static_cast<uint32_t>(frame->height);
			imgDistance.width = static_cast<uint32_t>(frame->width);
			imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
			imgDistance.step = imgDistance.width * frame->px_size;
			imgDistance.is_bigendian = 0;
			imgDistance.data = frame->distData;
			imgDistancePub->publish(imgDistance);
		}
	}

	if(frame->dataType == Frame::DISTANCE_AMPLITUDE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){
		if(lidarParam.FormatMono8) //mono 8
		{
			imgAmpl.header.stamp = data_stamp;
			imgAmpl.header.frame_id = lidarParam.publishName;
			imgAmpl.height = static_cast<uint32_t>(frame->height);
			imgAmpl.width = static_cast<uint32_t>(frame->width);
			imgAmpl.encoding = sensor_msgs::image_encodings::MONO8;
			imgAmpl.step = imgAmpl.width;
			imgAmpl.is_bigendian = 0;

			uint16_t amplitude = 0;
			const size_t nPixel = frame->width * frame->height;
			imgAmpl.data.resize(nPixel);

			for (size_t i = 0, l = 0; i < nPixel; ++i, l += 2) {
				amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];
				imgAmpl.data[i] = static_cast<uint8_t>(amplitude >> 8);
			}

			/* 정규화
			for (size_t i = 0, l = 0; i < nPixel; ++i, l += 2) {
				amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];
				uint8_t data_value = static_cast<uint8_t>(std::min(255.0, (amplitude / (double)2897) * 255.0));
				imgAmpl.data[i] = data_value;
			}
			*/

			imgAmplPub->publish(imgAmpl);
		}
		else if(lidarParam.Format16UC1) //16UC1
		{
			imgAmpl.header.stamp = data_stamp;
			imgAmpl.header.frame_id = lidarParam.publishName;
			imgAmpl.height = static_cast<uint32_t>(frame->height);
			imgAmpl.width = static_cast<uint32_t>(frame->width);
			imgAmpl.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
			imgAmpl.step = imgAmpl.width * frame->px_size;
			imgAmpl.is_bigendian = 0;
			imgAmpl.data = frame->amplData;
			imgAmplPub->publish(imgAmpl);
		}
		else if(lidarParam.FormatRGB8) //RGB8
		{		
			imgAmpl.header.stamp = data_stamp;
			imgAmpl.header.frame_id = lidarParam.publishName;
			imgAmpl.height = static_cast<uint32_t>(frame->height);
			imgAmpl.width = static_cast<uint32_t>(frame->width);
			imgAmpl.encoding = sensor_msgs::image_encodings::RGB8;
			imgAmpl.step = imgAmpl.width * 3;
			imgAmpl.is_bigendian = 0;
			
			uint16_t amplitude = 0;
			const size_t nPixel = frame->width * frame->height;
			imgAmpl.data.resize(nPixel * 3);

			RGB888Pixel color;

			for (size_t i = 0, l = 0; i < nPixel; ++i, l += 2) {
				amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];
			
				AmplitudeColor24((double)amplitude, color, colorVector, 2897.0f);
				
				imgAmpl.data[3 * i + 0] = color.r; // R
				imgAmpl.data[3 * i + 1] = color.g; // G
				imgAmpl.data[3 * i + 2] = color.b; // B
			}

			imgAmplPub->publish(imgAmpl);
		}
		else if(lidarParam.FormatMono16) // mono16
		{
			imgAmpl.header.stamp = data_stamp;
			imgAmpl.header.frame_id = lidarParam.publishName;
			imgAmpl.height = static_cast<uint32_t>(frame->height);
			imgAmpl.width = static_cast<uint32_t>(frame->width);
			imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
			imgAmpl.step = imgAmpl.width * frame->px_size;
			imgAmpl.is_bigendian = 0;
			imgAmpl.data = frame->amplData;
			imgAmplPub->publish(imgAmpl);
		}
	}

	if(frame->dataType == Frame::GRAYSCALE || frame->dataType == Frame::DISTANCE_GRAYSCALE || frame->dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){
		imgGray.header.stamp = data_stamp;
		imgGray.header.frame_id = lidarParam.publishName;
		imgGray.height = static_cast<uint32_t>(frame->height);
		imgGray.width = static_cast<uint32_t>(frame->width);
		imgGray.encoding = sensor_msgs::image_encodings::MONO16;
		imgGray.step = imgGray.width * frame->px_size;
		imgGray.is_bigendian = 0;
		imgGray.data = frame->grayData;
		imgGrayPub->publish(imgGray);
	}

	if(frame->dataType == Frame::DCS){
		imgDCS.header.stamp = data_stamp;
		imgDCS.header.frame_id = lidarParam.publishName;
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
		cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
		cloud->header.frame_id = lidarParam.publishName;
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

		lidarParam.pointCount[0] = 0;
		lidarParam.pointCount[1] = 0;
		lidarParam.pointCount[2] = 0;
		lidarParam.pointCount[3] = 0;


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
								cartesianTransform.transformPixel(pc, y+lidarParam.roi_topY, d1, v0.x, v0.y, v0.z, lidarParam.transformAngle);
								cartesianTransform.transformPixel(pc+1, y+lidarParam.roi_topY, d2, v1.x, v1.y, v1.z, lidarParam.transformAngle);
								cartesianTransform.transformPixel(pc, y+1+lidarParam.roi_topY, d3, v2.x, v2.y, v2.z, lidarParam.transformAngle);

								if(edgeDetection(v0, v1, v2, 0.2, 30)){
									p.x = std::numeric_limits<float>::quiet_NaN();
									p.y = std::numeric_limits<float>::quiet_NaN();
									p.z = std::numeric_limits<float>::quiet_NaN();
									p.intensity = std::numeric_limits<float>::quiet_NaN();
									continue;
								}
							}
						}
					
						cartesianTransform.transformPixel(pc, y+lidarParam.roi_topY, distance, px, py, pz, lidarParam.transformAngle);
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

				//data check
				// area0
				if (lidarParam.areaBtn[0]) {
					if (p.x >= lidarParam.x_min[0] && p.x <= lidarParam.x_max[0] &&
						p.y >= lidarParam.y_min[0] && p.y <= lidarParam.y_max[0] &&
						p.z >= lidarParam.z_min[0] && p.z <= lidarParam.z_max[0]) 
					{
						lidarParam.pointCount[0]++;
					}	
					if(lidarParam.pointCount[0] > lidarParam.minPoint[0])
						lidarParam.pointDetect[0] = true;
					else
						lidarParam.pointDetect[0] = false; 
				}

				// area1
				if (lidarParam.areaBtn[1]) {
					if (p.x >= lidarParam.x_min[1] && p.x <= lidarParam.x_max[1] &&
						p.y >= lidarParam.y_min[1] && p.y <= lidarParam.y_max[1] &&
						p.z >= lidarParam.z_min[1] && p.z <= lidarParam.z_max[1]) 
					{
						lidarParam.pointCount[1]++;
					}
					if(lidarParam.pointCount[1] > lidarParam.minPoint[1])
						lidarParam.pointDetect[1] = true;
					else
						lidarParam.pointDetect[1] = false; 
				}

				// area2
				if (lidarParam.areaBtn[2]) {
					if (p.x >= lidarParam.x_min[2] && p.x <= lidarParam.x_max[2] &&
						p.y >= lidarParam.y_min[2] && p.y <= lidarParam.y_max[2] &&
						p.z >= lidarParam.z_min[2] && p.z <= lidarParam.z_max[2]) 
					{
						lidarParam.pointCount[2]++;
					}
					if(lidarParam.pointCount[2] > lidarParam.minPoint[2])
						lidarParam.pointDetect[2] = true;
					else
						lidarParam.pointDetect[2] = false; 
				}

				// area3
				if (lidarParam.areaBtn[3]) {
					if (p.x >= lidarParam.x_min[3] && p.x <= lidarParam.x_max[3] &&
						p.y >= lidarParam.y_min[3] && p.y <= lidarParam.y_max[3] &&
						p.z >= lidarParam.z_min[3] && p.z <= lidarParam.z_max[3]) 
					{
						lidarParam.pointCount[3]++;
					}
					if(lidarParam.pointCount[3] > lidarParam.minPoint[3])
						lidarParam.pointDetect[3] = true;
					else
						lidarParam.pointDetect[3] = false; 
				}

			}

		}

		pcl::toROSMsg(*cloud, msg);

		msg.header.stamp = data_stamp;
		msg.header.frame_id = lidarParam.publishName;
		pointcloudPub->publish(msg);  
		area0Pub->publish(area0Box);
		area1Pub->publish(area1Box);
		area2Pub->publish(area2Box);
		area3Pub->publish(area3Box);

        customMessage.header.frame_id = "roboscan_frame";	
        customMessage.header.stamp = data_stamp;
        customMessage.area0 = lidarParam.pointDetect[0];
        customMessage.point0 = lidarParam.pointCount[0];

        customMessage.area1 = lidarParam.pointDetect[1];
        customMessage.point1 = lidarParam.pointCount[1];

        customMessage.area2 = lidarParam.pointDetect[2];
        customMessage.point2 = lidarParam.pointCount[2];

        customMessage.area3 = lidarParam.pointDetect[3];    
        customMessage.point3 = lidarParam.pointCount[3];       
		
		areaMsgpub->publish(customMessage);
	
		

		
		if(lidarParam.paramLoad)
		{
			paramLoad();	
		}
		
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
	else if( frame->dataType == Frame::GRAYSCALE ){
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
    cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
	cv_ptr->header.stamp = data_stamp;
	cv_ptr->header.frame_id = lidarParam.publishName;
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

