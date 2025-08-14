#include <cstdio>
#include <chrono>
#include <functional>
#include <filesystem>
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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>

#include <yaml-cpp/yaml.h>
#include "roboscan_publish_node.hpp"

using namespace NslOption;
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

#define DISTANCE_INFO_HEIGHT	80

std::atomic<int> x_start = -1, y_start = -1;
std::unique_ptr<NslPCD> latestFrame = std::make_unique<NslPCD>();


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

    RCLCPP_INFO(this->get_logger(), "start roboscanPublisher...\n");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    imgDistancePub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
    imgAmplPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanAmpl", qos_profile); 
    imgGrayPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanGray", qos_profile); 
    pointcloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 

    roboscanPublisher::initialise();
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&roboscanPublisher::parametersCallback, this, std::placeholders::_1));

	reconfigure = false;
	mouseXpos = -1;
	mouseYpos = -1;
	runThread = true;
    publisherThread.reset(new boost::thread(boost::bind(&roboscanPublisher::thread_callback, this)));

    RCLCPP_INFO(this->get_logger(), "\nRun rqt to view the image!\n");
} 

roboscanPublisher::~roboscanPublisher()
{
	runThread = false;
	publisherThread->join();

	nsl_close();

    RCLCPP_INFO(this->get_logger(), "\nEnd roboscanPublisher()!\n");
}

void roboscanPublisher::thread_callback()
{
	auto lastTime = chrono::steady_clock::now();
	int frameCount = 0;

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	paramLoad();

	RCLCPP_INFO(this->get_logger(), "reconfigure = %d load = %d\n", reconfigure, viewerParam.paramLoad);

	while(runThread){

		if( reconfigure ){
			reconfigure = false;
			setReconfigure();
		}

		if( nsl_getPointCloudData(nsl_handle, latestFrame.get()) == NSL_ERROR_TYPE::NSL_SUCCESS )
		{
			frameCount++;			
			publishFrame(latestFrame.get());
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		auto now = chrono::steady_clock::now();
		auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - lastTime).count();
		if( elapsed >= 1000 ){
			viewerParam.frameCount = frameCount;
			frameCount = 0;
			lastTime = now;
//			RCLCPP_INFO(this->get_logger(), "frame = %d fps\n", viewerParam.frameCount);
		}
		
	}

	cv::destroyAllWindows();
	RCLCPP_INFO(this->get_logger(), "end thread_callback\n");
}


rcl_interfaces::msg::SetParametersResult roboscanPublisher::parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
{
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";

	if( viewerParam.paramLoad == false )
		viewerParam.paramSave = true;

	RCLCPP_INFO(this->get_logger(), "paramSave = %d\n", viewerParam.paramSave);
	
	// Here update class attributes, do some actions, etc.
	for (const auto &param: parameters)
	{
		if (param.get_name() == "A. cvShow")
		{
			
			bool showCv = param.as_bool();
			if( viewerParam.cvShow != showCv ){
				viewerParam.cvShow = showCv;
				viewerParam.changedCvShow = true;
				viewerParam.paramSave = false;
			}
			
		}
		else if (param.get_name() == "B. lensType")
		{
			nslConfig.lensType = static_cast<NslOption::LENS_TYPE>(param.as_int());
		}
		else if (param.get_name() == "C. imageType")
		{
			int imgType = param.as_int();			
			if( static_cast<int>(nslConfig.operationModeOpt) != imgType ){
				nslConfig.operationModeOpt = static_cast<NslOption::OPERATION_MODE_OPTIONS>(imgType);
				viewerParam.changedCvShow = true;
			}
		}
		else if (param.get_name() == "D. hdr_mode")
		{
			nslConfig.hdrOpt = static_cast<NslOption::HDR_OPTIONS>(param.as_int());
		}
		else if (param.get_name() == "E. int0")
		{
			nslConfig.integrationTime3D = param.as_int();
		}
		else if (param.get_name() == "F. int1")
		{
			nslConfig.integrationTime3DHdr1 = param.as_int();
		}
		else if (param.get_name() == "G. int2")
		{
			nslConfig.integrationTime3DHdr2 = param.as_int();
		}
		else if (param.get_name() == "H. intGr")
		{
			nslConfig.integrationTimeGrayScale = param.as_int();
		}
		else if (param.get_name() == "I. minAmplitude")
		{
			nslConfig.minAmplitude = param.as_int();
		}
		else if (param.get_name() == "J. modIndex")
		{
			nslConfig.mod_frequencyOpt = static_cast<NslOption::MODULATION_OPTIONS>(param.as_int());
		}
		else if (param.get_name() == "K. channel")
		{
			nslConfig.mod_channelOpt = static_cast<NslOption::MODULATION_CH_OPTIONS>(param.as_int());
		}
		else if (param.get_name() == "L. roi_leftX")
		{
			int x1_tmp = param.as_int();

			if(x1_tmp % X_INTERVAL ) x1_tmp+=X_INTERVAL-(x1_tmp % X_INTERVAL );
			if(x1_tmp > LEFTX_MAX ) x1_tmp = LEFTX_MAX;

			nslConfig.roiXMin = x1_tmp;

		}
		else if (param.get_name() == "N. roi_rightX")
		{
			int x2_tmp = param.as_int();
			
			if((x2_tmp-RIGHTX_MIN) % X_INTERVAL)	x2_tmp-=((x2_tmp-RIGHTX_MIN) % X_INTERVAL);
			if(x2_tmp < RIGHTX_MIN ) x2_tmp = RIGHTX_MIN;
			if(x2_tmp > RIGHTX_MAX ) x2_tmp = RIGHTX_MAX;
			
			nslConfig.roiXMax = x2_tmp;
		}
		else if (param.get_name() == "M. roi_topY")
		{
			int y1_tmp = param.as_int();
			
			if(y1_tmp % Y_INTERVAL )	y1_tmp++;
			if(y1_tmp > LEFTY_MAX ) y1_tmp = LEFTY_MAX;
			
			nslConfig.roiYMin = y1_tmp;
			
			int y2_tmp = RIGHTY_MAX - y1_tmp;
			nslConfig.roiYMax = y2_tmp;
		}
		else if (param.get_name() == "O. roi_bottomY")
		{
			int y2_tmp = param.as_int();
			
			if(y2_tmp % Y_INTERVAL == 0 )	y2_tmp++;
			if(y2_tmp < RIGHTY_MIN ) y2_tmp = RIGHTY_MIN;
			if(y2_tmp > RIGHTY_MAX ) y2_tmp = RIGHTY_MAX;
			
			nslConfig.roiYMax = y2_tmp;
			
			int y1_tmp = RIGHTY_MAX - y2_tmp;
			nslConfig.roiYMin = y1_tmp;
		}
		else if (param.get_name() == "P. transformAngle")
		{
			nslConfig.lidarAngle = param.as_double();
			if( viewerParam.paramLoad == false )
				viewerParam.reOpenLidar = true;
		}
		else if (param.get_name() == "R. medianFilter")
		{
			nslConfig.medianOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "S. averageFilter")
		{
			nslConfig.gaussOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "T. temporalFilterFactor")
		{
			nslConfig.temporalFactorValue = static_cast<int>(param.as_double()*1000);
			if( nslConfig.temporalFactorValue > 1000 ) nslConfig.temporalFactorValue = 1000;
			if( nslConfig.temporalFactorValue < 0 ) nslConfig.temporalFactorValue = 0;
		}
		else if (param.get_name() == "T. temporalFilterFactorThreshold")
		{
			nslConfig.temporalThresholdValue = param.as_int();
			if( nslConfig.temporalThresholdValue < 0 ) nslConfig.temporalThresholdValue = 0;
		}
		else if (param.get_name() == "U. edgeFilterThreshold")
		{
			nslConfig.edgeThresholdValue = param.as_int();
			if( nslConfig.edgeThresholdValue < 0 ) nslConfig.edgeThresholdValue = 0;
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
			nslConfig.interferenceDetectionLimitValue = param.as_int();
			if( nslConfig.interferenceDetectionLimitValue > 1000 ) nslConfig.interferenceDetectionLimitValue = 1000;
			if( nslConfig.interferenceDetectionLimitValue < 0 ) nslConfig.interferenceDetectionLimitValue = 0;
		}
		else if (param.get_name() == "V. useLastValue")
		{
			nslConfig.interferenceDetectionLastValueOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "W. dualBeam")
		{
			int dualBeam = param.as_int();
			if( dualBeam > 2 || dualBeam < 0 ) dualBeam = 0;
			nslConfig.dbModOpt = static_cast<NslOption::DUALBEAM_MOD_OPTIONS>(dualBeam);
		}
		else if (param.get_name() == "X. grayscale LED")
		{
			nslConfig.grayscaleIlluminationOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "Y. PointColud EDGE")
		{
			viewerParam.pointCloudEdgeFilter = param.as_bool();
		}
		else if (param.get_name() == "Z. MaxDistance")
		{
			viewerParam.maxDistance = param.as_int();
		}
		else if (param.get_name() == "0. IP Addr")
		{
			string tmpIp = param.as_string();
			if( tmpIp != viewerParam.ipAddr ) {
				RCLCPP_INFO(this->get_logger(), "changed IP addr %s -> %s\n", viewerParam.ipAddr.c_str(), tmpIp.c_str());

				viewerParam.changedIpInfo = true;
				viewerParam.ipAddr = tmpIp;
			}
		}
		else if (param.get_name() == "1. Net Mask")
		{
			string tmpIp = param.as_string();
			if( tmpIp != viewerParam.netMask ) {
				RCLCPP_INFO(this->get_logger(), "changed Netmask addr %s -> %s\n", viewerParam.netMask.c_str(), tmpIp.c_str());
				viewerParam.changedIpInfo = true;
				viewerParam.netMask= tmpIp;
			}
		}
		else if (param.get_name() == "2. GW Addr")
		{
			string tmpIp = param.as_string();
			if( tmpIp != viewerParam.gwAddr ) {
				RCLCPP_INFO(this->get_logger(), "changed Gw addr %s -> %s\n", viewerParam.gwAddr.c_str(), tmpIp.c_str());
				viewerParam.changedIpInfo = true;
				viewerParam.gwAddr= tmpIp;
			}
		}
	}

	reconfigure = true;
	return result;
}

void roboscanPublisher::timeDelay(int milli)
{
	auto start = std::chrono::steady_clock::now();
	while ( runThread != 0 ) {
		auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() >= milli) {
			break;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}


// Save & Load Test
void roboscanPublisher::paramDump(const std::string & filename)
{
    std::string package_path = ament_index_cpp::get_package_share_directory("roboscan_nsl3130");

    std::string full_path = package_path + "/" + filename;

    std::string command = "ros2 param dump /roboscan_publish_node > " + full_path;

	std::cout << "SAVE::system = "<< command <<std::endl;

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
	
}


void roboscanPublisher::paramLoad() // file exist check
{
	bool loadSuccess = false;

	std::string yaml_file = ament_index_cpp::get_package_share_directory("roboscan_nsl3130") + "/rqt.yaml";

	std::string command = "ros2 param load /roboscan_publish_node " + yaml_file + " > /dev/null";

	if (std::filesystem::exists(yaml_file)) {
		int result = std::system(command.c_str());

		if (result == 0) {
			std::cout << "Load successful!!!!!" << std::endl;
			loadSuccess = true;
		} else {
			std::cerr << "Load failed!!" << std::endl;
		}
	} else {
		std::cerr << "not exist yaml file: " << yaml_file << std::endl;
	}

	if( loadSuccess == false ){
		setReconfigure();
	}
}

void roboscanPublisher::setReconfigure()
{	
	if( viewerParam.paramLoad )
	{
		viewerParam.paramLoad = false;
		NslOption::OPERATION_MODE_OPTIONS mode = nslConfig.operationModeOpt;

		nsl_handle = nsl_open(viewerParam.ipAddr.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
		if( nsl_handle < 0 ){
			std::cout << "nsl_open::handle open error::" << nsl_handle << std::endl;
			exit(0);
		}

		nslConfig.operationModeOpt = mode;
		std::cout << "nsl_open :: mode = "<< (int)mode << std::endl;
	}
	else
	{
		if(!viewerParam.paramLoad && viewerParam.paramSave )
		{
			viewerParam.paramSave = false;
			std::future<void> result = std::async(std::launch::async, [&]() {
				paramDump("rqt_temp.yaml");
			});
		}
		
		
		nsl_streamingOff(nsl_handle);

		std::cout << "nsl_open :: changedIpInfo = "<< viewerParam.changedIpInfo << " reOpenLidar = "<< viewerParam.reOpenLidar << std::endl;

		if( viewerParam.changedIpInfo || viewerParam.reOpenLidar ){
		
			if( viewerParam.changedIpInfo ){
				// automatic reboot device
				paramDump("rqt_temp.yaml");
				nsl_setIpAddress(nsl_handle, viewerParam.ipAddr.c_str(), viewerParam.netMask.c_str(), viewerParam.gwAddr.c_str());
				RCLCPP_INFO(this->get_logger(), "wait time 5 sec\n");
				timeDelay(5000);
			}

			RCLCPP_INFO(this->get_logger(), "========== %d, %d ============\n", viewerParam.changedIpInfo, viewerParam.reOpenLidar );
			
			NslOption::OPERATION_MODE_OPTIONS mode = nslConfig.operationModeOpt;
			nsl_closeHandle(nsl_handle);
			nsl_handle = nsl_open(viewerParam.ipAddr.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
			nslConfig.operationModeOpt = mode;
		
			viewerParam.changedIpInfo = false;
			viewerParam.reOpenLidar = false;
		}
	}

	
	nsl_setMinAmplitude(nsl_handle, nslConfig.minAmplitude);
	nsl_setIntegrationTime(nsl_handle, nslConfig.integrationTime3D, nslConfig.integrationTime3DHdr1, nslConfig.integrationTime3DHdr2, nslConfig.integrationTimeGrayScale);
	nsl_setHdrMode(nsl_handle, nslConfig.hdrOpt);
	nsl_setFilter(nsl_handle, nslConfig.medianOpt, nslConfig.gaussOpt, nslConfig.temporalFactorValue, nslConfig.temporalThresholdValue, nslConfig.edgeThresholdValue, nslConfig.interferenceDetectionLimitValue, nslConfig.interferenceDetectionLastValueOpt);
	nsl_setAdcOverflowSaturation(nsl_handle, nslConfig.overflowOpt, nslConfig.saturationOpt);
	nsl_setDualBeam(nsl_handle, nslConfig.dbModOpt, nslConfig.dbOpsOpt);
	nsl_setModulation(nsl_handle, nslConfig.mod_frequencyOpt, nslConfig.mod_channelOpt, nslConfig.mod_enabledAutoChannelOpt);
	nsl_setRoi(nsl_handle, nslConfig.roiXMin, nslConfig.roiYMin, nslConfig.roiXMax, nslConfig.roiYMax);
	nsl_setGrayscaleillumination(nsl_handle, nslConfig.grayscaleIlluminationOpt);

	waitKey(1);
	setWinName();

	startStreaming();

	std::cout << "end setReconfigure"<< std::endl;

}

void roboscanPublisher::setWinName()
{
	bool changedCvShow = viewerParam.changedCvShow;
	viewerParam.changedCvShow = false;
	
	if( changedCvShow ){
		cv::destroyAllWindows();
	}
	
	if( viewerParam.cvShow == false || changedCvShow == false ) return;
	
	if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_MODE){
		sprintf(winName,"%s(Dist)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE){
		sprintf(winName,"%s(Dist/Ampl)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE){
		sprintf(winName,"%s(Dist/Gray)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_MODE){
		sprintf(winName,"%s(RGB)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE){
		sprintf(winName,"%s(RGB/Dist)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE){
		sprintf(winName,"%s(RGB/Dist/Ampl)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE){
		sprintf(winName,"%s(RGB/Dist/Gray)", WIN_NAME);
	}
	else{
		sprintf(winName,"%s(READY)", WIN_NAME);
	}
	
	cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
	cv::setWindowProperty(winName, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(winName, callback_mouse_click, NULL);
}


void roboscanPublisher::initialise()
{
	std::cout << "Init roboscan_nsl3130 node\n"<< std::endl;

	viewerParam.frameCount = 0;
	viewerParam.maxDistance = 12500;
	viewerParam.cvShow = false;
	viewerParam.changedCvShow = true;
	viewerParam.changedIpInfo = false;
	viewerParam.reOpenLidar = false;
	viewerParam.pointCloudEdgeFilter = true;
	viewerParam.paramSave = false;
	viewerParam.paramLoad = true;

	viewerParam.ipAddr = "192.168.0.220";
	viewerParam.netMask = "255.255.255.0";
	viewerParam.gwAddr = "192.168.0.1";

	nslConfig.lidarAngle = 0;
	nslConfig.lensType = NslOption::LENS_TYPE::LENS_SF;

	nslConfig.minAmplitude = 50;
	nslConfig.integrationTime3D = 1000;
	nslConfig.integrationTime3DHdr1 = 300;
	nslConfig.integrationTime3DHdr2 = 0;
	nslConfig.integrationTimeGrayScale = 100;
	
	nslConfig.hdrOpt = NslOption::HDR_OPTIONS::HDR_NONE_MODE;

	nslConfig.medianOpt = NslOption::FUNCTION_OPTIONS::FUNC_ON;
	nslConfig.gaussOpt = NslOption::FUNCTION_OPTIONS::FUNC_ON;
	nslConfig.temporalFactorValue = 300;
	nslConfig.temporalThresholdValue = 200;
	nslConfig.edgeThresholdValue = 200;
	nslConfig.interferenceDetectionLimitValue = 0;
	nslConfig.interferenceDetectionLastValueOpt = NslOption::FUNCTION_OPTIONS::FUNC_OFF;

	nslConfig.overflowOpt = NslOption::FUNCTION_OPTIONS::FUNC_ON;
	nslConfig.saturationOpt = NslOption::FUNCTION_OPTIONS::FUNC_ON;
	
	nslConfig.dbModOpt = NslOption::DUALBEAM_MOD_OPTIONS::DB_OFF;
	nslConfig.dbOpsOpt = NslOption::DUALBEAM_OPERATION_OPTIONS::DB_CORRECTION;

	nslConfig.mod_frequencyOpt = NslOption::MODULATION_OPTIONS::MOD_12Mhz;
	nslConfig.mod_channelOpt = NslOption::MODULATION_CH_OPTIONS::MOD_CH0;
	nslConfig.mod_enabledAutoChannelOpt = NslOption::FUNCTION_OPTIONS::FUNC_OFF;

	nslConfig.roiXMin = 0;
	nslConfig.roiYMin = 0;
	nslConfig.roiXMax = 319;
	nslConfig.roiYMax = 239;
	
	nslConfig.grayscaleIlluminationOpt = NslOption::FUNCTION_OPTIONS::FUNC_OFF;

	setWinName();

	rclcpp::Parameter pIPAddr("0. IP Addr", viewerParam.ipAddr);
//	rclcpp::Parameter pNetMask("1. Net Mask", viewerParam.netMask);
//	rclcpp::Parameter pGWAddr("2. GW Addr", viewerParam.gwAddr);

	rclcpp::Parameter pCvShow("A. cvShow", viewerParam.cvShow);
	rclcpp::Parameter pLensType("B. lensType", static_cast<int>(nslConfig.lensType));
	rclcpp::Parameter pImageType("C. imageType", static_cast<int>(nslConfig.operationModeOpt));
	rclcpp::Parameter pHdr_mode("D. hdr_mode", static_cast<int>(nslConfig.hdrOpt));
	rclcpp::Parameter pInt0("E. int0", nslConfig.integrationTime3D);
	rclcpp::Parameter pInt1("F. int1", nslConfig.integrationTime3DHdr1);
	rclcpp::Parameter pInt2("G. int2", nslConfig.integrationTime3DHdr2);
	rclcpp::Parameter pIntGr("H. intGr", nslConfig.integrationTimeGrayScale);
	rclcpp::Parameter pMinAmplitude("I. minAmplitude", nslConfig.minAmplitude);
	rclcpp::Parameter pModIndex("J. modIndex", static_cast<int>(nslConfig.mod_frequencyOpt));
	rclcpp::Parameter pChannel("K. channel", static_cast<int>(nslConfig.mod_channelOpt));
	rclcpp::Parameter pRoi_leftX("L. roi_leftX", nslConfig.roiXMin);
	rclcpp::Parameter pRoi_topY("M. roi_topY", nslConfig.roiYMin);
	rclcpp::Parameter pRoi_rightX("N. roi_rightX", nslConfig.roiXMax);
	//rclcpp::Parameter pRoi_bottomY("O. roi_bottomY", nslConfig.roiYMax);
	
	rclcpp::Parameter pTransformAngle("P. transformAngle", nslConfig.lidarAngle);
	rclcpp::Parameter pMedianFilter("R. medianFilter", static_cast<int>(nslConfig.medianOpt));
	rclcpp::Parameter pAverageFilter("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt));
	rclcpp::Parameter pTemporalFilterFactor("T. temporalFilterFactor", nslConfig.temporalFactorValue);
	rclcpp::Parameter pTemporalFilterThreshold("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue);
	rclcpp::Parameter pEdgeFilterThreshold("U. edgeFilterThreshold", nslConfig.edgeThresholdValue);
	rclcpp::Parameter pInterferenceDetectionLimit("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue);
	rclcpp::Parameter pUseLastValue("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt));


	rclcpp::Parameter pDualBeam("W. dualBeam", static_cast<int>(nslConfig.dbModOpt));
	rclcpp::Parameter pGrayLED("X. grayscale LED", static_cast<int>(nslConfig.grayscaleIlluminationOpt));
	rclcpp::Parameter pPCEdgeFilter("Y. PointColud EDGE", viewerParam.pointCloudEdgeFilter);
	rclcpp::Parameter pMaxDistance("Z. MaxDistance", viewerParam.maxDistance);

	this->declare_parameter<string>("0. IP Addr", viewerParam.ipAddr);
//	this->declare_parameter<string>("1. Net Mask", viewerParam.netMask);
//	this->declare_parameter<string>("2. GW Addr", viewerParam.gwAddr);
	this->declare_parameter<bool>("A. cvShow", viewerParam.cvShow);
	this->declare_parameter<int>("B. lensType", static_cast<int>(nslConfig.lensType));
	this->declare_parameter<int>("C. imageType", static_cast<int>(nslConfig.operationModeOpt));
	this->declare_parameter<int>("D. hdr_mode", static_cast<int>(nslConfig.hdrOpt));
	this->declare_parameter<int>("E. int0", nslConfig.integrationTime3D);
	this->declare_parameter<int>("F. int1", nslConfig.integrationTime3DHdr1);
	this->declare_parameter<int>("G. int2", nslConfig.integrationTime3DHdr2);
	this->declare_parameter<int>("H. intGr",nslConfig.integrationTimeGrayScale);
	this->declare_parameter<int>("I. minAmplitude", nslConfig.minAmplitude);
	this->declare_parameter<int>("J. modIndex", static_cast<int>(nslConfig.mod_frequencyOpt));
	this->declare_parameter<int>("K. channel", static_cast<int>(nslConfig.mod_channelOpt));
	this->declare_parameter<int>("L. roi_leftX", nslConfig.roiXMin);
	this->declare_parameter<int>("M. roi_topY",  nslConfig.roiYMin);
	this->declare_parameter<int>("N. roi_rightX", nslConfig.roiXMax);
//	this->declare_parameter<int>("O. roi_bottomY", nslConfig.roiYMax);

	this->declare_parameter<double>("P. transformAngle", nslConfig.lidarAngle);
	this->declare_parameter<bool>("R. medianFilter", static_cast<int>(nslConfig.medianOpt));
	this->declare_parameter<bool>("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt));
	this->declare_parameter<double>("T. temporalFilterFactor", nslConfig.temporalFactorValue);
	this->declare_parameter<int>("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue);
	this->declare_parameter<int>("U. edgeFilterThreshold", nslConfig.edgeThresholdValue);
	this->declare_parameter<int>("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue);
	this->declare_parameter<bool>("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt));


	this->declare_parameter<int>("W. dualBeam", static_cast<int>(nslConfig.dbModOpt));
	this->declare_parameter<bool>("X. grayscale LED", static_cast<int>(nslConfig.grayscaleIlluminationOpt));
	this->declare_parameter<bool>("Y. PointColud EDGE", viewerParam.pointCloudEdgeFilter);
	this->declare_parameter<int>("Z. MaxDistance", viewerParam.maxDistance);

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
//	this->set_parameter(pRoi_bottomY);
	this->set_parameter(pTransformAngle);
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

}


void roboscanPublisher::startStreaming()
{
	toString(nslConfig.operationModeOpt);
	
	if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE);
	}
	else{
		std::cout << "operation mode NONE~~~"<< std::endl;
	}
}


cv::Mat roboscanPublisher::addDistanceInfo(cv::Mat distMat, NslPCD *frame)
{
	int xpos = mouseXpos;
	int ypos = mouseYpos;
	
	if( (ypos > 0 && ypos < frame->height)){
		// mouseXpos, mouseYpos
//		int origin_xpos = xpos;
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		line(distMat, Point(xpos-10, ypos), Point(xpos+10, ypos), Scalar(255, 255, 0), 2);
		line(distMat, Point(xpos, ypos-10), Point(xpos, ypos+10), Scalar(255, 255, 0), 2);

		if( xpos >= frame->width ){ 
			xpos -= frame->width;
		}

		string dist2D_caption;
		string dist3D_caption;
		string info_caption;

		int distance2D = frame->distance2D[ypos][xpos];
		if( distance2D > NSL_LIMIT_FOR_VALID_DATA ){
			if( distance2D == NSL_ADC_OVERFLOW )
				dist2D_caption = format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( distance2D == NSL_SATURATION )
				dist2D_caption = format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( distance2D == NSL_BAD_PIXEL )
				dist2D_caption = format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( distance2D == NSL_INTERFERENCE )
				dist2D_caption = format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( distance2D == NSL_EDGE_DETECTED )
				dist2D_caption = format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist2D_caption = format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE || frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ) {
				dist2D_caption = format("2D X:%d Y:%d %dmm/%dlsb", xpos, ypos, frame->distance2D[ypos][xpos], frame->amplitude[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", frame->distance3D[OUT_X][ypos][xpos], frame->distance3D[OUT_Y][ypos][xpos], frame->distance3D[OUT_Z][ypos][xpos]);
			}
			else{
				dist2D_caption = format("2D X:%d Y:%d <%d>mm", xpos, ypos, frame->distance2D[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", frame->distance3D[OUT_X][ypos][xpos], frame->distance3D[OUT_Y][ypos][xpos], frame->distance3D[OUT_Z][ypos][xpos]);
			}
		}
		
		info_caption = format("%s:%dx%d %.2f'C, %d fps", toString(frame->operationMode), frame->width, frame->height, frame->temperature, viewerParam.frameCount);

		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist2D_caption.c_str(), Point(10, 46), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist3D_caption.c_str(), Point(10, 70), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		vconcat(distMat, infoImage, distMat);
	}
	else{
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		string info_caption = format("%s:%dx%d %.2f'C, %d fps", toString(frame->operationMode), frame->width, frame->height, frame->temperature, viewerParam.frameCount);
		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}

void setMatrixColor(Mat image, int x, int y, NslVec3b color)
{
	image.at<Vec3b>(y,x)[0] = color[0];
	image.at<Vec3b>(y,x)[1] = color[1];
	image.at<Vec3b>(y,x)[2] = color[2];
}

void roboscanPublisher::publishFrame(NslPCD *frame)
{
	static rclcpp::Clock s_rclcpp_clock;
	auto data_stamp = s_rclcpp_clock.now();

//	std::chrono::system_clock::time_point update_start = std::chrono::system_clock::now();

	cv::Mat distanceMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// distance
	cv::Mat amplitudeMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// amplitude
#ifdef image_transfer_function
	cv::Mat rgbMat(NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, CV_8UC3, Scalar(255, 255, 255));
#endif


	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE )
	{
		sensor_msgs::msg::Image imgDistance;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;
		
		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->distance2D[y+yMin][x+xMin] & 0xFF));		 // LSB
				result.push_back(static_cast<uint8_t>((frame->distance2D[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(distanceMat, x+xMin, y+yMin, nsl_getDistanceColor(frame->distance2D[y+yMin][x+xMin]));
			}
		}

		imgDistance.header.stamp = data_stamp;
		imgDistance.header.frame_id = "roboscan_frame";
		imgDistance.height = static_cast<uint32_t>(frame->height);
		imgDistance.width = static_cast<uint32_t>(frame->width);
		imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
		imgDistance.step = imgDistance.width * 2;
		imgDistance.is_bigendian = 0;
		imgDistance.data = result;
		imgDistancePub->publish(imgDistance);
	}

	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE)
	{
		sensor_msgs::msg::Image imgAmpl;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;
		
		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->amplitude[y+yMin][x+xMin] & 0xFF));		// LSB
				result.push_back(static_cast<uint8_t>((frame->amplitude[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(amplitudeMat, x+xMin, y+yMin, nsl_getDistanceColor(frame->amplitude[y+yMin][x+xMin]));
			}
		}

		imgAmpl.header.stamp = data_stamp;
		imgAmpl.header.frame_id = "roboscan_frame";
		imgAmpl.height = static_cast<uint32_t>(frame->height);
		imgAmpl.width = static_cast<uint32_t>(frame->width);
		imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
		imgAmpl.step = imgAmpl.width * 2;
		imgAmpl.is_bigendian = 0;
		imgAmpl.data = result;
		imgAmplPub->publish(imgAmpl);
	}	

	
	if(frame->operationMode == OPERATION_MODE_OPTIONS::GRAYSCALE_MODE
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)
	{
		sensor_msgs::msg::Image imgGray;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;
		
		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->amplitude[y+yMin][x+xMin] & 0xFF));		// LSB
				result.push_back(static_cast<uint8_t>((frame->amplitude[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(amplitudeMat, x+xMin, y+yMin, nsl_getDistanceColor(frame->amplitude[y+yMin][x+xMin]));
			}
		}

		imgGray.header.stamp = data_stamp;
		imgGray.header.frame_id = "roboscan_frame";
		imgGray.height = static_cast<uint32_t>(frame->height);
		imgGray.width = static_cast<uint32_t>(frame->width);
		imgGray.encoding = sensor_msgs::image_encodings::MONO16;
		imgGray.step = imgGray.width * 2;
		imgGray.is_bigendian = 0;
		imgGray.data = result;
		imgGrayPub->publish(imgGray);
	}		
	

#ifdef image_transfer_function
	if(frame->operationMode == OPERATION_MODE_OPTIONS::RGB_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)
	{
	
		int totalPixels = NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH;
		cv::Vec3b* dstPtr = rgbMat.ptr<cv::Vec3b>();
		NslOption::NslVec3b* srcPtr = &frame->rgb[0][0];
		
		for (int i = 0; i < totalPixels; ++i) {
			dstPtr[i] = cv::Vec3b(
				srcPtr[i][0],  // blue
				srcPtr[i][1],  // green
				srcPtr[i][2]   // red
			);
		}

		cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		cv_ptr->header.stamp = data_stamp;
		cv_ptr->header.frame_id = "roboscan_frame";
		cv_ptr->image = rgbMat;
		cv_ptr->encoding = "bgr8";
	
		imagePublisher.publish(cv_ptr->toImageMsg());		
	}
#endif

	if( frame->operationMode != OPERATION_MODE_OPTIONS::RGB_MODE
		&& frame->operationMode != OPERATION_MODE_OPTIONS::GRAYSCALE_MODE )
	{
		const size_t nPixel = frame->width * frame->height;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
		cloud->header.frame_id = "roboscan_frame";
		cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
		//cloud->header.stamp = static_cast<uint64_t>(data_stamp.nanoseconds());
		cloud->width = static_cast<uint32_t>(frame->width);
		cloud->height = static_cast<uint32_t>(frame->height);
		cloud->is_dense = false;
		cloud->points.resize(nPixel);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;

		for(int y = 0, index = 0; y < frame->height; y++)
		{
			for(int x = 0; x < frame->width; x++, index++)
			{
				pcl::PointXYZI &point = cloud->points[index];

				if( frame->distance3D[OUT_Z][y+yMin][x+xMin] < NSL_LIMIT_FOR_VALID_DATA )
				{
					point.x = (double)(frame->distance3D[OUT_Z][y+yMin][x+xMin]/1000);
					point.y = (double)(-frame->distance3D[OUT_X][y+yMin][x+xMin]/1000);
					point.z = (double)(-frame->distance3D[OUT_Y][y+yMin][x+xMin]/1000);
					point.intensity = frame->amplitude[y+yMin][x+xMin];
				}
				else{
					point.x = std::numeric_limits<float>::quiet_NaN();
					point.y = std::numeric_limits<float>::quiet_NaN();
					point.z = std::numeric_limits<float>::quiet_NaN();
					point.intensity = std::numeric_limits<float>::quiet_NaN();
				}
			}
		}

		
		sensor_msgs::msg::PointCloud2 msg;
		pcl::toROSMsg(*cloud, msg);
		msg.header.stamp = data_stamp;
		msg.header.frame_id = "roboscan_frame";
		pointcloudPub->publish(msg);  
	}
	
	if(viewerParam.cvShow == true)
	{	
		getMouseEvent(mouseXpos, mouseYpos);
			
		if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_MODE ){
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_MODE ){
			resize( rgbMat, rgbMat, Size( 640, 480 ), 0, 0);
			distanceMat = rgbMat;
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE ){
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			hconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			vconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			vconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		
		imshow(winName, distanceMat);
		waitKey(1);
	}
	
}


void roboscanPublisher::getMouseEvent( int &mouse_xpos, int &mouse_ypos )
{
	mouse_xpos = x_start;
	mouse_ypos = y_start;
}

/*
	ubuntu usb device
	
	sudo apt-get install libopencv-dev
	sudo apt-get install libpcl-dev(1.8.1)

	$ sudo vi /etc/udev/rules.d/defined_lidar.rules
	KERNEL=="ttyACM*", ATTRS{idVendor}=="1FC9", ATTRS{idProduct}=="0094", MODE:="0777",SYMLINK+="ttyNsl3140"

	$ service udev reload
	$ service udev restart

	ubuntu Network UDP speed up
	sudo sysctl -w net.core.rmem_max=22020096
	sudo sysctl -w net.core.rmem_default=22020096
*/

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
