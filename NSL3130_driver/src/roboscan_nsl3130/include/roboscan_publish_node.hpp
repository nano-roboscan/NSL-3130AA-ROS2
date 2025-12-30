#include <cstdio>
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
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nanolib.h"

#define image_transfer_function

#ifdef image_transfer_function
#include <image_transport/image_transport.hpp>
#endif

namespace nanosys {

	struct ViewerParameter {
		int	frameCount;
		int maxDistance;
		int pointCloudEdgeThreshold;
		int imageType;
		int lensType;
		double lidarAngle;
		
		bool cvShow;
		bool changedCvShow;
		bool changedImageType;
		bool reOpenLidar;
		bool saveParam;
		
		std::string	frame_id;
		std::string	ipAddr;
		std::string	netMask;
		std::string	gwAddr;
    };


	class roboscanPublisher : public rclcpp::Node { 

	public:
		roboscanPublisher();
		~roboscanPublisher();

		void initialise();
		void threadCallback();
		void setReconfigure();
		void publishFrame(NslPCD *frame, NslOption::NslVec3b *rgbframe);
		void startStreaming();

		//static rclcpp::Time timeNow;

		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgDistancePub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgAmplPub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgGrayPub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPub;

#ifdef image_transfer_function
		rclcpp::Node::SharedPtr nodeHandle;
		image_transport::ImageTransport imageTransport;
		image_transport::Publisher imagePublisher;
#endif
		ViewerParameter viewerParam;

		boost::scoped_ptr<boost::thread> publisherThread;
		bool runThread;
		NslConfig 		nslConfig;
		int 			nsl_handle;
	private:
		std::string yaml_path_;
	    const std::unordered_map<int, std::string> modeIntMap = {
	        {1, "DISTANCE"},
	        {2, "GRAYSCALE"},
			{3, "DISTANCE_AMPLITUDE"},
	        {4, "DISTANCE_GRAYSCALE"},
	        {5, "RGB"},
	        {6, "RGB_DISTANCE"},
	        {7, "RGB_DISTANCE_AMPLITUDE"},
	        {8, "RGB_DISTANCE_GRAYSCALE"}
	    };

		const std::unordered_map<int, std::string> lensIntMap = {
		    {0, "LENS_NF"},
		    {1, "LENS_SF"},
			{2, "LENS_WF"}
		};

		const std::unordered_map<int, std::string> hdrIntMap = {
		    {0, "HDR_None"},
		    {1, "HDR_Spatial"},
			{2, "HDR_Temporal"}
		};		

		const std::unordered_map<int, std::string> modulationIntMap = {
		    {0, "MOD_12Mhz"},
		    {1, "MOD_24Mhz"},
			{2, "MOD_6Mhz"},
			{3, "MOD_3Mhz"}
		};		

		const std::unordered_map<int, std::string> DBIntMap = {
		    {0, "DB_Off"},
		    {1, "DB_6Mhz"},
			{2, "DB_3Mhz"}
		};		

		const std::unordered_map<int, std::string> DBOptIntMap = {
		    {0, "DB_AVOIDANCE"},
		    {1, "DB_CORRECTION"},
			{2, "DB_FULL_CORRECTION"}
		};		


		const std::unordered_map<std::string, int> modeStrMap = {
			{"DISTANCE", 1},
			{"GRAYSCALE", 2},
			{"DISTANCE_AMPLITUDE", 3},
			{"DISTANCE_GRAYSCALE", 4},
			{"RGB", 5},
			{"RGB_DISTANCE", 6},
			{"RGB_DISTANCE_AMPLITUDE", 7},
			{"RGB_DISTANCE_GRAYSCALE", 8}
		};
		
		const std::unordered_map<std::string, int> lensStrMap = {
			{"LENS_NF", 0},
			{"LENS_SF", 1},
			{"LENS_WF", 2},
		};

		const std::unordered_map<std::string, int> hdrStrMap = {
		    {"HDR_None", 0},
		    {"HDR_Spatial", 1},
			{"HDR_Temporal", 2}
		};		

		const std::unordered_map<std::string, int> modulationStrMap = {
		    {"MOD_12Mhz", 0},
		    {"MOD_24Mhz", 1},
			{"MOD_6Mhz", 2},
			{"MOD_3Mhz", 3}
		};		

		const std::unordered_map<std::string, int> DBStrMap = {
		    {"DB_Off", 0},
		    {"DB_6Mhz", 1},
			{"DB_3Mhz", 2}
		};		

		const std::unordered_map<std::string, int> DBOptStrMap = {
		    {"DB_AVOIDANCE", 0},
		    {"DB_CORRECTION", 1},
			{"DB_FULL_CORRECTION", 2}
		};		

		
		// load yaml
		void load_params()
		{
			
			RCLCPP_INFO(this->get_logger(),"Loaded params: path=%s\n", yaml_path_.c_str());
			
			if (std::ifstream(yaml_path_))
			{
				YAML::Node config = YAML::LoadFile(yaml_path_);
				viewerParam.ipAddr = config["IP Addr"] ? config["IP Addr"].as<std::string>() : "192.168.0.220";
				viewerParam.frame_id = config["FrameID"] ? config["FrameID"].as<std::string>() : "roboscan_frame";
				viewerParam.maxDistance = config["MaxDistance"] ? config["MaxDistance"].as<int>() : 12500;
				viewerParam.pointCloudEdgeThreshold = config["PointColud EDGE"] ? config["PointColud EDGE"].as<int>() : 200;
				std::string tmpModeStr = config["ImageType"] ? config["ImageType"].as<std::string>() : "DISTANCE_AMPLITUDE";
				auto itMode = modeStrMap.find(tmpModeStr);
				viewerParam.imageType = (itMode != modeStrMap.end()) ? itMode->second : 3; // defeault DISTANCE_AMPLITUDE

				std::string tmpLensStr = config["LensType"] ? config["LensType"].as<std::string>() : "LENS_SF";
				auto itLens = lensStrMap.find(tmpLensStr);
				viewerParam.lensType = (itLens != lensStrMap.end()) ? itLens->second : 1; // defeault LENS_SF
				
				viewerParam.lidarAngle = config["LidarAngle"] ? config["LidarAngle"].as<double>() : 0;

				RCLCPP_INFO(this->get_logger(),"Loaded params: ip=%s, frame_id=%s, max = %d, edge = %d, imgType = %d, lensType = %d, Angle = %.2f\n", viewerParam.ipAddr.c_str(), viewerParam.frame_id.c_str(), viewerParam.maxDistance, viewerParam.pointCloudEdgeThreshold, viewerParam.imageType, viewerParam.lensType, viewerParam.lidarAngle);
			}
			else{
				RCLCPP_INFO(this->get_logger(),"Not found params: ip=%s, frame_id=%s, max = %d, edge = %d, imgType = %d, lensType = %d, Angle = %.2f\n", viewerParam.ipAddr.c_str(), viewerParam.frame_id.c_str(), viewerParam.maxDistance, viewerParam.pointCloudEdgeThreshold, viewerParam.imageType, viewerParam.lensType, viewerParam.lidarAngle);
			}
		}

	    // save yaml
	    void save_params()
	    {
	        std::ofstream fout(yaml_path_);
	        fout << "IP Addr: " << this->get_parameter("0. IP Addr").as_string() << "\n";
	        fout << "FrameID: " << this->get_parameter("Q. frameID").as_string() << "\n";
	        fout << "MaxDistance: " << this->get_parameter("Z. MaxDistance").as_int() << "\n";
	        fout << "PointColud EDGE: " << this->get_parameter("Y. PointColud EDGE").as_int() << "\n";
			fout << "ImageType: " << this->get_parameter("C. imageType").as_string() << "\n";
			fout << "LensType: " << this->get_parameter("B. lensType").as_string() << "\n";
	        fout << "LidarAngle: " << this->get_parameter("P. transformAngle").as_double() << "\n";

	        fout.close();
	        RCLCPP_INFO(this->get_logger(), "Params saved to %s", yaml_path_.c_str());
	    }
		
		void initNslLibrary();
		void setMatrixColor(cv::Mat image, int x, int y, NslOption::NslVec3b color);
		void timeDelay(int milli);
		void renewParameter();
		void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
		cv::Mat addDistanceInfo(cv::Mat distMat, NslPCD *frame);
		void setWinName();
		void paramDump(const std::string & filename);
		void paramLoad();
		rcl_interfaces::msg::ParameterDescriptor create_Slider(const std::string &description,int from, int to, int step);
		rcl_interfaces::msg::ParameterDescriptor create_Slider(const std::string &description, double from, double to, double step);

		OnSetParametersCallbackHandle::SharedPtr callback_handle_;
		rcl_interfaces::msg::SetParametersResult parametersCallback( const std::vector<rclcpp::Parameter> &parameters);
		int mouseXpos, mouseYpos;
		bool reconfigure;
		char winName[100];
	};


} //end namespace nanosys
