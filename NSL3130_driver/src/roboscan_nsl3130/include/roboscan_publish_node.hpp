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
		
		bool cvShow;
		bool changedCvShow;
		bool changedImageType;
		bool changedIpInfo;
		bool reOpenLidar;
		bool saveParam;
		
		std::string	frame_id;
		std::string	ipAddr;
		std::string	netMask;
		std::string	gwAddr;
    };


	class roboscanPublisher : public rclcpp::Node { 

		static const int PIXEL_VALID_DATA = 64000;
		static const int LOW_AMPLITUDE = 64001;
		static const int ADC_OVERFLOW = 64002;
		static const int SATURATION = 64003;
		static const int BAD_PIXEL = 64004;
		static const int INTERFERENCE = 64007;
		static const int EDGE_FILTERED = 64008;

		const int width   = 320;
		const int width2  = 160;
		const int height  = 240;
		const int height2 = 120;
		const double sensorPixelSizeMM = 0.02; //camera sensor pixel size 20x20 um

	public:
		roboscanPublisher();
		~roboscanPublisher();

		void initialise();
		void threadCallback();
		void setReconfigure();
		void publishFrame(NslPCD *frame);
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

		// load yaml
		void load_params()
		{
			RCLCPP_INFO(this->get_logger(),"Loaded params: path=%s\n", yaml_path_.c_str());
			
			if (std::ifstream(yaml_path_))
			{
				// 파일 존재 → 읽기
				YAML::Node config = YAML::LoadFile(yaml_path_);
				viewerParam.ipAddr = config["0. IP Addr"] ? config["0. IP Addr"].as<std::string>() : "192.168.0.220";
				viewerParam.frame_id = config["Q. frameID"] ? config["Q. frameID"].as<std::string>() : "roboscan_frame";
				
				
				RCLCPP_INFO(this->get_logger(),"Loaded params: ip=%s, frame_id=%s\n", viewerParam.ipAddr.c_str(), viewerParam.frame_id.c_str());
			}
			else{
				RCLCPP_INFO(this->get_logger(),"Not found params: ip=%s, frame_id=%s\n", viewerParam.ipAddr.c_str(), viewerParam.frame_id.c_str());
			}
		}

	    // save yaml
	    void save_params()
	    {
	        std::ofstream fout(yaml_path_);
	        fout << "0. IP Addr: " << this->get_parameter("0. IP Addr").as_string() << "\n";
	        fout << "Q. frameID: " << this->get_parameter("Q. frameID").as_string() << "\n";
	        fout.close();
	        RCLCPP_INFO(this->get_logger(), "Params saved to %s", yaml_path_.c_str());
	    }
		
		void initNslLibrary();
		void timeDelay(int milli);
		void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
		cv::Mat addDistanceInfo(cv::Mat distMat, NslPCD *frame);
		void setWinName();
		void paramDump(const std::string & filename);
		void paramLoad();

		OnSetParametersCallbackHandle::SharedPtr callback_handle_;
		rcl_interfaces::msg::SetParametersResult parametersCallback( const std::vector<rclcpp::Parameter> &parameters);
		int mouseXpos, mouseYpos;
		bool reconfigure;
		char winName[100];
	};


} //end namespace nanosys
