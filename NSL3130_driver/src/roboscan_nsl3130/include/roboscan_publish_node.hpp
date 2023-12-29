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

#define image_transfer_function

#ifdef image_transfer_function
#include <image_transport/image_transport.h>
#endif

namespace nanosys {

	struct SetParameter {
	
        uint16_t imageType;
        uint16_t lensType;
        uint16_t old_lensType;
        
		bool startStream;
		bool publishPointCloud;
		bool cartesian;
		uint16_t channel;
		uint16_t frequencyModulation;
		uint16_t int0, int1, int2, intGr; //integration times
		uint16_t hdr_mode; //0 - hdr off, 1 - hdr spatial, 2 - hdr temporal
		uint16_t minAmplitude;
		uint16_t lensCenterOffsetX = 0;
		uint16_t lensCenterOffsetY = 0;
		uint16_t old_lensCenterOffsetX = 0;
		uint16_t old_lensCenterOffsetY = 0;
		uint16_t modIndex = 0;

		uint16_t roi_leftX = 0;
		uint16_t roi_topY = 0;
		uint16_t roi_rightX = 319;
		uint16_t roi_bottomY = 239;

		bool grayscaleIlluminationMode = false;
		uint8_t bAdcOverflow = 1;
		uint8_t bSaturation = 1;
		double transformAngle;
		uint8_t cutPixels;

		bool medianFilter;
        bool averageFilter;
       	double temporalFilterFactor ;
        uint16_t temporalFilterThreshold;        
        uint16_t edgeFilterThreshold;
        uint16_t temporalEdgeThresholdLow;
		uint16_t temporalEdgeThresholdHigh;
		uint16_t interferenceDetectionLimit;
		bool useLastValue = false;

		uint32_t frameSeq;
        uint16_t dualBeam;        
		bool cvShow = false;
		bool changedCvShow = true;
		bool pointCloudEdgeFilter = false;

		std::string	ipAddr;
		std::string	netMask;
		std::string	gwAddr;
		bool changedIpInfo = false;

		int maxDistance;
    };

   	typedef struct _RGB888Pixel
	{
		unsigned char r;
		unsigned char g;
		unsigned char b;
	} RGB888Pixel;


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

		void thread_callback();
		double interpolate( double x, double x0, double y0, double x1, double y1);
		void createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
		int setDistanceColor(cv::Mat &imageLidar, int x, int y, int value );
		void setReconfigure();
		void publishFrame(Frame *frame);
		void updateFrame(Frame *frame);
		bool edgeDetection(Triple const& v0, Triple const& v1, Triple const& v2, double threshold, double distanceLimit);
		void setGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range );
		void setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value, double end_range );
		void startStreaming();

		boost::signals2::connection connectionFrames;
		boost::signals2::connection connectionCameraInfo;

		std::vector<cv::Vec3b> colorVector;
		

		Interface interface;
		CartesianTransform cartesianTransform;
		sensor_msgs::msg::CameraInfo cameraInfo;


		//static rclcpp::Time timeNow;

		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgDistancePub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgAmplPub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgGrayPub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgDCSPub;	
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPub;

#ifdef image_transfer_function
		rclcpp::Node::SharedPtr nodeHandle;
		image_transport::ImageTransport imageTransport;
		image_transport::Publisher imagePublisher;
#endif
		int Convert_To_RGB24( float fValue, RGB888Pixel &nRGBData, float fMinValue, float fMaxValue);

	    SetParameter lidarParam;

		std::vector<Frame *> backupFrame;
		boost::scoped_ptr<boost::thread> publisherThread;
		bool runThread;
	private:
		void initialise();
		void setParameters();
		void updateCameraInfo(std::shared_ptr<CameraInfo> ci);
		bool setCameraInfo(sensor_msgs::srv::SetCameraInfo::Request& req, sensor_msgs::srv::SetCameraInfo::Response& res);
		void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
		cv::Mat addDistanceInfo(cv::Mat distMat, Frame *frame);
		cv::Mat addDCSInfo(cv::Mat distMat, Frame *frame);
		void setWinName();
		OnSetParametersCallbackHandle::SharedPtr callback_handle_;
		rcl_interfaces::msg::SetParametersResult parametersCallback( const std::vector<rclcpp::Parameter> &parameters);
		int mouseXpos, mouseYpos;
		bool reconfigure;
		char winName[100];
	};


} //end namespace nanosys
