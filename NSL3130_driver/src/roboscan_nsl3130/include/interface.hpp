#ifndef __ROBOSCAN_INTERFACE_H__
#define __ROBOSCAN_INTERFACE_H__

#include <boost/thread.hpp>
#include "frame.hpp"
#include "camera_info.hpp"
#include "tcp_connection.hpp"
#include "udp_server.hpp"

//#define __CLIENT_FILTER__

#define MASK_TEMPORAL_FILTER	0x01
#define MASK_AVERAGE_FILTER		0x02
#define MASK_MEDIAN_FILTER		0x04
#define MASK_EDGE_FILTER		0x08

namespace nanosys {

typedef std::vector<uint8_t> Packet;

class Interface {
public:
  Interface();
  ~Interface();

  void stopStream();  
  void streamDCS();
  void streamGrayscale();
  void streamDistance();
  void streamDistanceAmplitude();
  void streamDistanceGrayscale();
  void streamDistanceAmplitudeGrayscale();
  void setOffset(int32_t offset);
  void setMinAmplitude(uint16_t minAmplitude);
  void setRoi(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1);
  void setIntegrationTime(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t);
  void setHDRMode(uint8_t mode);
  void setModulation(const uint8_t index, const uint8_t channel);
  void setFilter(bool medianFilter, bool averageFilter, uint16_t temporalFactor, uint16_t temporalThreshold, uint16_t edgeThreshold, uint16_t temporalEdgeThresholdLow, uint16_t temporalEdgeThresholdHigh, uint16_t interferenceDetectionLimit, bool interferenceDetectionUseLastValue);
  void setGrayscaleIlluminationMode(uint8_t mode);
  void setAdcOverflowSaturation(uint8_t bAdcOverflow, uint8_t bSaturation);
  void setCompensation(uint8_t bDrnu, uint8_t bTemperature, uint8_t bGrayscale, uint8_t bAmbientLight);
  void setDataIpAddress(uint32_t dataIpaddr);
  void setDevIpAddress(uint32_t ipaddr, uint32_t subnet, uint32_t gwaddr);
  void getChipInfomation();
  void getFirmwareInfomation();
  void getDevIpaddress();
  void getTemperatureInfo();

  boost::signals2::connection subscribeFrame(std::function<void (std::shared_ptr<Frame>)>);
  boost::signals2::connection subscribeCameraInfo(std::function<void (std::shared_ptr<CameraInfo>)>);
  std::shared_ptr<CameraInfo> getCameraInfo(const Packet &);

private:
  // OPCODE ...
  const static uint16_t COMMAND_SET_ROI = 0;
  const static uint16_t COMMAND_SET_INT_TIMES = 1;
  const static uint16_t COMMAND_GET_DIST_AND_AMP = 2;
  const static uint16_t COMMAND_GET_DISTANCE = 3;
  const static uint16_t COMMAND_GET_GRAYSCALE = 5;
  const static uint16_t COMMAND_STOP_STREAM = 6;
  const static uint16_t COMMAND_GET_DCS = 7;
  const static uint16_t COMMAND_GET_DIST_AND_GRY = 8;
  const static uint16_t COMMAND_GET_DISTANCE_AMPLITUDE_GRAYSCALE = 9;
  const static uint16_t COMMAND_SET_ADC_OVERFLOW = 10;
  const static uint16_t COMMAND_SET_OFFSET = 20;
  const static uint16_t COMMAND_SET_MIN_AMPLITUDE = 21;
  const static uint16_t COMMAND_SET_FILTER = 22;
  const static uint16_t COMMAND_SET_MODULATION = 23;
  const static uint16_t COMMAND_SET_HDR = 25;
  const static uint16_t COMMAND_SET_COMPENSATION = 28;
  const static uint16_t COMMAND_GET_CHIP_INFORMATION = 36;
  const static uint16_t COMMAND_GET_FIRMWARE_INFORMATION = 37;
  const static uint16_t COMMAND_SET_DATA_IP_ADDRESS = 38;
  const static uint16_t COMMAND_SET_GRAYSCALE_ILLUMINATION = 39;
  const static uint16_t COMMAND_SET_CAMERA_IP_SETTINGS = 40;
  const static uint16_t COMMAND_GET_CAMERA_IP_ADDRESS = 48;
  const static uint16_t COMMAND_GET_TEMPERATURE = 52;




  std::shared_ptr<Frame> currentFrame[2];
  boost::asio::io_service ioService;
  boost::scoped_ptr<boost::thread> serverThread;
  boost::signals2::signal<void (std::shared_ptr<Frame>)> frameReady;
  boost::signals2::signal<void (std::shared_ptr<CameraInfo>)> cameraInfoReady;
  TcpConnection tcpConnection;
  UdpServer udpServer;
  uint8_t isStreaming;
  uint8_t dataType;
  uint64_t currentFrame_id;  
  Packet data;
  int currentFrameIdx;
  uint32_t frameRxCnt;

  uint16_t usedTemporalFactor;
  uint16_t usedTemporalThreshold;
  uint16_t usedEdgeThreshold;
  int32_t edgeDetectX;
  int32_t edgeDetectY;

  uint32_t filterSelector;					///<Variable containing filter flags

  void setDataType(uint8_t);  
  void streamMeasurement(uint8_t);
  void insertValue8(std::vector<uint8_t> &output, const int8_t value);
  void insertValue8(std::vector<uint8_t> &output, const uint8_t value);
  void insertValue(std::vector<uint8_t> &output, const int16_t value);
  void insertValue(std::vector<uint8_t> &output, const uint16_t value);
  uint8_t boolToUint8(const bool value);
  int8_t boolToInt8(const bool value);

};

} //end 

#endif
