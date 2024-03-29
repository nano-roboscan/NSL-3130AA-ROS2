#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "frame.hpp"
#include "interface.hpp"


std::chrono::system_clock::time_point start;
std::chrono::system_clock::time_point end;

namespace nanosys {

Interface::Interface() : tcpConnection(ioService),
    udpServer(ioService),
    isStreaming(0),
    dataType(0),
    currentFrame_id(0),
    data(Packet(25+320*240*4*2)),
    currentFrameIdx(0),
    frameRxCnt(0)
{
	rxFrame = NULL;
    serverThread.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &ioService)));
    udpServer.subscribe([&](const Packet& p) -> void
    {
        uint32_t packetNum  = (p[16] << 24) + (p[17] << 16) + (p[18] << 8) + p[19];
        uint32_t offset = (p[8] << 24) + (p[9] << 16) + (p[10] << 8) + p[11];
        uint16_t payloadSize = (p[6] << 8) + p[7];

		if( !isStreaming ) return;

		if( packetNum == frameRxCnt ){
	        if(packetNum == 0 ) { // new frame
	            uint16_t width  = (p[23] << 8) + p[24];
	            uint16_t height = (p[25] << 8) + p[26];
	            int payloadHeaderOffset = (p[43] << 8) + p[44];

//				start = std::chrono::system_clock::now();
//				std::chrono::milliseconds timeCnt = std::chrono::duration_cast<std::chrono::milliseconds>(start-end);
//				printf("start time = %ld ms\n", timeCnt.count());

	            //currentFrame[currentFrameIdx] = std::shared_ptr<Frame>(new Frame(dataType, currentFrame_id++, width, height, payloadHeaderOffset));
	            rxFrame = new Frame(dataType, currentFrame_id++, width, height, payloadHeaderOffset);
	            memcpy(&data[offset], &p[Frame::UDP_HEADER_OFFSET], payloadSize);
	            cameraInfoReady(getCameraInfo(p));

				frameRxCnt++;

	        }else if( rxFrame != NULL ){
	            uint32_t numPackets = (p[12] << 24) + (p[13] << 16) + (p[14] << 8) + p[15];
	            memcpy(&data[offset], &p[Frame::UDP_HEADER_OFFSET], payloadSize);

//				if( frameRxCnt != packetNum ){
//					printf("error::frameRxCnt = %d packetNum = %d\n", frameRxCnt, packetNum);
//				}

				frameRxCnt++;

	            if (packetNum == numPackets - 1) { //last frame                                
	                rxFrame->sortData(data);  //copy data -> dist, ampl, dcs
	                frameReady(rxFrame);
					currentFrameIdx ^= 1;

//					end = std::chrono::system_clock::now();
//					std::chrono::milliseconds timeCnt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//					printf("end time = %ld ms\n", timeCnt.count());


//					if( frameRxCnt != 220 )		printf("err frameRxCnt = %d\n", frameRxCnt);
					frameRxCnt = 0;
	            }
	        }
		}else{
//			printf("frameRxCnt = %d packetNum = %d\n", frameRxCnt, packetNum);
			frameRxCnt = 0;
		}
    }); //end lambda function
	
	usedTemporalFactor = 0;
	usedTemporalThreshold = 0;
	usedEdgeThreshold = 0;
	filterSelector = 0;
}

Interface::~Interface() {
    stopStream();
    serverThread->interrupt();
    ioService.stop();
}

uint32_t Interface::ip2int(const char * ip)
{
    unsigned v = 0;
    int i;
    const char * start;

    start = ip;
    for (i = 0; i < 4; i++) {
        char c;
        int n = 0;
        while (1) {
            c = * start;
            start++;
            if (c >= '0' && c <= '9') {
                n *= 10;
                n += c - '0';
            }
            else if ((i < 3 && c == '.') || i == 3) {
                break;
            }
            else {
                return 0;
            }
        }
        if (n >= 256) {
            return 0;
        }
		
        v *= 256;
        v += n;
    }
	
    return v;
}

void Interface::stopStream() {
    if (!isStreaming) { return; }

    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_STOP_STREAM;

    insertValue(payload, command);

    tcpConnection.sendCommand(payload);
    isStreaming = 0;
}

void Interface::streamDCS()
{
    setDataType(Frame::DCS);
    streamMeasurement(static_cast<uint8_t>(COMMAND_GET_DCS));
}

void Interface::streamDistanceAmplitude() {
    setDataType(Frame::DISTANCE_AMPLITUDE);
    streamMeasurement(static_cast<uint8_t>(COMMAND_GET_DIST_AND_AMP));
}

void Interface::streamDistance() {
    setDataType(Frame::DISTANCE);
    streamMeasurement(static_cast<uint8_t>(COMMAND_GET_DISTANCE));
}

void Interface::streamGrayscale() {
    setDataType(Frame::GRAYSCALE);
    streamMeasurement(static_cast<uint8_t>(COMMAND_GET_GRAYSCALE));
}

void Interface::streamDistanceGrayscale() {
    setDataType(Frame::DISTANCE_GRAYSCALE);
    streamMeasurement(static_cast<uint8_t>(COMMAND_GET_DIST_AND_GRY));
}


void Interface::streamDistanceAmplitudeGrayscale(){
	setDataType(Frame::DISTANCE_AMPLITUDE_GRAYSCALE);
	streamMeasurement(static_cast<uint8_t>(COMMAND_GET_DISTANCE_AMPLITUDE_GRAYSCALE));
}

void Interface::setOffset(int32_t offset){

    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_OFFSET;

    insertValue(payload, command);

    payload.push_back(static_cast<uint8_t>(offset>>24 & 0xFF));
    payload.push_back(static_cast<uint8_t>(offset>>16 & 0xFF));
    payload.push_back(static_cast<uint8_t>(offset>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(offset>>0 & 0xFF));

    tcpConnection.sendCommand(payload);
}

void Interface::setMinAmplitude(uint16_t minAmplitude){

    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_MIN_AMPLITUDE;

    insertValue(payload, command);

    payload.push_back(static_cast<uint8_t>(minAmplitude>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(minAmplitude>>0 & 0xFF));

    tcpConnection.sendCommand(payload);
}


void Interface::setRoi(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1)
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_ROI;

    insertValue(payload, command);


    payload.push_back(static_cast<uint8_t>(x0>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(x0>>0 & 0xFF));

    payload.push_back(static_cast<uint8_t>(y0>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(y0>>0 & 0xFF));

    payload.push_back(static_cast<uint8_t>(x1>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(x1>>0 & 0xFF));

    payload.push_back(static_cast<uint8_t>(y1>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(y1>>0 & 0xFF));

    tcpConnection.sendCommand(payload);
}


void Interface::setIntegrationTime(uint16_t low, uint16_t mid, uint16_t high, uint16_t gray, bool grayMode)
{
    if(low > 2500)
    {  
        low = 2500;
    }
    if(mid > 2500)  
    {
        mid = 2500;
    }
    if(high > 2500)  
    {
        high = 2500;
    }

	if( grayMode > 0 ){
		if( gray > 10000 ){
			gray /= 10;
		}
	}

    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_INT_TIMES;

    insertValue(payload, command);

    payload.push_back(static_cast<uint8_t>(low>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(low>>0 & 0xFF));

    payload.push_back(static_cast<uint8_t>(mid>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(mid>>0 & 0xFF));

    payload.push_back(static_cast<uint8_t>(high>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(high>>0 & 0xFF));

    payload.push_back(static_cast<uint8_t>(gray>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(gray>>0 & 0xFF));

    tcpConnection.sendCommand(payload);
}


void Interface::setHDRMode(uint8_t mode)
{
	std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_HDR;

    insertValue(payload, command);

    payload.push_back(mode);

    tcpConnection.sendCommand(payload);
}

//add

/*
	0 : �̻�� 
	1 : 6Mhz roll-over
	2 : 3Mhz roll-over
*/
void Interface::setDualBeam(uint8_t mode, bool usedDualbeamDist)
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_DUALBEAM_MODE;
	
	uint8_t usedDualBeamDistance = usedDualbeamDist ? 1 : 0;
	//Insert the 16Bit command
    insertValue(payload, command);
    //insertValue(payload, mode);

    payload.push_back(mode);
    payload.push_back(usedDualBeamDistance);

    tcpConnection.sendCommand(payload);
}




void Interface::setGrayscaleIlluminationMode(bool mode)
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_GRAYSCALE_ILLUMINATION;
	uint8_t modeLed = mode ? 1 : 0;
	
    insertValue(payload, command);
    //insertValue(payload, mode);

    payload.push_back(modeLed);

    tcpConnection.sendCommand(payload);
}

void Interface::setAdcOverflowSaturation(uint8_t bAdcOverflow, uint8_t bSaturation)
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_ADC_OVERFLOW;

    insertValue(payload, command);

    payload.push_back(bAdcOverflow);
    payload.push_back(bSaturation);
    
    tcpConnection.sendCommand(payload);
}

void Interface::setCompensation(uint8_t bDrnu, uint8_t bTemperature, uint8_t bGrayscale, uint8_t bAmbientLight)
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_COMPENSATION;

    insertValue(payload, command);

    payload.push_back(bDrnu);
    payload.push_back(bTemperature);
    payload.push_back(bGrayscale);
    payload.push_back(bAmbientLight);
    
    tcpConnection.sendCommand(payload);
}

void Interface::setDataIpAddress(uint32_t dataIpaddr)
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_DATA_IP_ADDRESS;

    insertValue(payload, command);

    payload.push_back(static_cast<uint8_t>(dataIpaddr>>24 & 0xFF));
    payload.push_back(static_cast<uint8_t>(dataIpaddr>>16 & 0xFF));
    payload.push_back(static_cast<uint8_t>(dataIpaddr>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(dataIpaddr>>0 & 0xFF));

    tcpConnection.sendCommand(payload);
}


void Interface::setIpAddr(std::string ipAddr, std::string netmask, std::string gwAddr)
{
	std::ignore = netmask;
	std::ignore = gwAddr;
	
	tcpConnection.setIpAddr(ipAddr);

//	if(tcpConnection.isConnected() ) setDevIpAddress(ip2int(ipAddr.c_str()), ip2int(netmask.c_str()), ip2int(gwAddr.c_str()));
}


void Interface::setDevIpAddress(uint32_t ipaddr, uint32_t subnet, uint32_t gwaddr)
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_CAMERA_IP_SETTINGS;

    insertValue(payload, command);

    payload.push_back(static_cast<uint8_t>(ipaddr>>24 & 0xFF));
    payload.push_back(static_cast<uint8_t>(ipaddr>>16 & 0xFF));
    payload.push_back(static_cast<uint8_t>(ipaddr>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(ipaddr>>0 & 0xFF));

    payload.push_back(static_cast<uint8_t>(subnet>>24 & 0xFF));
    payload.push_back(static_cast<uint8_t>(subnet>>16 & 0xFF));
    payload.push_back(static_cast<uint8_t>(subnet>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(subnet>>0 & 0xFF));

    payload.push_back(static_cast<uint8_t>(gwaddr>>24 & 0xFF));
    payload.push_back(static_cast<uint8_t>(gwaddr>>16 & 0xFF));
    payload.push_back(static_cast<uint8_t>(gwaddr>>8 & 0xFF));
    payload.push_back(static_cast<uint8_t>(gwaddr>>0 & 0xFF));

    tcpConnection.sendCommand(payload);
}

void Interface::getChipInfomation()
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_GET_CHIP_INFORMATION;

    insertValue(payload, command);
    
    tcpConnection.sendCommand(payload);
}


void Interface::setFilter(bool medianFilter, bool averageFilter, uint16_t temporalFactor, uint16_t temporalThreshold, uint16_t edgeThreshold, uint16_t temporalEdgeThresholdLow, uint16_t temporalEdgeThresholdHigh, uint16_t interferenceDetectionLimit, bool interferenceDetectionUseLastValue)
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_FILTER;

    //Insert the 16Bit command
    insertValue(payload, command);

    //Insert temporal filter factor
    insertValue(payload, temporalFactor);

    //Insert temporal filter threshold
    insertValue(payload, temporalThreshold);

    //Insert median filter
    insertValue8(payload, boolToUint8(medianFilter));

    //Insert average filter
    insertValue8(payload, boolToUint8(averageFilter));

    //Insert edge filter threshold
    insertValue(payload, edgeThreshold);

    //Insert interference detection use last value flag
    insertValue8(payload, boolToUint8(interferenceDetectionUseLastValue));

    //Insert edge filter interference detection limit
    insertValue(payload, interferenceDetectionLimit);

    //Insert edge filter threshold low
    insertValue(payload, temporalEdgeThresholdLow);

    //Insert edge filter threshold high
    insertValue(payload, temporalEdgeThresholdHigh);

    tcpConnection.sendCommand(payload);
}


void Interface::setModulation(const uint8_t index, const uint8_t channel){

    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_MODULATION;

    insertValue(payload, command);

    uint8_t data = static_cast<char>(index);
    payload.push_back(data);

    data = static_cast<char>(channel);
    payload.push_back(data);

    data = 0; //AutoChannel reserved
    payload.push_back(data);

    tcpConnection.sendCommand(payload);
}

void Interface::getFirmwareInfomation()
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_GET_FIRMWARE_INFORMATION;

    insertValue(payload, command);
    
    tcpConnection.sendCommand(payload);
}

void Interface::getDevIpaddress()
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_GET_CAMERA_IP_ADDRESS;

    insertValue(payload, command);
    
    tcpConnection.sendCommand(payload);
}

void Interface::getTemperatureInfo()
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_GET_TEMPERATURE;

    insertValue(payload, command);
    
    tcpConnection.sendCommand(payload);
}


void Interface::insertValue8(std::vector<uint8_t> &output, const uint8_t value){
    output.push_back(static_cast<int8_t>(value));
}

void Interface::insertValue8(std::vector<uint8_t> &output, const int8_t value){
    output.push_back(value);
}

void Interface::insertValue(std::vector<uint8_t> &output, const uint16_t value)
{
    output.push_back(static_cast<int8_t>(value >> 8));
    output.push_back(static_cast<int8_t>(value & 0xFF));
}

void Interface::insertValue(std::vector<uint8_t> &output, const int16_t value)
{
    output.push_back(value >> 8);
    output.push_back(static_cast<int8_t>(value & 0xFF));
}

uint8_t Interface::boolToUint8(const bool value)
{
    if (value)  return 1;
    else        return 0;
}

int8_t Interface::boolToInt8(const bool value)
{
    if(value)   return 1;
    else        return 0;
}

boost::signals2::connection Interface::subscribeFrame(std::function<void (Frame*)> onFrameReady)
{
	return frameReady.connect(onFrameReady);
}

boost::signals2::connection Interface::subscribeCameraInfo(std::function<void (std::shared_ptr<CameraInfo>)> onCameraInfoReady)
{
	return cameraInfoReady.connect(onCameraInfoReady);
}

std::shared_ptr<CameraInfo> Interface::getCameraInfo(const Packet& p) {
    std::shared_ptr<CameraInfo> camInfo(new CameraInfo);

    int offset = 23;
    camInfo->width  = (p[offset+0] << 8) + p[offset+1];
    camInfo->height = (p[offset+2] << 8) + p[offset+3];
    camInfo->roiX0  = (p[offset+4] << 8) + p[offset+5];
    camInfo->roiY0  = (p[offset+6] << 8) + p[offset+7];
    camInfo->roiX1  = (p[offset+8] << 8) + p[offset+9];
    camInfo->roiY1  = (p[offset+10] << 8) + p[offset+11];

    return camInfo;
}

void Interface::setDataType(uint8_t d) {
    dataType = d;
}

void Interface::streamMeasurement(uint8_t cmd) {
    tcpConnection.sendCommand(std::vector<uint8_t>({0x00, cmd, 0x01}));
    isStreaming = 2;
}

} //end namespace nanosys
