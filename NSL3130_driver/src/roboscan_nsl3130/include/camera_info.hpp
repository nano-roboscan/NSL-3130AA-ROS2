#ifndef __ROBOSCAN_CAMERAINFO_H__
#define __ROBOSCAN_CAMERAINFO_H__

#include <cstdint>

namespace nanosys {

    struct CameraInfo {
        uint16_t frame_id;
        uint16_t width;
        uint16_t height;
        uint16_t roiX0;
        uint16_t roiY0;
        uint16_t roiX1;
        uint16_t roiY1;
    };

} //end namespace nanosys

#endif // __ROBOSCAN_CAMERAINFO_H__
