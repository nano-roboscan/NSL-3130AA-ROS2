#ifndef __ROBOSCAN_CARTESIAN_TRANSFORM_H__
#define __ROBOSCAN_CARTESIAN_TRANSFORM_H__

#include <cstdint>
#include <vector>

namespace nanosys {

typedef unsigned int uint;

class CartesianTransform
{
public:

    enum LensType { LENS50 = 0, LENS90, LENS110 };

    CartesianTransform();
    ~CartesianTransform();    
    void transformPixel(uint srcX, uint srcY, double srcZ, double &destX, double &destY, double &destZ, double &transformAngle);
    void initLensTransform(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY, int lensType);


private:

    double fixAngle;
    double angleR;
    double sin_angle;
    double cos_angle;   

    int distortionTableSize;
    int numCols;
    int numRows;

    double angle[101];
    double rp[101];
    double xUA[320][240];
    double yUA[320][240];
    double zUA[320][240];

    double getAngle(double x, double y, double sensorPointSizeMM);
    double interpolate(double x_in, double x0, double y0, double x1, double y1);
    void initLensDistortionTable(LensType lensType);

};


} //end namespace nanosys

#endif // __ROBOSCAN_CARTESIAN_TRANSFORM_H__
