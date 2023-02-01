#include "cartesian_transform.hpp"
#include <stdint.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

namespace nanosys{

CartesianTransform::CartesianTransform(){

}

CartesianTransform::~CartesianTransform(){

}

void CartesianTransform::initLensDistortionTable(LensType lensType)
{
  
  if(lensType == LensType::LENS50)
  {
        // 50 
      distortionTableSize = 46;
        

        angle[  0   ] =     0   ;
        angle[  1   ] =     0.775   ;
        angle[  2   ] =     1.55    ;
        angle[  3   ] =     2.325   ;
        angle[  4   ] =     3.1 ;
        angle[  5   ] =     3.875   ;
        angle[  6   ] =     4.65    ;
        angle[  7   ] =     5.425   ;
        angle[  8   ] =     6.2 ;
        angle[  9   ] =     6.975   ;
        angle[  10  ] =     7.75    ;
        angle[  11  ] =     8.525   ;
        angle[  12  ] =     9.3 ;
        angle[  13  ] =     10.075  ;
        angle[  14  ] =     10.85   ;
        angle[  15  ] =     11.625  ;
        angle[  16  ] =     12.4    ;
        angle[  17  ] =     13.175  ;
        angle[  18  ] =     13.95   ;
        angle[  19  ] =     14.725  ;
        angle[  20  ] =     15.5    ;
        angle[  21  ] =     16.275  ;
        angle[  22  ] =     17.05   ;
        angle[  23  ] =     17.825  ;
        angle[  24  ] =     18.6    ;
        angle[  25  ] =     19.375  ;
        angle[  26  ] =     20.15   ;
        angle[  27  ] =     20.925  ;
        angle[  28  ] =     21.7    ;
        angle[  29  ] =     22.475  ;
        angle[  30  ] =     23.25   ;
        angle[  31  ] =     24.025  ;
        angle[  32  ] =     24.8    ;
        angle[  33  ] =     25.575  ;
        angle[  34  ] =     26.35   ;
        angle[  35  ] =     27.125  ;
        angle[  36  ] =     27.9    ;
        angle[  37  ] =     28.675  ;
        angle[  38  ] =     29.45   ;
        angle[  39  ] =     30.225  ;
        angle[  40  ] =     31  ;
        angle[  41  ] =     31.775  ;
        angle[  42  ] =     32.55   ;
        angle[  43  ] =     33.325  ;
        angle[  44  ] =     34.1    ;
        angle[  45  ] =     34.875  ;



        //size mm
        rp[ 0   ] =     0   ;
        rp[ 1   ] =     0.1 ;
        rp[ 2   ] =     0.2 ;
        rp[ 3   ] =     0.3 ;
        rp[ 4   ] =     0.4 ;
        rp[ 5   ] =     0.5 ;
        rp[ 6   ] =     0.6 ;
        rp[ 7   ] =     0.7 ;
        rp[ 8   ] =     0.8 ;
        rp[ 9   ] =     0.9 ;
        rp[ 10  ] =     1   ;
        rp[ 11  ] =     1.1 ;
        rp[ 12  ] =     1.2 ;
        rp[ 13  ] =     1.3 ;
        rp[ 14  ] =     1.4 ;
        rp[ 15  ] =     1.5 ;
        rp[ 16  ] =     1.6 ;
        rp[ 17  ] =     1.7 ;
        rp[ 18  ] =     1.8 ;
        rp[ 19  ] =     1.9 ;
        rp[ 20  ] =     2   ;
        rp[ 21  ] =     2.1 ;
        rp[ 22  ] =     2.2 ;
        rp[ 23  ] =     2.3 ;
        rp[ 24  ] =     2.4 ;
        rp[ 25  ] =     2.5 ;
        rp[ 26  ] =     2.6 ;
        rp[ 27  ] =     2.7 ;
        rp[ 28  ] =     2.8 ;
        rp[ 29  ] =     2.9 ;
        rp[ 30  ] =     3   ;
        rp[ 31  ] =     3.1 ;
        rp[ 32  ] =     3.2 ;
        rp[ 33  ] =     3.3 ;
        rp[ 34  ] =     3.4 ;
        rp[ 35  ] =     3.5 ;
        rp[ 36  ] =     3.6 ;
        rp[ 37  ] =     3.7 ;
        rp[ 38  ] =     3.8 ;
        rp[ 39  ] =     3.9 ;
        rp[ 40  ] =     4   ;
        rp[ 41  ] =     4.1 ;
        rp[ 42  ] =     4.2 ;
        rp[ 43  ] =     4.3 ;
        rp[ 44  ] =     4.4 ;
        rp[ 45  ] =     4.5 ;
    }
    else if(lensType == LensType::LENS90)
    {
        // 90 
        distortionTableSize = 46;
        

        angle[  0   ] =     0   ;
        angle[  1   ] =     1.391769226 ;
        angle[  2   ] =     2.783538452 ;
        angle[  3   ] =     4.175307678 ;
        angle[  4   ] =     5.567076905 ;
        angle[  5   ] =     6.958846131 ;
        angle[  6   ] =     8.350615357 ;
        angle[  7   ] =     9.742384583 ;
        angle[  8   ] =     11.13415381 ;
        angle[  9   ] =     12.52592304 ;
        angle[  10  ] =     13.91769226 ;
        angle[  11  ] =     15.30946149 ;
        angle[  12  ] =     16.70123071 ;
        angle[  13  ] =     18.09299994 ;
        angle[  14  ] =     19.48476917 ;
        angle[  15  ] =     20.87653839 ;
        angle[  16  ] =     22.26830762 ;
        angle[  17  ] =     23.66007684 ;
        angle[  18  ] =     25.05184607 ;
        angle[  19  ] =     26.4436153  ;
        angle[  20  ] =     27.83538452 ;
        angle[  21  ] =     29.22715375 ;
        angle[  22  ] =     30.61892298 ;
        angle[  23  ] =     32.0106922  ;
        angle[  24  ] =     33.40246143 ;
        angle[  25  ] =     34.79423065 ;
        angle[  26  ] =     36.18599988 ;
        angle[  27  ] =     37.57776911 ;
        angle[  28  ] =     38.96953833 ;
        angle[  29  ] =     40.36130756 ;
        angle[  30  ] =     41.75307678 ;
        angle[  31  ] =     43.14484601 ;
        angle[  32  ] =     44.53661524 ;
        angle[  33  ] =     45.92838446 ;
        angle[  34  ] =     47.32015369 ;
        angle[  35  ] =     48.71192292 ;
        angle[  36  ] =     50.10369214 ;
        angle[  37  ] =     51.49546137 ;
        angle[  38  ] =     52.88723059 ;
        angle[  39  ] =     54.27899982 ;
        angle[  40  ] =     55.67076905 ;
        angle[  41  ] =     57.06253827 ;
        angle[  42  ] =     58.4543075  ;
        angle[  43  ] =     59.84607672 ;
        angle[  44  ] =     61.23784595 ;
        angle[  45  ] =     62.62961518 ;

        //size mm
        rp[ 0   ] =     0   ;
        rp[ 1   ] =     0.1 ;
        rp[ 2   ] =     0.2 ;
        rp[ 3   ] =     0.3 ;
        rp[ 4   ] =     0.4 ;
        rp[ 5   ] =     0.5 ;
        rp[ 6   ] =     0.6 ;
        rp[ 7   ] =     0.7 ;
        rp[ 8   ] =     0.8 ;
        rp[ 9   ] =     0.9 ;
        rp[ 10  ] =     1   ;
        rp[ 11  ] =     1.1 ;
        rp[ 12  ] =     1.2 ;
        rp[ 13  ] =     1.3 ;
        rp[ 14  ] =     1.4 ;
        rp[ 15  ] =     1.5 ;
        rp[ 16  ] =     1.6 ;
        rp[ 17  ] =     1.7 ;
        rp[ 18  ] =     1.8 ;
        rp[ 19  ] =     1.9 ;
        rp[ 20  ] =     2   ;
        rp[ 21  ] =     2.1 ;
        rp[ 22  ] =     2.2 ;
        rp[ 23  ] =     2.3 ;
        rp[ 24  ] =     2.4 ;
        rp[ 25  ] =     2.5 ;
        rp[ 26  ] =     2.6 ;
        rp[ 27  ] =     2.7 ;
        rp[ 28  ] =     2.8 ;
        rp[ 29  ] =     2.9 ;
        rp[ 30  ] =     3   ;
        rp[ 31  ] =     3.1 ;
        rp[ 32  ] =     3.2 ;
        rp[ 33  ] =     3.3 ;
        rp[ 34  ] =     3.4 ;
        rp[ 35  ] =     3.5 ;
        rp[ 36  ] =     3.6 ;
        rp[ 37  ] =     3.7 ;
        rp[ 38  ] =     3.8 ;
        rp[ 39  ] =     3.9 ;
        rp[ 40  ] =     4   ;
        rp[ 41  ] =     4.1 ;
        rp[ 42  ] =     4.2 ;
        rp[ 43  ] =     4.3 ;
        rp[ 44  ] =     4.4 ;
        rp[ 45  ] =     4.5 ;
    }
    else if(lensType == LensType::LENS110)
    {
        // 110
        distortionTableSize = 46;
        

        angle[  0   ] =     0   ;
        angle[  1   ] =     1.677055033 ;
        angle[  2   ] =     3.354110067 ;
        angle[  3   ] =     5.0311651   ;
        angle[  4   ] =     6.708220134 ;
        angle[  5   ] =     8.385275167 ;
        angle[  6   ] =     10.0623302  ;
        angle[  7   ] =     11.73938523 ;
        angle[  8   ] =     13.41644027 ;
        angle[  9   ] =     15.0934953  ;
        angle[  10  ] =     16.77055033 ;
        angle[  11  ] =     18.44760537 ;
        angle[  12  ] =     20.1246604  ;
        angle[  13  ] =     21.80171543 ;
        angle[  14  ] =     23.47877047 ;
        angle[  15  ] =     25.1558255  ;
        angle[  16  ] =     26.83288053 ;
        angle[  17  ] =     28.50993557 ;
        angle[  18  ] =     30.1869906  ;
        angle[  19  ] =     31.86404563 ;
        angle[  20  ] =     33.54110067 ;
        angle[  21  ] =     35.2181557  ;
        angle[  22  ] =     36.89521074 ;
        angle[  23  ] =     38.57226577 ;
        angle[  24  ] =     40.2493208  ;
        angle[  25  ] =     41.92637584 ;
        angle[  26  ] =     43.60343087 ;
        angle[  27  ] =     45.2804859  ;
        angle[  28  ] =     46.95754094 ;
        angle[  29  ] =     48.63459597 ;
        angle[  30  ] =     50.311651   ;
        angle[  31  ] =     51.98870604 ;
        angle[  32  ] =     53.66576107 ;
        angle[  33  ] =     55.3428161  ;
        angle[  34  ] =     57.01987114 ;
        angle[  35  ] =     58.69692617 ;
        angle[  36  ] =     60.3739812  ;
        angle[  37  ] =     62.05103624 ;
        angle[  38  ] =     63.72809127 ;
        angle[  39  ] =     65.4051463  ;
        angle[  40  ] =     67.08220134 ;
        angle[  41  ] =     68.75925637 ;
        angle[  42  ] =     70.4363114  ;
        angle[  43  ] =     72.11336644 ;
        angle[  44  ] =     73.79042147 ;
        angle[  45  ] =     75.4674765  ;


        //size mm
        rp[ 0   ] =     0   ;
        rp[ 1   ] =     0.1 ;
        rp[ 2   ] =     0.2 ;
        rp[ 3   ] =     0.3 ;
        rp[ 4   ] =     0.4 ;
        rp[ 5   ] =     0.5 ;
        rp[ 6   ] =     0.6 ;
        rp[ 7   ] =     0.7 ;
        rp[ 8   ] =     0.8 ;
        rp[ 9   ] =     0.9 ;
        rp[ 10  ] =     1   ;
        rp[ 11  ] =     1.1 ;
        rp[ 12  ] =     1.2 ;
        rp[ 13  ] =     1.3 ;
        rp[ 14  ] =     1.4 ;
        rp[ 15  ] =     1.5 ;
        rp[ 16  ] =     1.6 ;
        rp[ 17  ] =     1.7 ;
        rp[ 18  ] =     1.8 ;
        rp[ 19  ] =     1.9 ;
        rp[ 20  ] =     2   ;
        rp[ 21  ] =     2.1 ;
        rp[ 22  ] =     2.2 ;
        rp[ 23  ] =     2.3 ;
        rp[ 24  ] =     2.4 ;
        rp[ 25  ] =     2.5 ;
        rp[ 26  ] =     2.6 ;
        rp[ 27  ] =     2.7 ;
        rp[ 28  ] =     2.8 ;
        rp[ 29  ] =     2.9 ;
        rp[ 30  ] =     3   ;
        rp[ 31  ] =     3.1 ;
        rp[ 32  ] =     3.2 ;
        rp[ 33  ] =     3.3 ;
        rp[ 34  ] =     3.4 ;
        rp[ 35  ] =     3.5 ;
        rp[ 36  ] =     3.6 ;
        rp[ 37  ] =     3.7 ;
        rp[ 38  ] =     3.8 ;
        rp[ 39  ] =     3.9 ;
        rp[ 40  ] =     4   ;
        rp[ 41  ] =     4.1 ;
        rp[ 42  ] =     4.2 ;
        rp[ 43  ] =     4.3 ;
        rp[ 44  ] =     4.4 ;
        rp[ 45  ] =     4.5 ;
    }
    else
    {
        // 90 agree - psd (940nm, glass window)
        distortionTableSize = 46;
        

        angle[  0   ] =     0   ;
        angle[  1   ] =     1.391769226 ;
        angle[  2   ] =     2.783538452 ;
        angle[  3   ] =     4.175307678 ;
        angle[  4   ] =     5.567076905 ;
        angle[  5   ] =     6.958846131 ;
        angle[  6   ] =     8.350615357 ;
        angle[  7   ] =     9.742384583 ;
        angle[  8   ] =     11.13415381 ;
        angle[  9   ] =     12.52592304 ;
        angle[  10  ] =     13.91769226 ;
        angle[  11  ] =     15.30946149 ;
        angle[  12  ] =     16.70123071 ;
        angle[  13  ] =     18.09299994 ;
        angle[  14  ] =     19.48476917 ;
        angle[  15  ] =     20.87653839 ;
        angle[  16  ] =     22.26830762 ;
        angle[  17  ] =     23.66007684 ;
        angle[  18  ] =     25.05184607 ;
        angle[  19  ] =     26.4436153  ;
        angle[  20  ] =     27.83538452 ;
        angle[  21  ] =     29.22715375 ;
        angle[  22  ] =     30.61892298 ;
        angle[  23  ] =     32.0106922  ;
        angle[  24  ] =     33.40246143 ;
        angle[  25  ] =     34.79423065 ;
        angle[  26  ] =     36.18599988 ;
        angle[  27  ] =     37.57776911 ;
        angle[  28  ] =     38.96953833 ;
        angle[  29  ] =     40.36130756 ;
        angle[  30  ] =     41.75307678 ;
        angle[  31  ] =     43.14484601 ;
        angle[  32  ] =     44.53661524 ;
        angle[  33  ] =     45.92838446 ;
        angle[  34  ] =     47.32015369 ;
        angle[  35  ] =     48.71192292 ;
        angle[  36  ] =     50.10369214 ;
        angle[  37  ] =     51.49546137 ;
        angle[  38  ] =     52.88723059 ;
        angle[  39  ] =     54.27899982 ;
        angle[  40  ] =     55.67076905 ;
        angle[  41  ] =     57.06253827 ;
        angle[  42  ] =     58.4543075  ;
        angle[  43  ] =     59.84607672 ;
        angle[  44  ] =     61.23784595 ;
        angle[  45  ] =     62.62961518 ;

        //size mm
        rp[ 0   ] =     0   ;
        rp[ 1   ] =     0.1 ;
        rp[ 2   ] =     0.2 ;
        rp[ 3   ] =     0.3 ;
        rp[ 4   ] =     0.4 ;
        rp[ 5   ] =     0.5 ;
        rp[ 6   ] =     0.6 ;
        rp[ 7   ] =     0.7 ;
        rp[ 8   ] =     0.8 ;
        rp[ 9   ] =     0.9 ;
        rp[ 10  ] =     1   ;
        rp[ 11  ] =     1.1 ;
        rp[ 12  ] =     1.2 ;
        rp[ 13  ] =     1.3 ;
        rp[ 14  ] =     1.4 ;
        rp[ 15  ] =     1.5 ;
        rp[ 16  ] =     1.6 ;
        rp[ 17  ] =     1.7 ;
        rp[ 18  ] =     1.8 ;
        rp[ 19  ] =     1.9 ;
        rp[ 20  ] =     2   ;
        rp[ 21  ] =     2.1 ;
        rp[ 22  ] =     2.2 ;
        rp[ 23  ] =     2.3 ;
        rp[ 24  ] =     2.4 ;
        rp[ 25  ] =     2.5 ;
        rp[ 26  ] =     2.6 ;
        rp[ 27  ] =     2.7 ;
        rp[ 28  ] =     2.8 ;
        rp[ 29  ] =     2.9 ;
        rp[ 30  ] =     3   ;
        rp[ 31  ] =     3.1 ;
        rp[ 32  ] =     3.2 ;
        rp[ 33  ] =     3.3 ;
        rp[ 34  ] =     3.4 ;
        rp[ 35  ] =     3.5 ;
        rp[ 36  ] =     3.6 ;
        rp[ 37  ] =     3.7 ;
        rp[ 38  ] =     3.8 ;
        rp[ 39  ] =     3.9 ;
        rp[ 40  ] =     4   ;
        rp[ 41  ] =     4.1 ;
        rp[ 42  ] =     4.2 ;
        rp[ 43  ] =     4.3 ;
        rp[ 44  ] =     4.4 ;
        rp[ 45  ] =     4.5 ;
    }
}

double CartesianTransform::interpolate(double x_in, double x0, double y0, double x1, double y1)
{
    if(fabs(x1 - x0) < std::numeric_limits<double>::epsilon())  return y0;
    else return ((x_in-x0)*(y1-y0)/(x1-x0) + y0);
}

double CartesianTransform::getAngle(double x, double y, double sensorPointSizeMM)
{
    double radius = sensorPointSizeMM * sqrt(x*x + y*y);
    double alfaGrad = 0;

    for(int i=1; i < distortionTableSize; i++)
    {
        if(radius >= rp[i-1] && radius <= rp[i]){

            alfaGrad = interpolate(radius, rp[i-1], angle[i-1], rp[i], angle[i]);
        }
    }

    return alfaGrad;
}



void CartesianTransform::initLensTransform(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY, int lensType)
{
    int x, y, row, col;
    numCols = width;
    numRows = height;

    initLensDistortionTable(static_cast<LensType>(lensType));

    int r0 = 1 - numRows/2 + offsetY; //lens optical center offset
    int c0 = 1 - numCols/2 + offsetX;

    for(y=0, row = r0; y < numRows; row++, y++){
        for(x=0, col = c0; x < numCols; col++, x++){

            double c = col - 0.5;
            double r = row - 0.5;

            double angleGrad = getAngle(c, r, sensorPointSizeMM);
            double angleRad =  angleGrad * 3.14159265 / 180.0;

            double rp = sqrt(c * c + r * r);
            double rUA = sin(angleRad);

            xUA[x][y] = c * rUA / rp;
            yUA[x][y] = r * rUA / rp;
            zUA[x][y] = cos(angleRad);
        }
    }

}


// function for cartesian transfrmation
void CartesianTransform::transformPixel(uint srcX, uint srcY, double srcZ, double &destX, double &destY, double &destZ)
{
    destX = srcZ * xUA[srcX][srcY];
    destY = srcZ * yUA[srcX][srcY];
    destZ = srcZ * zUA[srcX][srcY];
}



} //end namespace nanosys




