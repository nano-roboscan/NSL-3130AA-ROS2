#ifndef __ROBOSCAN_CARTESIAN_TRANSFORM_H__
#define __ROBOSCAN_CARTESIAN_TRANSFORM_H__

#include <cstdint>
#include <vector>
#include <math.h>
#include <float.h>

namespace nanosys {

typedef unsigned int uint;

inline double Min_(double a, double b)
{
	return (a<b) ? a : b;
}

inline bool isPracticallyZero(double a, double b = 0)
{
	if (!b)
		return (fabs(a) <=  DBL_MIN);	

	return (fabs (a - b) <= Min_(fabs(a), fabs(b))*DBL_EPSILON);	
}

inline bool pcFilter(int dist1, int dist2, int dist3, int threshold)
{
    if(abs(dist1-dist2)>threshold || abs(dist1-dist3)>threshold || abs(dist2-dist3)>threshold)
        return false;

    return true;
}


/**
Consider Triples also as vectors in R^3
*/
struct Triple
{
    //! Initialize Triple with x,y and z
    explicit Triple(double xv = 0,double yv = 0,double zv = 0)
        : x(xv), y(yv), z(zv)
    {
    }

    //! Triple coordinates
    double x,y,z;

    Triple& operator+=(Triple t)
    {
        x += t.x;
        y += t.y;
        z += t.z;

        return *this;
    }

    Triple& operator-=(Triple t)
    {
        x -= t.x;
        y -= t.y;
        z -= t.z;

        return *this;
    }
    Triple& operator*=(double d)
    {
        x *= d;
        y *= d;
        z *= d;

        return *this;
    }
    Triple& operator/=(double d)
    {
        x /= d;
        y /= d;
        z /= d;

        return *this;
    }
    Triple& operator*=(Triple t) // scale
    {
        x *= t.x;
        y *= t.y;
        z *= t.z;

        return *this;
    }

    bool operator!=(Triple t) const
    {
        return !isPracticallyZero(x,t.x) || !isPracticallyZero(y,t.y) || !isPracticallyZero(z,t.z);
    }

    bool operator==(Triple t) const
    {
        return !operator!=(t);
    }

    double length() const
    {
        double l2 = x*x + y*y + z*z;
        return (isPracticallyZero(l2)) ? 0 :sqrt(l2);
    }

    void normalize()
    {
        double l = length();
        if (l)
            *this /= l;
    }
};

 
inline double dotProduct(Triple const& u, Triple const& v)
{
	return u.x*v.x + u.y*v.y + u.z*v.z;
}
 
inline Triple normalizedcross(Triple const& u, Triple const& v)
{
	Triple n;

	/* compute the cross product (u x v for right-handed [ccw]) */
	n.x = u.y * v.z - u.z * v.y;
	n.y = u.z * v.x - u.x * v.z;
	n.z = u.x * v.y - u.y * v.x;

	/* normalize */
	double l = n.length();
	if (l)
	{
		n /= l;
	}
	else
	{
		n = Triple(0,0,0);
	}

	return n;
}

inline const Triple operator+(const Triple& t, const Triple& t2)
{
    return Triple(t) += t2;
}
inline const Triple operator-(const Triple& t, const Triple& t2)
{
    return Triple(t) -= t2;
}
inline const Triple operator*(double d, const Triple& t)
{
    return Triple(t) *= d;
}
inline const Triple operator*(const Triple& t, double d)
{
    return Triple(t) *= d;
}
inline const Triple operator/(double d, const Triple& t)
{
    return Triple(t) /= d;
}
inline const Triple operator/(const Triple& t, double d)
{
    return Triple(t) /= d;
}
inline const Triple operator*(const Triple& t, const Triple& t2)
{
    return Triple(t) *= t2;
}



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
