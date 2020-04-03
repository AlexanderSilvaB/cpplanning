#ifndef _CPPLANNING_COMMOM_HPP_
#define _CPPLANNING_COMMOM_HPP_

class CPPAlgorithm;

enum ShapeType
{
    ROBOT, OBSTACLE
};

typedef struct
{
    ShapeType type;
    float x, y, theta;
    float vs, vt;
    float gx, gy, gtheta;
    CPPAlgorithm *alg;
}Shape;

#define PI 3.141592653589793f
#define Deg2Rad(d) (d*0.017453293f)
#define Rad2Deg(r) (r*57.295779513f)

#endif
