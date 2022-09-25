
extern "C" {
#include "apriltag_pose.h"
}

#include <math.h>

#define PI 3.1415926535897932384626433832795
#define RAD2DEG 57.29577951308232087679815481410

class Pose {
public:

    Pose(); 
    Pose(apriltag_pose_t pose);

    void print();

private:

    double _x, _y, _z;
    double _roll, _pitch, _yaw;

};