
#ifndef POSE_H
#define POSE_H

extern "C" {
#include "apriltag_pose.h"
}

#include <math.h>

#define PI 3.1415926535897932384626433832795
#define RAD2DEG 57.29577951308232087679815481410
#define DEG2RAD 0.017453292519943295769236907685

class Pose {
public:

    Pose(); 
    Pose(apriltag_pose_t pose, int id);

    void print();
    void printIn();

    double getDistance() {
        return _distance;
    }

    int getId() {
        return _id;
    }

    double getPitch() {
        return _pitch;
    }

    double getAngle() {
	return _angle;
    }

    double getZin() {
        return _z * 39.3701;
    }

    double getXin() {
        return _x * 39.3701;
    }

private:

    double _x, _y, _z;
    double _roll, _pitch, _yaw;
    double _angle;

    double _distance;

    int _id;

};

#endif
