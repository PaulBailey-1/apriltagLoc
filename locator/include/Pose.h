
#ifndef POSE_H
#define POSE_H

extern "C" {
#include "apriltag_pose.h"
}

#include <math.h>

#define PI 3.1415926535897932384626433832795
#define RAD2DEG 57.29577951308232087679815481410

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

private:

    double _x, _y, _z;
    double _roll, _pitch, _yaw;
    double _angle;

    double _distance;

    int _id;

};

#endif
