#include "Pose.h"

Pose::Pose(apriltag_pose_t pose, int id) {

    _id = id;

    _x = matd_get(pose.t, 0, 0);
    _y = matd_get(pose.t, 1, 0);
    _z = matd_get(pose.t, 2, 0);

    _distance = sqrt(pow(_x, 2) + pow(_y, 2) + pow(_z, 2));

    double m00 = matd_get(pose.R, 0, 0);
    double m10 = matd_get(pose.R, 1, 0);
    double m20 = matd_get(pose.R, 2, 0);
    double m21 = matd_get(pose.R, 2, 1);
    double m22 = matd_get(pose.R, 2, 2);

    double cosPitch = sqrt(pow(m00, 2) + pow(m10, 2));

    if (!cosPitch < 1e-6) {

        _roll = atan2(m21, m22);
        _pitch = atan2(-m20, cosPitch);
        _yaw = atan2(m10, m00);

    } else {

        _roll = atan2(m21, m22);
        _pitch = atan2(-m20, cosPitch);
        _yaw = 0.0;

    }

    _roll *= RAD2DEG;
    _pitch *= RAD2DEG;
    _yaw *= RAD2DEG;

}

void Pose::print() {
    printf("Translation-\nX: %f\nY: %f\nZ: %f\nTotal Distance: %f\n", _x, _y, _z, _distance);
    printf("Rotation-\nYaw: %f\nPitch: %f\nRoll: %f\n", _yaw, _pitch, _roll);
}

void Pose::printIn() {
    printf("Translation-\nX: %f\nY: %f\nZ: %f\nTotal Distance: %f\n", _x * 39.3701, _y * 39.3701, _z * 39.3701, _distance * 39.3701);
    printf("Rotation-\nYaw: %f\nPitch: %f\nRoll: %f\n", _yaw, _pitch, _roll);
}