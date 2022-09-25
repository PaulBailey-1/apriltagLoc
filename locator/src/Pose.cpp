#include "Pose.h"

Pose::Pose(apriltag_pose_t pose) {

    _x = matd_get(pose.t, 0, 0);
    _y = matd_get(pose.t, 1, 0);
    _z = matd_get(pose.t, 2, 0);

    double m00 = matd_get(pose.R, 0, 0);
    double m02 = matd_get(pose.R, 0, 2);
    double m10 = matd_get(pose.R, 1, 0);
    double m11 = matd_get(pose.R, 1, 1);
    double m12 = matd_get(pose.R, 1, 2);
    double m20 = matd_get(pose.R, 2, 0);
    double m22 = matd_get(pose.R, 2, 2);

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        _roll = 0;
        _pitch = PI / 2;
        _yaw = atan2(m02, m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        _roll = 0;
        _pitch = -PI / 2;
        _yaw = atan2(m02, m22);
    }
    else
    {
        _roll = atan2(-m12, m11);
        _pitch = asin(m10);
        _yaw = atan2(-m20, m00);
    }

    _roll *= RAD2DEG;
    _pitch *= RAD2DEG;
    _yaw *= RAD2DEG;

}

void Pose::print() {
    printf("Translation:\nX: %f\nY: %f\nZ: %f\n", _x, _y, _z);
    printf("Rotation:\nRoll: %f\nPitch: %f\nYaw: %f\n", _roll, _yaw, _pitch);
}