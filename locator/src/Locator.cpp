#include "Locator.h"

Locator::Locator(float fieldLength, float fieldWidth) {

    _tagPoints = {
        {9, {30.0, 1.0}},
        {10, {60.0, 1.0}},
        {8, {112.0, 30.0}},
        {0, {112.0, 60.0}},
        {4, {112.0, 90.0}},
        {3, {112.0, 120.0}},
        {6, {90.0, 151.0}},
        {7, {60.0, 151.0}},
        {1, {30.0, 151.0}},
        {13, {0.0, 120.0}},
        {2, {0.0, 90.0}},
        {12, {1.0, 60.0}},
        {11, {1.0, 30.0}}
    };

    _fieldLength = fieldLength * 0.0254;
    _fieldWidth = fieldWidth * 0.0254;

}

void Locator::run(std::vector<Pose> poses) {

    if (poses.size() > 1) {
        if (triangulate(poses))
            _newPos = true;
    } else {
        // std::cout << "Sorry!\n";
    }

}

void Locator::print() {
    std::cout << "Position: (" << _pos.x << ", " << _pos.y << ")\n";
}

bool Locator::triangulate(std::vector<Pose> poses) {

    Pose t1Pose = poses[0];
    Pose t2Pose = poses[1];

    for (int i = 2; i < poses.size(); i++) {
        if (poses[i].getDistance() < t1Pose.getDistance()) {
            t1Pose = poses[i];
        } else if (poses[i].getDistance() < t2Pose.getDistance()) {
            t2Pose = poses[i];
        }
    }
    
    double r1 = t1Pose.getDistance();
    double r2 = t2Pose.getDistance();

    Point pos;

    try {

        Point t1 = _tagPoints.at(t1Pose.getId());
        Point t2 = _tagPoints.at(t2Pose.getId());

        t1 *= 0.0254;
        t2 *= 0.0254;

        double d = sqrt(pow(t1.x - t2.x, 2) + pow(t1.y - t2.y, 2));
        double l = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d);
        double h = sqrt(pow(r1, 2) - pow(l, 2));

        double ld = l / d;
        double hd = h / d;

        double xp1 = ld * (t2.x - t1.x);
        double xp2 = hd * (t2.y - t1.y);
        double xInt1 = xp1 + xp2 + t1.x;
        double xInt2 = xp1 - xp2 + t1.x;
        if (xInt1 > _fieldLength || xInt1 < 0) {
            pos.x = xInt2;
        } else {
            pos.x = xInt1;
        }

        double yp1 = (l / d) * (t2.y - t1.y);
        double yp2 =  (h / d) * (t2.x - t1.x);
        double yInt1 = yp1 + yp2 + t1.y;
        double yInt2 = yp1 - yp2 + t1.y;
        if (yInt1 > _fieldWidth || yInt1 < 0) {
            pos.y = yInt2;
        } else {
            pos.y = yInt1;
        }

        pos.x = abs(pos.x);

        pos *= 39.3701;
        _pos = pos;

        return true;

    } catch(const std::exception& e) {
        std::cout << "Triangulation Error: Position not known for tags " << t1Pose.getId() << " " << t2Pose.getId();
    }
    return false;
}