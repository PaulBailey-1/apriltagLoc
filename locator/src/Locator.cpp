#include "Locator.h"

Locator::Locator() {

    _tagPoints = {
        {6, {0.0, 0.0}},
        {8, {74 * 0.0254, 0.0}}
    };

    _pos = {};

}

void Locator::run(std::vector<Pose> poses) {

    if (poses.size() > 1) {
        _posGood = triangulate(poses);
    } else {
        std::cout << "Sorry!\n";
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
    
    float r1 = t1Pose.getDistance();
    float r2 = t2Pose.getDistance();

    Point pos;

    try {

        Point t1 = _tagPoints.at(t1Pose.getId());
        Point t2 = _tagPoints.at(t2Pose.getId());

        float d = sqrt(pow(t1.x - t2.x, 2) + pow(t1.y - t1.y, 2));
        float l = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d);
        float h = sqrt(pow(r1, 2) - pow(l, 2));

        pos.x = (l / d) * (t2.x - t1.x) + (h / d) * (t2.y - t1.y) + t1.x;
        pos.y = (l / d) * (t2.y - t1.y) - (h / d) * (t2.x - t1.x) + t1.y;

        pos *= 39.3701;
        _pos = pos;


        return true;

    } catch(const std::exception& e) {
        std::cout << "Triangulation Error: Position not known for tags\n";
    }
    return false;
}