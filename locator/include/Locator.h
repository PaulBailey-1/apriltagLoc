
#include <map>
#include <vector>
#include <iostream>

#include "Point.h"
#include "Pose.h"

class Locator {
public:

    Locator(float fieldLength, float fieldWidth);

    void run(std::vector<Pose> poses);
    void print();

    Point getPos() {
        return _pos;
    }

    bool newPos() {
        if (_newPos) {
            _newPos = false;
            return true;
        }
        return false;
    }

private:

    std::map<int, Point> _tagPoints;

    Point _pos = {0.0, 0.0};
    bool _newPos;

    float _fieldWidth;
    float _fieldLength;

    bool triangulate(std::vector<Pose> poses);

};