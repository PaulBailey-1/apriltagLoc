
#include <map>
#include <vector>
#include <iostream>

#include "Point.h"
#include "Pose.h"

class Locator {
public:

    Locator();

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

    Point _pos;
    bool _newPos;

    bool triangulate(std::vector<Pose> poses);

};