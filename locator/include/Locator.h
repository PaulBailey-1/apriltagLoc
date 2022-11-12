
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

    bool getPosGood() {
        return _posGood;
    }

private:

    std::map<int, Point> _tagPoints;

    Point _pos;
    bool _posGood;

    bool triangulate(std::vector<Pose> poses);

};