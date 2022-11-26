#ifndef POINT_H
#define POINT_H

struct Point {

    float x, y;

    Point& operator*=(float mul) {
        Point newPoint;
        x *= mul;
        y *= mul;
        return *this;
    }
};

#endif