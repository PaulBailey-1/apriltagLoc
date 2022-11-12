#ifndef POINT_H
#define POINT_H

struct Point {

    float x, y;

    Point operator*=(float mul) {
        Point newPoint;
        newPoint.x = x * mul;
        newPoint.y = y * mul;
        return newPoint;
    }
};

#endif