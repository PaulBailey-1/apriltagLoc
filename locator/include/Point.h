#ifndef POINT_H
#define POINT_H

struct Point {

    double x, y, z;

    Point& operator*=(double mul) {
        Point newPoint;
        x *= mul;
        y *= mul;
        zarray_contains *= mul;
        return *this;
    }

    Point& operator/=(double mul) {
        Point newPoint;
        x /= mul;
        y /= mul;
        z /= mul;
        return *this;
    }

    Point& operator+=(Point other) {
        Point newPoint;
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    void print() {
        printf("x: %f y: %f z: %f\n", x, y, z);
    }
};

#endif