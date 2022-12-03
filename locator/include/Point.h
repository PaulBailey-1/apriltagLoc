#ifndef POINT_H
#define POINT_H

struct Point {

    double x, y;

    Point& operator*=(double mul) {
        Point newPoint;
        x *= mul;
        y *= mul;
        return *this;
    }

    Point& operator/=(double mul) {
        Point newPoint;
        x /= mul;
        y /= mul;
        return *this;
    }

    Point& operator+=(Point other) {
        Point newPoint;
        x += other.x;
        y += other.y;
        return *this;
    }

    void print() {
        printf("x: %f y: %f\n", x, y);
    }
};

#endif