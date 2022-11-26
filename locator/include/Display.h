
extern "C" {
    #include "apriltag.h"
}

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <fstream>

class Display {
public:

    Display();

    void drawDetections(zarray_t *detections);
    void showFrame();
    void saveFrame();

    void setFrame(cv::Mat* frame) {
        _frame = frame;
    }

    void setDims(int width, int height) {
        _width = width;
        _height = height;
    }

    void setFps(float fps) {
        _fps = fps;
    }

private:

    const int fontface = cv::FONT_HERSHEY_SIMPLEX;
    const double fontscale = 1.0;

    cv::Mat* _frame;

    float _fps;

    int _width;
    int _height;

    int _imageIdx;

};