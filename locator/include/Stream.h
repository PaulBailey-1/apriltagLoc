
#include "opencv2/imgproc.hpp"



class Stream {

public:

    Stream();

    void writeFrame(cv::Mat* frame);

private:

    const int VIDEO_WIDTH = 848, VIDEO_HEIGHT = 480;

    int _outputDevice;

    size_t _frameSize;

};