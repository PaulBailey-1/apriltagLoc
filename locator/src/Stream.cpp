
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <iostream>

#include "Stream.h"

Stream::Stream() {

    // open video device
    _outputDevice = open("/dev/video6", O_RDWR);
    if(_outputDevice < 0) {
        std::cerr << "ERROR: could not open output device!\n" <<
        strerror(errno);
    }

    // acquire video format from device
    struct v4l2_format vid_format;
    memset(&vid_format, 0, sizeof(vid_format));
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

    if (ioctl(_outputDevice, VIDIOC_G_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to get video format!\n" <<
        strerror(errno);
    }

    // configure desired video format on device
    _frameSize = VIDEO_WIDTH * VIDEO_HEIGHT * 3;
    vid_format.fmt.pix.width = VIDEO_WIDTH;
    vid_format.fmt.pix.height = VIDEO_HEIGHT;
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    vid_format.fmt.pix.sizeimage = _frameSize;
    vid_format.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(_outputDevice, VIDIOC_S_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to set video format!\n" <<
        strerror(errno);
    }

}

void Stream::writeFrame(cv::Mat* frame) {

    size_t written = write(_outputDevice, frame->data, _frameSize);
    if (written < 0) {
        std::cerr << "ERROR: could not write to output device!\n";
        close(_outputDevice);
    }
}
