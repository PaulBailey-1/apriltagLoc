#include <iostream>
#include <iomanip>
#include <string>
#include <libv4l2.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "common/getopt.h"
}

#include "Pose.h"

int main(int argc, char *argv[]) {

    getopt_t* getopt = getopt_create();

    getopt_add_int(getopt, 'r', "resolution", "4", "Divide base resolution by this factor");
    getopt_add_double(getopt, 'd', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_int(getopt, 's', "show", "1", "Show video stream");

    if (!getopt_parse(getopt, argc, argv, 1)) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    std::cout << "Enabling video capture" << std::endl;

    const char* dev_name = "/dev/video0";
    int width = 2560 / getopt_get_int(getopt, "resolution");
    int height = 1920 / getopt_get_int(getopt, "resolution");
    int maxFPS = 90;

    int widthDisplay = 640;
    int heightDisplay = 480;

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Couldn't open video capture device" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, maxFPS);

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = 4;
    td->debug = false;
    td->refine_edges = true;

    std::cout << "Detector tag36h11 initialized\n";

    std::cout << width << "x" <<
        height << " @" <<
        maxFPS << "FPS" << std::endl;

    apriltag_detection_info_t info;
    info.tagsize = 0.149;
    info.fx = 571.8367321574847;
    info.fy = 572.8478769322065;
    info.cx = 308.4643624352032;
    info.cy = 248.84989598432;

    cv::Mat frame, gray;
    cv::TickMeter timer;
    float fps = 0.0;
    int every = 10;
    double sum = 0.0;

    while (true) {
        timer.start();
        errno = 0;
        cap >> frame;
        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        image_u8_t im = { 
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        int k = cv::pollKey();
        if (k == 27) {
            break;
        } 
        if (k == 32 && detections->size > 0) {
            zarray_get(detections, 0, &info.det);
            apriltag_pose_t apPose;
            double err = estimate_tag_pose(&info, &apPose);
            Pose pose(apPose);
            pose.print();
        }

        int fontface = cv::FONT_HERSHEY_SIMPLEX;
        double fontscale = 1.0;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Scalar(0, 0xff, 0), 2);
            cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0, 0, 0xff), 2);
            cv::line(frame, cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Scalar(0xff, 0, 0), 2);
            cv::line(frame, cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0xff, 0, 0), 2);

            std::stringstream ss;
            ss << det->id;
            std::string text = ss.str();
            
            int baseline;
            cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            cv::putText(frame, text, cv::Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
        }
        apriltag_detections_destroy(detections);
        timer.stop();

        sum += timer.getTimeSec();
        if (every == 10) {
            fps = 10 / sum;
            every = 0;
            sum = 0.0;
        }
        every++;

        if (getopt_get_bool(getopt, "show")) {

            cv::Mat outFrame;
            cv::resize(frame, outFrame, cv::Size(widthDisplay, heightDisplay));

            std::stringstream fpsText; 
            fpsText << "FPS: " << std::fixed << std::setprecision(1) << fps;
            cv::putText(outFrame, fpsText.str(), cv::Point(20, 40), fontface, fontscale, cv::Scalar(0x00, 0x00, 0xff));

            const int crossSize = 10;
            cv::line(outFrame, cv::Point(widthDisplay / 2 - crossSize, heightDisplay / 2), cv::Point(widthDisplay / 2 + crossSize, heightDisplay / 2), cv::Scalar(0, 0, 0xff));
            cv::line(outFrame, cv::Point(widthDisplay / 2, heightDisplay / 2 - crossSize), cv::Point(widthDisplay / 2, heightDisplay / 2 + crossSize), cv::Scalar(0, 0, 0xff));

            cv::imshow("Tag Detections", outFrame);

        } else if (every == 10) {
            std::cout << "FPS: " << std::fixed << std::setprecision(1) << fps << std::endl;
        }
        
        timer.reset();
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}
