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
}

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
    cv::Mat euler(3, 1, CV_64F);

    double m00 = rotationMatrix.at<double>(0, 0);
    double m02 = rotationMatrix.at<double>(0, 2);
    double m10 = rotationMatrix.at<double>(1, 0);
    double m11 = rotationMatrix.at<double>(1, 1);
    double m12 = rotationMatrix.at<double>(1, 2);
    double m20 = rotationMatrix.at<double>(2, 0);
    double m22 = rotationMatrix.at<double>(2, 2);

    double x, y, z;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI / 2;
        z = atan2(m02, m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        x = 0;
        y = -CV_PI / 2;
        z = atan2(m02, m22);
    }
    else
    {
        x = atan2(-m12, m11);
        y = asin(m10);
        z = atan2(-m20, m00);
    }

    euler.at<double>(0) = x;
    euler.at<double>(1) = y;
    euler.at<double>(2) = z;

    return euler;
}

int main() {

    std::cout << "Enabling video capture" << std::endl;

    const char* dev_name = "/dev/video0";
    int width = 2560 / 2;
    int height = 1920 / 2;
    int maxFPS = 90;

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

    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
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
            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);
            std::cout << "Translation:" <<std::endl;
            matd_print(pose.t, "%f ");
            std::cout << "Rotation:" <<std::endl;

            cv::Mat rotationMatrix(3, 3, CV_64F);
            for(int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    rotationMatrix.at<float64_t>(row, col) = matd_get(pose.R, row, col);
                }
            }
            cv::Mat eulers(3, 1, CV_64F);
            eulers = rot2euler(rotationMatrix);
            eulers *= 180 / CV_PI;
            std::cout << eulers << std::endl;
        }

        // Draw detection outlines
        int fontface = cv::FONT_HERSHEY_SIMPLEX;
        double fontscale = 1.0;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Scalar(0, 0xff, 0), 2);
            line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0, 0, 0xff), 2);
            line(frame, cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Scalar(0xff, 0, 0), 2);
            line(frame, cv::Point(det->p[2][0], det->p[2][1]),
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

        cv::Mat outFrame;
        cv::resize(frame, outFrame, cv::Size(640, 480));

        static int every = 5;
        if (every == 5) {
            fps = 1 / timer.getTimeSec();
            every = 0;
        }
        every++;

        std::stringstream fpsText; 
        fpsText << "FPS: " << std::fixed << std::setprecision(1) << fps;
        cv::putText(outFrame, fpsText.str(), cv::Point(20, 40), fontface, fontscale, cv::Scalar(0x00, 0x00, 0xff));

        cv::imshow("Tag Detections", outFrame);
        
        timer.reset();
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}
