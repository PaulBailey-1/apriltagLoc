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
#include <iostream>
#include <fstream>
#include <thread>

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
    int width = 1280;
    int height = 800;

    int widthDisplay = 640;
    int heightDisplay = 400;

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Couldn't open video capture device" << std::endl;
        return -1;
    }

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = 4;
    td->debug = false;
    td->refine_edges = true;

    std::cout << "Detector tag36h11 initialized\n";

    std::cout << width << "x" << height << std::endl;

    apriltag_detection_info_t info;

    info.tagsize = 0.149;
    info.fx = 1120.556787989777;
    info.fy = 1125.442975043206;
    info.cx = 634.3519601327893;
    info.cy = 406.9630521749576;

    cv::Mat frame, grey;
    cv::TickMeter timer;
    float fps = 0.0;
    int every = 0;
    double sum = 0.0;
    const int fpsDisplay = 10;

    int imageIdx = 0;
    std::fstream idxFile;
    std::string idxFilePath = "./images/idx";
    idxFile.open(idxFilePath, std::ios::in);
    if (idxFile.is_open()) {
        idxFile >> imageIdx;
    }
    idxFile.close();

    bool running = false;
    double actual = 0.0;
    std::string data = "";

    while (true) {
        timer.start();
        errno = 0;

        cap >> frame;
        cvtColor(frame, grey, cv::COLOR_BGR2GRAY);

        image_u8_t im = { 
            .width = grey.cols,
            .height = grey.rows,
            .stride = grey.cols,
            .buf = grey.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);
        // zarray_t *detections = zarray_create(1);

        int k = cv::pollKey();
        if (k == 27) { // esc
            break;
        } 
        if (k == 32 && detections->size > 0) { // space
            zarray_get(detections, 0, &info.det);
            apriltag_pose_t apPose;
            double err = estimate_tag_pose(&info, &apPose);
            Pose pose(apPose);
            pose.print();
        }

        if (running) {

            double measured = -1.0;
            if (zarray_size(detections) > 0) {
                zarray_get(detections, 0, &info.det);
                apriltag_pose_t apPose;
                double err = estimate_tag_pose(&info, &apPose);
                Pose pose(apPose);
                measured = pose.getDistance();
            }
            double error = abs(actual - measured) * 100;
            data += std::to_string(measured) + ", " + std::to_string(error) + ", ";
            std::cout << td->quad_decimate << " decimation - " << error << " cm error" << std::endl;

            if (td->quad_decimate < 2) {
                td->quad_decimate += 0.5;
            } else if (td->quad_decimate < 9) {
                td->quad_decimate += 1;
            } else {
                running = false;
                data += "\n";
                td->quad_decimate = 1;
                std::cout << "Finished Test\n\n";
            }
        }

        if (k == 111) { // o
            std::ofstream dataFile;
            dataFile.open("data.csv");
            dataFile << data;
            dataFile.close();
            std::cout << "Output data to data.csv\n";
        }

        if (k == 114) { // r
            std::cout << "Running Tests. Input actual distance in inches:\n";
            running = true;
            td->quad_decimate = 1.0;
            double actualIn = 0.0;
            std::cin.clear();
            std::cin >> actualIn;
            actual = actualIn * 0.0254;
            data += std::to_string(actualIn) + ", " + std::to_string(actual) + ", ";
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
        every++;
        if (every == fpsDisplay) {
            fps = fpsDisplay / sum;
            every = 0;
            sum = 0.0;
        }

        if (getopt_get_int(getopt, "show") == 1) {

            cv::Mat outFrame;
            cv::resize(frame, outFrame, cv::Size(widthDisplay, heightDisplay));

            std::stringstream fpsText; 
            fpsText << "FPS: " << std::fixed << std::setprecision(1) << fps;
            cv::putText(outFrame, fpsText.str(), cv::Point(20, 40), fontface, fontscale, cv::Scalar(0x00, 0x00, 0xff));

            const int crossSize = 10;
            cv::line(outFrame, cv::Point(widthDisplay / 2 - crossSize, heightDisplay / 2), cv::Point(widthDisplay / 2 + crossSize, heightDisplay / 2), cv::Scalar(0, 0, 0xff));
            cv::line(outFrame, cv::Point(widthDisplay / 2, heightDisplay / 2 - crossSize), cv::Point(widthDisplay / 2, heightDisplay / 2 + crossSize), cv::Scalar(0, 0, 0xff));

            cv::imshow("Tag Detections", outFrame);

            if (k == 115) { // s
                std::string imageName = "./images/image" + std::to_string(imageIdx) + ".jpg";
                cv::imwrite(imageName, frame);
                std::cout << "Saved image to " << imageName << std::endl;
                imageIdx++;
                idxFile.open(idxFilePath, std::ofstream::out | std::ofstream::trunc);
                idxFile << imageIdx;
                idxFile.close();
            }

        } else if (every == 0) {
            std::cout << "FPS: " << std::fixed << std::setprecision(1) << fps << std::endl;
        }
        
        timer.reset();
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return 0;
}
