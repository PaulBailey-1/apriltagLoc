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
#include "Point.h"

int main(int argc, char *argv[]) {

    getopt_t* getopt = getopt_create();

    getopt_add_double(getopt, 'd', "decimate", "4.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_int(getopt, 's', "show", "1", "Show video stream");
    getopt_add_int(getopt, 'r', "rotate", "3", "Rotates camera feed [0-3]");
    getopt_add_int(getopt, 'c', "camera", "1", "0 - picamera, 1 - OV9281");

    if (!getopt_parse(getopt, argc, argv, 1)) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    if (getopt_get_int(getopt, "show") == 1) {
        cv::namedWindow("Tag Detections");
    }


    std::map<int, Point> tagPoints = {
        {6, {0.0, 0.0}},
        {8, {74 * 0.0254, 0.0}}
    };

    std::cout << "Enabling video capture" << std::endl;

    const char* dev_name = "/dev/video0";
    int width;
    int height;

    int widthDisplay = 640;
    int heightDisplay = 480;

    apriltag_detection_info_t info;

    info.tagsize = 0.149;

    switch(getopt_get_int(getopt, "camera")) {
        case 0:
            width = 2560;
            height = 1920;

            info.fx = 571.8367321574847 * 4;
            info.fy = 572.8478769322065 * 4;
            info.cx = 308.4643624352032 * 4;
            info.cy = 248.84989598432 * 4;

            break;
        case 1:
            width = 1280;
            height = 800;

            info.fx = 1120.556787989777;
            info.fy = 1125.442975043206;
            info.cx = 634.3519601327893;
            info.cy = 406.9630521749576;

            break;
        default:
            break;
    }

    if (getopt_get_int(getopt, "rotate") == 1 || getopt_get_int(getopt, "rotate") == 3) {
        int temp = width;
        width = height;
        height = temp;

        temp = widthDisplay;
        widthDisplay = heightDisplay;
        heightDisplay = temp;
    }

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Couldn't open video capture device" << std::endl;
        return -1;
    }

    if (getopt_get_int(getopt, "camera") == 0) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap.set(cv::CAP_PROP_FPS, 60);
    }

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = 4;
    td->debug = false;
    td->refine_edges = true;

    std::cout << "Detector tag36h11 initialized\n";

    std::cout << width << "x" << height << " Decimation: " << getopt_get_double(getopt, "decimate") << std::endl;

    cv::Mat frameRaw, frame, grey, greyDecimated;
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

    std::string data = "";

    bool logging = false;
    std::string log = "";

    while (true) {
        timer.start();
        errno = 0;

        if (getopt_get_int(getopt, "rotate") != 0) {
            cap >> frameRaw;
            cv::rotate(frameRaw, frame, getopt_get_int(getopt, "rotate") - 1);
        } else {
            cap >> frame;
        }

        cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
        // double decimation = getopt_get_double(getopt, "decimate");
        // cv::resize(grey, greyDecimated, cv::Size(), 1 / decimation, 1 / decimation, cv::INTER_NEAREST);

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
            std::cout << "Tag: " << info.det->id << "\n";
            pose.printIn();
        }

        if (k == 111) { // o
            std::ofstream dataFile;
            dataFile.open("data.csv", std::ios_base::app);
            dataFile << data;
            dataFile.close();
            std::cout << "Output data to data.csv\n";
        }

        if (k == 114) { // r
            std::cout << "Running Tests. Input actual distance in inches:\n";
            td->quad_decimate = 1.0;
            std::string in;
            std::cin.clear();
            std::getline(std::cin, in, '\n');
            float actual = std::stof(in) * 0.0254;
            data += in + ", " + std::to_string(actual) + ", ";

            while(td->quad_decimate < 10) {

                detections = apriltag_detector_detect(td, &im);

                double measured = -1.0;
                std::cout << td->quad_decimate << " decimation: ";
                if (zarray_size(detections) > 0) {
                    zarray_get(detections, 0, &info.det);
                    apriltag_pose_t apPose;
                    double err = estimate_tag_pose(&info, &apPose);
                    Pose pose(apPose);
                    measured = pose.getDistance();

                    double error = abs(actual - measured) * 100;
                    data += std::to_string(error) + ", ";
                    std::cout << error << " cm error" << std::endl;
                } else {
                    std::cout  << "-1 cm error" << std::endl;
                }

                if (td->quad_decimate < 2) {
                    td->quad_decimate += 0.5;
                } else {
                    td->quad_decimate += 1;
                }
            }
            data += "\n";
            td->quad_decimate = 1;
            std::cout << "Finished Test\n\n";
        }

        if (k == 116) { // t
            if (zarray_size(detections) < 2) {
                std::cout << "Triangulation Failed: Need at least 2 tags\n";
            } else {
                Point t1;
                Point t2;

                apriltag_detection_t *det1;
                apriltag_detection_t *det2;
                zarray_get(detections, 0, &det1);
                zarray_get(detections, 1, &det2);

                apriltag_pose_t apPose;
                info.det = det1;
                double err = estimate_tag_pose(&info, &apPose);
                Pose t1Pose(apPose);

                info.det = det2;
                err = estimate_tag_pose(&info, &apPose);
                Pose t2Pose(apPose);

                float r1 = t1Pose.getDistance();
                float r2 = t2Pose.getDistance();

                std::cout << "Enter distance between tags in inches: \n";
                std::string in;
                std::cin.clear();
                std::getline(std::cin, in, '\n');
                tagPoints[8].x = 0.0254 * stoi(in);

                try {

                    t1 = tagPoints.at(det1->id);
                    t2 = tagPoints.at(det2->id);

                    float d = sqrt(pow(t1.x - t2.x, 2) + pow(t1.y - t1.y, 2));
                    float l = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d);
                    float h = sqrt(pow(r1, 2) - pow(l, 2));

                    float x = (l / d) * (t2.x - t1.x) + (h / d) * (t2.y - t1.y) + t1.x;
                    float y = (l / d) * (t2.y - t1.y) - (h / d) * (t2.x - t1.x) + t1.y;

                    x *= 39.3701;
                    y *= 39.3701;

                    std::cout << "Position: (" << x << ", " << y << ")\n";

                } catch(const std::exception& e) {
                    std::cout << "Triangulation Error: Position not known for tags\n";
                }
                    
            }
        }

        if (k == 108) { // l
            if (!logging) {
                logging = true;
                std::cout << "Started logging\n";
            } else {
                std::ofstream logFile;
                logFile.open("log.csv", std::ios_base::app);
                logFile << log;
                logFile.close();
                std::cout << "Saved log\n";
            }
        }

        if (logging) {
            log += std::to_string(fps) + ", " + std::to_string(zarray_size(detections)) + "\n";
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
