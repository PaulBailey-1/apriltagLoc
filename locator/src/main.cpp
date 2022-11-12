#include <iostream>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <fstream>

#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "common/getopt.h"
}

#include "Display.h"
#include "Detector.h"
#include "Locator.h"

int main(int argc, char *argv[]) {

    Display* display = nullptr;
    Detector* detector = nullptr;
    Locator* locator = nullptr;

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
        display = new Display();
    }

    int width;
    int height;

    int widthDisplay = 640;
    int heightDisplay = 480;

    if (getopt_get_int(getopt, "rotate") == 1 || getopt_get_int(getopt, "rotate") == 3) {
        int temp = width;
        width = height;
        height = temp;

        temp = widthDisplay;
        widthDisplay = heightDisplay;
        heightDisplay = temp;
    }

    if (display != nullptr) {
        display->setDims(widthDisplay, heightDisplay);
    }

    detector = new Detector(width, height, getopt_get_int(getopt, "rotate"), (Detector::Camera) getopt_get_int(getopt, "camera"), getopt_get_double(getopt, "decimate"), getopt_get_double(getopt, "blur"));

    locator = new Locator();

    cv::TickMeter timer;
    float fps = 0.0;
    int every = 0;
    double sum = 0.0;
    const int fpsDisplay = 10;

    bool logging = false;
    std::string log = "";

    while (true) {
        timer.start();

        detector->run();
        if (detector->getDetectionsSize() > 0) {
            locator->run(detector->getPoses());
        }

        int k = cv::pollKey();
        if (k == 27) { // esc
            break;
        }

        switch (k) {
            case ' ':
                detector->printPoses();
                break;
            case 'r':
                detector->runTest();
                break;
            case 'o':
                detector->saveData();
                break;
            case 't':
                locator->print();
                break;
            case 's':
                if(display != nullptr) display->saveFrame();
                break;
            case 'l':
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
                break;
            
            default:
                break;
        }

        if (logging) {
            log += std::to_string(fps) + ", " + std::to_string(detector->getDetectionsSize()) + "\n";
        }

        if (display != nullptr) {
            display->setFrame(detector->getFrame());
            display->drawDetections(detector->getDetections());
        }

        detector->destroyDetections();

        timer.stop();

        sum += timer.getTimeSec();
        every++;
        if (every == fpsDisplay) {
            fps = fpsDisplay / sum;
            every = 0;
            sum = 0.0;
        }

        if (display != nullptr) {

            display->showFrame();

        } else if (every == 0) {
            std::cout << "FPS: " << std::fixed << std::setprecision(1) << fps << std::endl;
        }
        
        timer.reset();
    }

    return 0;
}
