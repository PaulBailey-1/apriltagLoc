#include <iostream>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <fstream>
#include <string_view>

#include "opencv2/opencv.hpp"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include <networktables/DoubleTopic.h>

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/getopt.h"
}

#include "Display.h"
#include "Detector.h"
#include "Locator.h"
#include "Pose.h"
#include "Stream.h"

int main(int argc, char *argv[]) {

    Display* display = nullptr;
    Detector* detector = nullptr;
    Locator* locator = nullptr;
    Stream* stream = nullptr;

    getopt_t* getopt = getopt_create();

    getopt_add_double(getopt, 'd', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_int(getopt, 's', "show", "1", "Show video stream");
    getopt_add_int(getopt, 'r', "rotate", "0", "Rotates camera feed [0-3]");
    getopt_add_int(getopt, 'c', "camera", "2", "0 - picamera, 1 - OV9281, 2 - D435");
    getopt_add_int(getopt, 'h', "depthMap", "0", "Show depth map");
    getopt_add_int(getopt, 'o', "stream", "1", "Stream to virtual camera");

    if (!getopt_parse(getopt, argc, argv, 1)) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    if (getopt_get_int(getopt, "show") == 1) {
        display = new Display();
    }

    if (getopt_get_int(getopt, "stream") == 1) {
        stream = new Stream();
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

    detector = new Detector(width, height, getopt_get_int(getopt, "rotate"), (Detector::Camera) getopt_get_int(getopt, "camera"), getopt_get_double(getopt, "decimate"), getopt_get_double(getopt, "blur"), getopt_get_int(getopt, "depthMap"));

    locator = new Locator(112, 150);

    cv::TickMeter timer;
    float fps = 0.0;
    int every = 0;
    double sum = 0.0;
    const int fpsDisplay = 10;

    bool logging = false;
    std::string log = "";
    bool takeMean = false;
    int meanCounter;
    Point meanPos;

    int heartbeat = 0;

    // nt::NetworkTableInstance serverInst = nt::NetworkTableInstance::Create();
    // serverInst.StartServer("networktables.json", "169.254.4.2");
    // serverInst.StartServer("networktables.json", "192.168.1.200");

    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> ntCam = inst.GetTable("ATCamera");
    ntCam->PutNumber("xPos", 0.0);
    ntCam->PutNumber("yPos", 0.0);
    ntCam->PutNumber("z", 0.0);
    ntCam->PutNumber("x", 0.0);
    ntCam->PutNumber("angle", 0.0);
    ntCam->PutNumber("distance", 0.0);
    ntCam->PutBoolean("valid", false);
    ntCam->PutBoolean("stero", false);
    ntCam->PutNumber("id", 0);
    ntCam->PutNumber("heartbeat", 0);


    inst.StartClient3("Pi");
    inst.SetServer("10.99.18.2", 1735);
    // inst.SetServer("169.254.4.2", 1735);
    // inst.SetServer("192.168.1.200", 1735);

    while (true) {
        timer.start();

        detector->run();
        if (detector->getDetectionsSize() > 0) {
            locator->run(detector->getPoses(), 0.0);
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
                    logFile << "FPS, Detections, PosX, PosY\n";
                    logFile << log;
                    logFile.close();
                    std::cout << "Saved log\n";
                }
                break;
            case 'm':
                if (takeMean) {
                    takeMean = false;
                    meanPos /= meanCounter;
                    std::cout << "Mean Pos - ";
                    meanPos.print();
                } else {
                    meanPos = {0, 0};
                    meanCounter = 0;
                    takeMean = true;
                    std::cout << "Taking mean\n";
                }
                
                break;
            default:
                break;
        }

        if (locator->newPos()) {

            if (takeMean) {
                meanPos += locator->getPos();;
                meanCounter++;
            }

            Point pos = locator->getPos();
            Pose tag = locator->getTagPose();
            ntCam->PutNumber("z", tag.getZin());
            ntCam->PutNumber("x", tag.getXin());
            ntCam->PutNumber("xPos", pos.x);
            ntCam->PutNumber("yPos", pos.y);
            ntCam->PutNumber("angle", tag.getAngle());
            ntCam->PutNumber("distance", tag.getDistance() * 39.3701);
            ntCam->PutBoolean("valid", true);
            ntCam->PutBoolean("stero", tag.getStero());
	        ntCam->PutNumber("id", tag.getId());
	        ntCam->PutNumber("heartbeat", heartbeat);
            inst.Flush();

	        heartbeat++;

            if (logging) {
                log += std::to_string(fps) + ", " + 
                std::to_string(detector->getDetectionsSize()) + ", " + 
                std::to_string(pos.x) + ", " + 
                std::to_string(pos.y) + "\n";
            }
        }

        if (display != nullptr) {
            display->setFrame(detector->getFrame());
            display->drawDetections(detector->getDetections());
        }

        if (stream != nullptr) {
            stream->writeFrame(detector->getFrame());
        }

        detector->destroyDetections();

        timer.stop();

        sum += timer.getTimeSec();
        every++;
        if (every == fpsDisplay) {
            fps = fpsDisplay / sum;
            if (display != nullptr) {display->setFps(fps);}
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
