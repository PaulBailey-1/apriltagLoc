#include "Detector.h"
#include <iostream>

#include "opencv2/imgproc.hpp"

Detector::Detector(int width, int height, int rotation, Camera camera, double decimate, double blur) : 
        _frame(), _frameRaw(), _greyFrame(), _cap(), _detectionInfo(), _poses()
    {

    _rotation = rotation;

    _detections = nullptr;
    _tagFamily = nullptr;
    _tagDetector = nullptr;

    _testData = "";
    
    std::cout << "Enabling video capture" << std::endl;

    _cap.open(0);
    if (!_cap.isOpened()) {
        std::cerr << "Couldn't open video capture device" << std::endl;
    }

    _detectionInfo.tagsize = 0.149;

    switch(camera) {
        case PICAMERA:
            width = 2560;
            height = 1920;

            _detectionInfo.fx = 571.8367321574847 * 4;
            _detectionInfo.fy = 572.8478769322065 * 4;
            _detectionInfo.cx = 308.4643624352032 * 4;
            _detectionInfo.cy = 248.84989598432 * 4;

            _cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
            _cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
            _cap.set(cv::CAP_PROP_FPS, 60);

            break;
        case OV9281:
            width = 1280;
            height = 800;

            _detectionInfo.fx = 1120.556787989777;
            _detectionInfo.fy = 1125.442975043206;
            _detectionInfo.cx = 634.3519601327893;
            _detectionInfo.cy = 406.9630521749576;

            break;
        default:
            break;
    }

    _tagFamily = tag36h11_create();
    _tagDetector = apriltag_detector_create();
    apriltag_detector_add_family(_tagDetector, _tagFamily);

    _tagDetector->quad_decimate = decimate;
    _tagDetector->quad_sigma = blur;
    _tagDetector->nthreads = 4;
    _tagDetector->debug = false;
    _tagDetector->refine_edges = true;

    std::cout << "Detector tag36h11 initialized\n";
    std::cout << width << "x" << height << " Decimation: " << decimate << std::endl;
}

Detector::~Detector() {
    apriltag_detector_destroy(_tagDetector);
    tag36h11_destroy(_tagFamily);
}

void Detector::run() {

    if (_rotation != 0) {
        _cap >> _frameRaw;
        cv::rotate(_frameRaw, _frame, _rotation - 1);
    } else {
        _cap >> _frame;
    }

    cvtColor(_frame, _greyFrame, cv::COLOR_BGR2GRAY);

    _img = new image_u8_t { 
        .width = _greyFrame.cols,
        .height = _greyFrame.rows,
        .stride = _greyFrame.cols,
        .buf = _greyFrame.data
    };

    _detections = apriltag_detector_detect(_tagDetector, _img);
    // _detections = zarray_create(1);

    for (int i = 0; i < zarray_size(_detections); i++) {

        zarray_get(_detections, 0, &_detectionInfo.det);
        apriltag_pose_t apPose;
        double err = estimate_tag_pose(&_detectionInfo, &apPose);
        _poses.push_back(Pose(apPose, _detectionInfo.det->id));

    }
}

void Detector::printPoses() {

    for (int i = 0; i < _poses.size(); i++) {
        std::cout << "Tag: " << _poses[i].getId() << "\n";
        _poses[i].printIn();
    }

}

void Detector::runTest() {

    std::cout << "Running Tests. Input actual distance in inches:\n";
    float previousDecimate = _tagDetector->quad_decimate;
    _tagDetector->quad_decimate = 1.0;
    std::string in;
    std::cin.clear();
    std::getline(std::cin, in, '\n');
    float actual = std::stof(in) * 0.0254;
    _testData += in + ", " + std::to_string(actual) + ", ";

    while(_tagDetector->quad_decimate < 10) {

        _detections = apriltag_detector_detect(_tagDetector, _img);

        double measured = -1.0;
        std::cout << _tagDetector->quad_decimate << " decimation: ";
        if (zarray_size(_detections) > 0) {
            zarray_get(_detections, 0, &_detectionInfo.det);
            apriltag_pose_t apPose;
            double err = estimate_tag_pose(&_detectionInfo, &apPose);
            Pose pose(apPose, _detectionInfo.det->id);
            measured = pose.getDistance();

            double error = abs(actual - measured) * 100;
            _testData += std::to_string(error) + ", ";
            std::cout << error << " cm error" << std::endl;
        } else {
            std::cout  << "-1 cm error" << std::endl;
        }

        if (_tagDetector->quad_decimate < 2) {
            _tagDetector->quad_decimate += 0.5;
        } else {
            _tagDetector->quad_decimate += 1;
        }
    }
    _testData += "\n";
    _tagDetector->quad_decimate = 1;
    std::cout << "Finished Test\n\n";
}

void Detector::saveData() {
    std::ofstream dataFile;
    dataFile.open("data.csv", std::ios_base::app);
    dataFile << _testData;
    dataFile.close();
    std::cout << "Output data to data.csv\n";
}

void Detector::destroyDetections() {
    apriltag_detections_destroy(_detections);
}