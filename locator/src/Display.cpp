#include "Display.h"
#include <iomanip>

Display::Display() {

    _frame = nullptr;

    _fps = 0.0;

    _width = 0;
    _height = 0;

    _imageIdx = 0;
    std::fstream idxFile;
    std::string idxFilePath = "./images/idx";
    idxFile.open(idxFilePath, std::ios::in);
    if (idxFile.is_open()) {
        idxFile >> _imageIdx;
    }
    idxFile.close();

    cv::namedWindow("Tag Detections");

}

void Display::drawDetections(zarray_t *detections) {

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        cv::line(*_frame, cv::Point(det->p[0][0], det->p[0][1]),
                    cv::Point(det->p[1][0], det->p[1][1]),
                    cv::Scalar(0, 0xff, 0), 2);
        cv::line(*_frame, cv::Point(det->p[0][0], det->p[0][1]),
                    cv::Point(det->p[3][0], det->p[3][1]),
                    cv::Scalar(0, 0, 0xff), 2);
        cv::line(*_frame, cv::Point(det->p[1][0], det->p[1][1]),
                    cv::Point(det->p[2][0], det->p[2][1]),
                    cv::Scalar(0xff, 0, 0), 2);
        cv::line(*_frame, cv::Point(det->p[2][0], det->p[2][1]),
                    cv::Point(det->p[3][0], det->p[3][1]),
                    cv::Scalar(0, 0, 0xff), 2);

        std::stringstream ss;
        ss << det->id;
        std::string text = ss.str();
        
        int baseline;
        cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                        &baseline);
        cv::putText(*_frame, text, cv::Point(det->c[0]-textsize.width/2,
                                    det->c[1]+textsize.height/2),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
    }
}

void Display::showFrame() {
    cv::Mat outFrame;
    cv::resize(*_frame, outFrame, cv::Size(_width, _height));

    std::stringstream fpsText; 
    fpsText << "FPS: " << std::fixed << std::setprecision(1) << _fps;
    cv::putText(outFrame, fpsText.str(), cv::Point(20, 40), fontface, fontscale, cv::Scalar(0x00, 0x00, 0xff));

    const int crossSize = 10;
    cv::line(outFrame, cv::Point(_width / 2 - crossSize, _height / 2), cv::Point(_width / 2 + crossSize, _height / 2), cv::Scalar(0, 0, 0xff));
    cv::line(outFrame, cv::Point(_width / 2, _height / 2 - crossSize), cv::Point(_width / 2, _height / 2 + crossSize), cv::Scalar(0, 0, 0xff));

    cv::imshow("Tag Detections", outFrame);
}

void Display::saveFrame() {
    std::string imageName = "./images/image" + std::to_string(_imageIdx) + ".jpg";
    cv::imwrite(imageName, *_frame);
    std::cout << "Saved image to " << imageName << std::endl;
    _imageIdx++;

    std::fstream idxFile;
    std::string idxFilePath = "./images/idx";
    idxFile.open(idxFilePath, std::ofstream::out | std::ofstream::trunc);
    idxFile << _imageIdx;
    idxFile.close();
}