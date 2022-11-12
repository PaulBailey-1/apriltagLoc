
extern "C" {
    #include "apriltag.h"
    #include "apriltag_pose.h"
    #include "tag36h11.h"
}

#include "opencv2/videoio.hpp"
#include <vector>
#include <fstream>

#include "Pose.h"

class Detector {
public:

    enum Camera {
        PICAMERA,
        OV9281
    };

    Detector(int width, int height, int rotation, Camera camera, double decimate, double blur);
    ~Detector();

    void run();
    void printPoses();
    void runTest();
    void saveData();
    void destroyDetections();

    zarray_t* getDetections() {
        if (_detections != nullptr) {
            return _detections;
        }
        return nullptr;
    }

    int getDetectionsSize() {
        if (_detections != nullptr) {
            return zarray_size(_detections);
        }
        return -1;
    }

    cv::Mat* getFrame() {
        return &_frame;
    }

    std::vector<Pose> getPoses() {
        return _poses;
    }

private:

    zarray_t* _detections;
    std::vector<Pose> _poses;

    cv::VideoCapture _cap;

    apriltag_detection_info_t _detectionInfo;
    apriltag_family_t* _tagFamily;
    apriltag_detector_t* _tagDetector;
    image_u8_t* _img;

    int _rotation;

    cv::Mat _frameRaw, _greyFrame, _frame;

    std::string _testData;

};