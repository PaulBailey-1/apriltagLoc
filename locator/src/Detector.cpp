#include "Detector.h"
#include <iostream>

#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

Detector::Detector(int width, int height, int rotation, Camera camera, double decimate, double blur, int depthMap) : 
        _frame(), _frameRaw(), _greyFrame(), _cap(), _detectionInfo(), _poses(), _align2Color(RS2_STREAM_INFRARED)
    {

    _rotation = rotation;

    _detections = nullptr;
    _tagFamily = nullptr;
    _tagDetector = nullptr;

    _testData = "";

    _depthMap = depthMap;
    
    std::cout << "Enabling video capture" << std::endl;

    try {
        init();
    } catch (const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        init();
    }

    rs2::video_stream_profile colorProfile = _pipe.get_active_profile().get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
    rs2::video_stream_profile depthProfile = _pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

    _colorIntrinsics = colorProfile.get_intrinsics();
    _colorExtrinsics = colorProfile.get_extrinsics_to(depthProfile);
    
    _depthIntrinsics = depthProfile.get_intrinsics();
    _depthExtrinsics = depthProfile.get_extrinsics_to(colorProfile);

    // _cap.open(0);
    // if (!_cap.isOpened()) {
    //     std::cerr << "Couldn't open video capture device" << std::endl;
    // }

    _detectionInfo.tagsize = 0.1524;

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
        case D435:

            width = _depthIntrinsics.width;
            height = _depthIntrinsics.height;

            _detectionInfo.fx = _colorIntrinsics.fx;
            _detectionInfo.fy = _colorIntrinsics.fy;
            _detectionInfo.cx = _colorIntrinsics.ppx;
            _detectionInfo.cy = _colorIntrinsics.ppy;

            break;

        default:
            break;
    }

    if (rotation == 1 || rotation == 3) {
        double temp = _detectionInfo.fx;
        _detectionInfo.fx = _detectionInfo.fy;
        _detectionInfo.fy = temp;

        temp = _detectionInfo.cx;
        _detectionInfo.cx = _detectionInfo.cy;
        _detectionInfo.cy = temp;

    }

    _tagFamily = tag16h5_create();
    _tagDetector = apriltag_detector_create();
    apriltag_detector_add_family_bits(_tagDetector, _tagFamily, 0);

    _tagDetector->quad_decimate = decimate;
    _tagDetector->quad_sigma = blur;
    _tagDetector->nthreads = 4;
    _tagDetector->debug = false;
    _tagDetector->refine_edges = true;
    _tagDetector->qtp.max_line_fit_mse = 1; // Default 10, change if problems

    std::cout << "Detector tag16h5 initialized\n";
    std::cout << width << "x" << height << " Decimation: " << decimate << std::endl;
}

Detector::~Detector() {
    apriltag_detector_destroy(_tagDetector);
    tag16h5_destroy(_tagFamily);
    _pipe.stop();
}

void Detector::init() {
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device dev = devices[0];

    rs2::config cfg;
    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::string json_file_name = "rsConfig.json";

    std::cout << "Configuring camera : " << serial << std::endl;

    auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

    // Check if advanced-mode is enabled to pass the custom config
    if (!advanced_mode_dev.is_enabled())
        {
            // If not, enable advanced-mode
            advanced_mode_dev.toggle_advanced_mode(true);
            std::cout << "Advanced mode enabled. " << std::endl;
        }

    std::ifstream t(json_file_name);
    std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    advanced_mode_dev.load_json(preset_json);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2);
    cfg.enable_device(serial);
    rs2::pipeline_profile pipeProfile = _pipe.start(cfg);

    rs2::depth_sensor depth_sensor = pipeProfile.get_device().first<rs2::depth_sensor>();
    _depthScale = depth_sensor.get_depth_scale();
}

void Detector::run() {

    try {

        rs2::frameset data = _pipe.wait_for_frames();

        rs2::depth_frame depthFrame = data.get_depth_frame();
        rs2::frame irFrame = data.get_infrared_frame();
        // rs2::frameset processed = _align2ir.process(data);
        // rs2::depth_frame alignedDepthFrame = processed.get_depth_frame();

        int irWidth = irFrame.as<rs2::video_frame>().get_width();
        int irHeight = irFrame.as<rs2::video_frame>().get_height();

        // _greyFrame = cv::Mat(cv::Size(irWidth, irHeight), CV_8UC1, (void*)irFrame.get_data(), cv::Mat::AUTO_STEP);
        _frameRaw = cv::Mat(cv::Size(irWidth, irHeight), CV_8UC1, (void*) irFrame.get_data(), cv::Mat::AUTO_STEP);
        // cv::cvtColor(_frameRaw, _greyFrame, cv::COLOR_BGR2GRAY);
        // cv::cvtColor(_frameRaw, _greyFrame, cv::COLOR_BGR2GRAY);
        _greyFrame = cv::Mat(_frameRaw, cv::Rect(0, _yCrop, irWidth, irHeight - (_yCrop * 2)));

        // rs2::frame depthFrameColor;
        // if (_depthMap) {
        //     depthFrameColor = rs2::frame(depthFrame);
        //     depthFrameColor.apply_filter(_colorMap);
        //     _frame = cv::Mat(cv::Size(depthFrameColor.as<rs2::video_frame>().get_width(), depthFrameColor.as<rs2::video_frame>().get_height()), CV_8UC3, (void*)depthFrameColor.get_data(), cv::Mat::AUTO_STEP);
        // } else {
        cv::Mat frameMono = cv::Mat(cv::Size(irWidth, irHeight), CV_8UC1);
        _frameRaw(cv::Rect(0, _yCrop, irWidth, irHeight - (_yCrop * 2))).copyTo(frameMono(cv::Rect(0, _yCrop, irWidth, irHeight - (_yCrop * 2))));
        cv::cvtColor(frameMono, _frame, cv::COLOR_GRAY2BGR);
        
            // _frame = cv::Mat(_frameRaw, cv::Rect(0, YFILTER, colorWidth, colorHeight - (YFILTER * 2)));

            // cv::cvtColor(_greyFrame, _frame, cv::COLOR_GRAY2BGR);
            // cv::cvtColor(_frameRaw, _frame, cv::COLOR_RGB2BGR);
        // }

            // if (_rotation != 0) {
    //     _cap >> _frameRaw;
    //     cv::rotate(_frameRaw, _frame, _rotation - 1);
    // } else {
    //     _cap >> _frame;
    // }

    // cvtColor(_frame, _greyFrame, cv::COLOR_BGR2GRAY);

    _img = new image_u8_t { 
        .width = _greyFrame.cols,
        .height = _greyFrame.rows,
        .stride = _greyFrame.cols,
        .buf = _greyFrame.data
    };

    if (_img->width > 0) {
        _detections = apriltag_detector_detect(_tagDetector, _img);
    } else {
        _detections = zarray_create(1);
    }

    _poses.clear();

    for (int i = 0; i < zarray_size(_detections); i++) {

        zarray_get(_detections, i, &_detectionInfo.det);
        _detectionInfo.det->c[1] = _detectionInfo.det->c[1] + _yCrop;
        for (int j = 0; j < 4; j++) {
            _detectionInfo.det->p[j][1] = _detectionInfo.det->p[j][1] + _yCrop;
        }
        zarray_set(_detections, i, &_detectionInfo.det, NULL);

        if(_detectionInfo.det->id <= 8 && _detectionInfo.det->id >= 1) {

            apriltag_pose_t apPose;
            double err = estimate_tag_pose(&_detectionInfo, &apPose);

            double steroDistance = 0.0;
            try {

                float colorTagCenter[2];
                float depthTagCenter[2];

                colorTagCenter[0] = (int)_detectionInfo.det->c[0];
                colorTagCenter[1] = (int)_detectionInfo.det->c[1];

                depthTagCenter[0] = colorTagCenter[0];
                depthTagCenter[1] = colorTagCenter[1];

                steroDistance = depthFrame.get_distance(depthTagCenter[0], depthTagCenter[1]);
            } catch (const rs2::error & e) {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            }

            double pixelsX = _detectionInfo.det->c[0] - (_img->width / 2);
            _poses.push_back(Pose(apPose, _detectionInfo.det->id, pixelsX, _detectionInfo.fx, steroDistance));

        } else {
            zarray_remove_index(_detections, i, false);
        }
    }

    } catch (const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    }
}

void Detector::printPoses() {

    for (int i = 0; i < _poses.size(); i++) {
        //std::cout << "Tag: " << _poses[i].getId() << "\n";
        //_poses[i].printIn();
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
            // Pose pose(apPose, _detectionInfo.det->id, 0.0);
            // measured = pose.getDistance();

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
