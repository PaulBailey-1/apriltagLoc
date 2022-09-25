#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

int main() {

  int width = 640;
  int height = 480;

  cv::VideoCapture cap(0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap.set(cv::CAP_PROP_FPS, 30);

  if (!cap.isOpened()) {
    printf("Error: Couldn't open camera\n");
    exit(-1);
  }

  int idx = 0;

  cv::Mat frame;

  while (true) {
    cap >> frame;
    // printf("Got frame %i x %i\n", frame.cols, frame.rows);

    imshow("Camera", frame);
    int k = cv::pollKey();
    if (k == 27) { // esc
      break;
    }
    if (k == 32) { // space
      char filename[20];
      sprintf(filename, "images/image%i.jpg", idx);
      if (imwrite(filename, frame)) {
        printf("Stored image%i.jpg\n", idx);
        idx++;
      }
    }
    // if (k != -1) {
    //   printf("%i\n", k);
    // }

  }

  cap.release();
  cv::destroyAllWindows();

  return 0;
}