#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <typeinfo>
#include <ctime>
#include <cmath>
#include <thread>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>

using namespace cv;
using namespace std;
using namespace boost::asio::ip;

VideoCapture camera;
Mat frame, frame_hsv, frame_thresh;
std::string message  = "";
bool frameExists = false;
char key;
int thresh = 100;
int id_thresh = 0;
int id_thresh_max = 0;
int max_thresh = 255;
int thresh_max = 100;
int lowR = 0, highR = 255;
int lowG = 0, highG = 255;
int lowB = 0, highB = 255;
int minContourSize = 25;
double image_t = 0, threshold_t = 0;
const double PI = 3.14159265358979323846264338328;

void denoiseMat(Mat, int);
double rectangularity(double, Rect);
double circularity(double, float);

int main(int argc, char *argv[]) {
  VideoCapture camera;
  if (argc > 1 && boost::starts_with(argv[1], "http://")) {
    std::string address;
    address.append(argv[1]);
    address.append("video?x.mjpeg");
    camera = VideoCapture(address);
  } else {
    camera = VideoCapture(CV_CAP_ANY);
  }

  #pragma omp parallel for
  for (;;) {
    frameExists = camera.read(frame);

    if (!frameExists) {
      printf("Cannot read image from camera. \n");
      return -1;
    }

    key = waitKey(10);     // Capture Keyboard stroke
    if (char(key) == 27) {
      printf("\n");
      break;      // If you hit ESC key loop will break.
    }

    namedWindow("Image", CV_WINDOW_AUTOSIZE);
    imshow("Image", frame);

    createTrackbar("LowR", "Image", &lowR, 255); //Hue (0 - 179)
    createTrackbar("HighR", "Image", &highR, 255);

    createTrackbar("LowG", "Image", &lowG, 255); //Saturation (0 - 255)
    createTrackbar("HighG", "Image", &highG, 255);

    createTrackbar("LowB", "Image", &lowB, 255); //Value (0 - 255)
    createTrackbar("HighB", "Image", &highB, 255);

    //#pragma omp task
    //cvtColor(frame, frame_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    #pragma omp task
    inRange(frame, Scalar(lowB, lowG, lowR), Scalar(highB, highG, highR), frame_thresh); //Threshold the image

    namedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE);
    imshow("Thresholded Image", frame_thresh);

    Mat canny_output, drawing;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy, lines;

    #pragma omp task
    Canny(frame_thresh, canny_output, thresh, thresh_max, 3);
    #pragma omp task
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > contours_poly(contours.size());
    vector<Point2f> centre(contours.size());
    vector<float> radius(contours.size());
    Scalar circle_colour(0, 0, 255);
    Scalar rect_colour(0, 255, 0);
    drawing = Mat::zeros(canny_output.size(), CV_8UC3);

    double largest_area = -1;
    Point largest_rect;

    #pragma omp parallel for simd
    for (int i = 0; i < contours.size(); i++) {
      #pragma omp task
      double area = contourArea(contours[i]);
      if (area < minContourSize) continue;

      Rect rect = boundingRect(contours[i]);
      bool isRect = (rectangularity(area, rect)) > 85;
    }
  }

  waitKey(0);
  return 0;
}

void denoiseMat(Mat frame, int structure_size) {
  //morphological opening (remove small objects from the foreground)
  erode(frame, frame, getStructuringElement(MORPH_RECT, Size(structure_size, structure_size)));
  dilate(frame, frame, getStructuringElement(MORPH_RECT, Size(structure_size, structure_size)));

  //morphological closing (fill small holes in the foreground)
  dilate(frame, frame, getStructuringElement(MORPH_RECT, Size(structure_size, structure_size)));
  erode(frame, frame, getStructuringElement(MORPH_RECT, Size(structure_size, structure_size)));

}

double rectangularity(double contourArea, Rect boundRect) {
  double boundRectArea = boundRect.width * boundRect.height;
  return (contourArea / boundRectArea) * 100;
}

double circularity(double contourArea, float radius) {
  double boundCircleArea = PI*pow(radius, 2);
  return (contourArea / boundCircleArea) * 100;
}
