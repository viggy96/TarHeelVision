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

using namespace cv;
using namespace std;
using namespace boost::asio::ip;

VideoCapture camera;
UMat frame, frame_hsv, frame_thresh;
std::string message  = "";
bool frameExists = false;
char key;
int thresh = 100;
int id_thresh = 0;
int id_thresh_max = 0;
int max_thresh = 255;
int thresh_max = 100;
int lowH = 0, highH = 179;
int lowS = 0, highS = 255;
int lowV = 0, highV = 255;
int minContourSize = 25;
double image_t = 0, threshold_t = 0;
const double PI = 3.14159265358979323846264338328;

void denoiseMat(Mat, int);
double rectangularity(double, Rect);
double circularity(double, float);

int main(int argc, char *argv[]) {
  // roborio-TEAM-frc.local:1185/?action=stream
  VideoCapture camera;
  if (argc < 3) {
    cout << "Not enough arguments." << endl;
    cout << "usage: vision camera_stream_address roboRIO_address" << endl;
    return 1;
  } else if (argc > 1) {
    std::string address;
    if (!boost::starts_with(argv[1], "http://")) address.append("http://");
    address.append(argv[1]);
    if (!boost::ends_with(argv[1], "/") && !boost::ends_with(argv[1], "?action=stream"))
      address.append("/");
    address.append("video?x.mjpeg");
    camera = VideoCapture(address);
  } else {
    camera = VideoCapture(CV_CAP_ANY);
  }

  boost::asio::io_service io_service;
  udp::resolver resolver(io_service);
  udp::resolver::query query(udp::v4(), argv[2], "3331");
  udp::endpoint receiver_endpoint = *resolver.resolve(query);
  udp::socket socket(io_service);
  socket.open(udp::v4());

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

    createTrackbar("LowH", "Image", &lowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Image", &highH, 179);

    createTrackbar("LowS", "Image", &lowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Image", &highS, 255);

    createTrackbar("LowV", "Image", &lowV, 255); //Value (0 - 255)
    createTrackbar("HighV", "Image", &highV, 255);

    cvtColor(frame, frame_hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    inRange(frame_hsv, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frame_thresh); //Threshold the image

    namedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE);
    imshow("Thresholded Image", frame_thresh);

    Mat canny_output, drawing;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy, lines;

    Canny(frame_thresh, canny_output, thresh, thresh_max, 3);
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > contours_poly(contours.size());
    vector<Point2f> centre(contours.size());
    vector<float> radius(contours.size());
    Scalar circle_colour(0, 0, 255);
    Scalar rect_colour(0, 255, 0);
    drawing = Mat::zeros(canny_output.size(), CV_8UC3);

    double largest_area = -1;
    Point largest_rect;

    message = "";

    #pragma omp parallel for simd
    for (auto contour = contours.begin(); contour != contours.end(); contour++) {
      #pragma omp task
      double area = contourArea(*contour);
      if (area < minContourSize) continue;

      Rect rect = boundingRect(*contour);
      bool isRect = (rectangularity(area, rect)) > 75;
      if (isRect && area > largest_area) {
        largest_area = area;
        largest_rect = Point((rect.x + rect.width)/2, (rect.y + rect.height)/2);
      }
    }
    message += "{";
    message += "'x':";
    message += std::to_string(largest_rect.x);
    message += ",";
    message += "'y':";
    message += std::to_string(largest_rect.y);
    message += "}";

    socket.send_to(boost::asio::buffer(message), receiver_endpoint);
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
