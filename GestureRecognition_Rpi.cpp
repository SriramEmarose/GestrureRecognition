///////////////////////////////////////////////////////////////////////////////////
/// GestureDetection_Rpi.cpp
///
/// Detects hand gestures and controls respective GPIO
///
///
/// Note: Make sure to use with a plain background with controlled lighting
/// else, additional segmentation may be needed wo filter out the clutter
/// 
/// Connection Details,
/// 1. For 5 fingers, Pin number 38 in board will be high
/// 2. For 3 fingers, Pin number 40 in board will be high
/// 3. Both pin Low when fingers closed
///
/// Dependencies: 
/// OpenCV, WiringPi, C++11
///
/// Author: Sriram Emarose
/// Contact: sriram.emarose@gmail.com
///
///////////////////////////////////////////////////////////////////////////////////

// Check if RPI, else just do detection
#ifdef __arm__
#include <wiringPi.h>
#endif

#include <iostream>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

void DetectGesture(cv::Mat frame);

int controlPin1 = 28; // Pin 38 in Rpi3
int controlPin2 = 29; // Pin 40 in Rpi3

int main()
{

#ifdef __arm__
    if (wiringPiSetup() == -1)
    {
        std::cout << "Unable to setup GPIOs access \n";
        return 1;
    }
    pinMode(controlPin1, OUTPUT);
    pinMode(controlPin2, OUTPUT);
#endif

    cv::Mat img;

    cv::VideoCapture capture(0);
    {
        while (capture.isOpened())
        {
            capture >> img;

            if (!img.empty())
            {
                DetectGesture(img);
                //cv::imshow("live",img);
                cv::waitKey(1);
            }
            
            // If escape key is pressed, bail out
            if (cv::waitKey(100) == 27)
                break;
        }
    }
    return 0;
}
void DetectGesture(cv::Mat frame)
{
    cv::Mat img, binary;
    cvtColor(frame, img, CV_BGR2GRAY);

    //thresholding
    cv::threshold(img, binary, 35, 255, cv::THRESH_BINARY_INV);    

    //Contour storage
    vector<vector<Point>> Contours;
    vector<Vec4i> hierarchy;    

    //morphological transformations
    erode(binary, binary, getStructuringElement(MORPH_RECT, Size(3, 3)));
    erode(binary, binary, getStructuringElement(MORPH_RECT, Size(3, 3)));

    dilate(binary, binary, getStructuringElement(MORPH_RECT, Size(8, 8)));
    dilate(binary, binary, getStructuringElement(MORPH_RECT, Size(8, 8)));

    cv::imshow("SegmentedOut", binary);
    
    //finding the contours required
    findContours(binary, Contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0));

    //finding the contour of largest area and storing its index
    int biggestContour = 0;
    int maxarea = 0;

    //convex Hull storage
    vector<vector<Point>> cnvxHull(Contours.size());
    vector<vector<int>>   cnvxHullsI(Contours.size());
    vector<vector<Vec4i>> defects(Contours.size());

    for (int i = 0; i < Contours.size(); i++)
    {
        double currentArea = contourArea(Contours[i]);
        if (currentArea > maxarea)
        {
            maxarea = currentArea;
            biggestContour = i;
        }
    }

    for (int i = 0; i < Contours.size(); i++)
    {
        convexHull(Contours[i], cnvxHull[i], false);
        convexHull(Contours[i], cnvxHullsI[i], false);
        if (cnvxHullsI[i].size() > 3)
        {
            convexityDefects(Contours[i], cnvxHullsI[i], defects[i]);
        }
    }

    std::vector<cv::Point> peakCount;
    

    if (maxarea > 100)
    {

        /// Draw convexity Defects
        for (int j = 0; j < defects[biggestContour].size(); ++j)
        {
            const Vec4i &v = defects[biggestContour][j];            
            float depth = v[3] / 256;

            if (depth > 10) //
            {
                int startidx = v[0];
                Point ptStart(Contours[biggestContour][startidx]);
                peakCount.push_back(ptStart);

                int endidx = v[1];
                Point ptEnd(Contours[biggestContour][endidx]);

                int faridx = v[2];
                Point ptFar(Contours[biggestContour][faridx]);

                line(frame, ptStart, ptEnd, Scalar(0, 255, 0), 5);
                line(frame, ptStart, ptFar, Scalar(0, 255, 0), 5);
                line(frame, ptEnd, ptFar, Scalar(0, 255, 0), 5);
                circle(frame, ptFar, 4, Scalar(255, 0, 0), 5);
                circle(frame, ptStart, 6, Scalar(0, 0, 255), 5);
            }
        }
    }

    if (peakCount.size() > 4)
    {
        std::cout<<"5 Fingers detected \n";

        #ifdef __arm__
        digitalWrite(controlPin1, 1);
        #endif
    }
    else if (peakCount.size() == 4)
    {
        std::cout<<" 3 Fingers Detected \n";
        #ifdef __arm__
        digitalWrite(controlPin2, 1);
        digitalWrite(controlPin1, 0);
        #endif
    }
    else
    {
        std::cout<<"0 Fingers \n";
        #ifdef __arm__
        digitalWrite(controlPin1, 0);
        digitalWrite(controlPin2, 0);
        #endif
    }

    cv::imshow("Live", frame);
}