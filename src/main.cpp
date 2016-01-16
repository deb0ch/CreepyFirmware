/*
 * Copyright (c) 2016 Thomas Chauvot de Beauchene
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <err.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include "gnuplot-iostream.h"

#include "DxlServo.hh"
#include "FaceDetector.hh"
#include "PIDControl.hh"
#include "Timer.hh"

#define PLOT_MAX_S	30               // in second
#define TIMEOUT_S       10               // Tracking timeout in seconds
#define TRACKER_TYPE    "MEDIANFLOW"

void	plot_stats(Gnuplot & gp,
		   int x,
		   double y1,
		   double y2,
		   double y3,
		   double y4,
		   double y5);

Gnuplot     g_gp;

Timer       g_timer;

DxlServo    g_servo1(1);
DxlServo    g_servo2(2);

float       g_speedSetpoint;	// Debug (for plotting curves)

float	    g_prevTime = g_timer.getTime();

enum eState
{
    IDLE,
    TRACKING
};

void    idleActions(enum eState& state, FaceDetector& faceDetect, cv::Mat& frame, cv::Rect& box)
{
    g_servo1.setMovingSpeed(0);
    box = faceDetect.detect(frame);
    if (box.area() > 0)
        state = TRACKING;
    g_prevTime = g_timer.getTime();
}

void	displayInfo(cv::Mat& frame, float errorX, float errorY)
{
  cv::line(frame,
	   cv::Point(frame.cols / 2, frame.rows / 2),
	   cv::Point(frame.cols / 2 - (int)errorX, frame.rows / 2),
	   cv::Scalar(0, 255, 0));
  cv::line(frame,
	   cv::Point(frame.cols / 2, frame.rows / 2),
	   cv::Point(frame.cols / 2, frame.rows / 2 - (int)errorY),
	   cv::Scalar(255, 0, 0));
  cv::line(frame,
	   cv::Point(frame.cols / 2, frame.rows / 2),
	   cv::Point(frame.cols / 2 - (int)errorX, frame.rows / 2 - (int)errorY),
	   cv::Scalar(0, 0, 255));
}

void    armCorrectPosition(cv::Mat& frame, cv::Rect& face)
{
    static bool         servo1Initialized = false;
    static bool         servo2Initialized = false;
    float		currentTime = g_timer.getTime();
    float               errorX = (frame.cols / 2.f) - (face.x + face.width / 2.f);
    float               errorY = (frame.rows / 2.f) - (face.y + face.height / 2.f);
    static PIDControl	pid1(0.0012, 0.00075, 0.00005, 1, 10);
    static PIDControl	pid2(0.0012, 0.00075, 0.00005, 1, 10);
    float		speedCommand1 = 0;
    float		speedCommand2 = 0;
    float		posCommand1;
    float		posCommand2;

    displayInfo(frame, errorX, errorY);
    if (!servo1Initialized)
    {
        if (!g_servo1.init())
            throw std::exception();
	g_servo1.setCWAngleLimit(0);
        g_servo1.setCCWAngleLimit(1); // Servo now set to joint mode
	g_servo1.setMovingSpeed(0.5);
        servo1Initialized = true;
    }
    if (!servo2Initialized)
    {
        if (!g_servo2.init())
            throw std::exception();
	g_servo2.setCWAngleLimit(0.25);
        g_servo2.setCCWAngleLimit(0.5); // Servo now set to joint mode
	g_servo2.setMovingSpeed(0.5);
        servo2Initialized = true;
    }

    pid1.set_dt((currentTime - g_prevTime) / 1000000.f);
    pid1.set_input_filter_all(errorX);
    speedCommand1 = pid1.get_pid();
    posCommand1 = g_servo1.presentPos() + (currentTime - g_prevTime) / 1000000.f * speedCommand1;
    g_servo1.setGoalPos(posCommand1);

    g_speedSetpoint = speedCommand1; // debug

    pid2.set_dt((currentTime - g_prevTime) / 1000000.f);
    pid2.set_input_filter_all(-errorY);
    speedCommand2 = pid2.get_pid();
    posCommand2 = g_servo2.presentPos() + (currentTime - g_prevTime) / 1000000.f * speedCommand2;
    g_servo2.setGoalPos(posCommand2);

    g_prevTime = currentTime;
}

void    trackingActions(enum eState& state,
			cv::Ptr<cv::Tracker>& tracker,
			cv::Mat& frame,
			cv::Rect& box)
{
    static float    currentTime = g_timer.getTime();
    static float    timeoutStart = 0;
    static bool     initialized = false;
    cv::Rect2d      box2d = box;                    // Type needed by tracker (wtf ?)

    currentTime = g_timer.getTime();
    if (!initialized)
    {
        timeoutStart = currentTime;
        if (!tracker->init(frame, box2d))
        {
            std::cerr << "tracker init failed" << std::endl;
            state = IDLE;
            return;
        }
        else
            initialized = true;
    }
    else
    {
        if (currentTime - timeoutStart > TIMEOUT_S * 1000000)
            std::cout << "************ Timeout ! ************" << std::endl;
        if (!tracker->update(frame, box2d)
            || currentTime - timeoutStart > TIMEOUT_S * 1000000) // Timeout
        {
	    g_servo1.setMovingSpeed(0);
            tracker.release();
            tracker = cv::Tracker::create(TRACKER_TYPE);
            initialized = false;
            state = IDLE;
            return;
        }
        box = box2d;
        armCorrectPosition(frame, box);
    }
}

int main()
{
    cv::VideoCapture        cap(1);
    cv::Mat                 frame;
    FaceDetector            faceDetect("resources/haarcascade_frontalface_alt.xml");
    cv::Ptr<cv::Tracker>    tracker = cv::Tracker::create(TRACKER_TYPE);    //  "MIL", "BOOSTING", "MEDIANFLOW", "TLD"
    cv::Rect                box;
    enum eState             state = IDLE;

    if (!DxlServo::devInit(0))
        errx(EXIT_FAILURE, "error: could not initialize dxl serial device %d", 0);
    if (!cap.isOpened())
        errx(EXIT_FAILURE, "error: could not open video capture");
    cv::namedWindow("frame", cv::WINDOW_KEEPRATIO);
    while (cv::waitKey(30) < 0)
    {
        cap >> frame;
        switch (state)
        {
            case IDLE:
            std::cout << "idle..." << std::endl;
                idleActions(state, faceDetect, frame, box);
                break;
            case TRACKING:
                std::cout << "tracking..." << std::endl;
                trackingActions(state, tracker, frame, box);
                break;
        }
        cv::rectangle(frame, box, cv::Scalar(127, 127, 0), 2, 2);
        cv::imshow("frame", frame);
	plot_stats(g_gp,
		   g_timer.getTime(),
		   g_speedSetpoint,
		   g_servo1.presentSpeed(),
		   state / 2,
		   g_servo1.presentPos(),
		   0);
    }
    return 0;
}

void	plot_stats(Gnuplot & gp,
		   int x,
		   double y1,
		   double y2,
		   double y3,
		   double y4,
		   double y5)
{
  static double xmin = x;
  static double xmax = x + (PLOT_MAX_S * 1000000);
  static double ymin = y1;
  static double ymax = y1;
  static std::list<boost::tuple<double, double> > y1Data;
  static std::list<boost::tuple<double, double> > y2Data;
  static std::list<boost::tuple<double, double> > y3Data;
  static std::list<boost::tuple<double, double> > y4Data;
  static std::list<boost::tuple<double, double> > y5Data;

  if (xmax - xmin > (PLOT_MAX_S * 1000000))
    xmin = xmax - (PLOT_MAX_S * 1000000);
  if (x >= xmax)
    xmax = x;

  if (y1 <= ymin)
    ymin = y1;
  if (y1 >= ymax)
    ymax = y1;
  if (y2 <= ymin)
    ymin = y2;
  if (y2 >= ymax)
    ymax = y2;
  if (y3 <= ymin)
    ymin = y3;
  if (y3 >= ymax)
    ymax = y3;
  if (y4 <= ymin)
    ymin = y4;
  if (y4 >= ymax)
    ymax = y4;
  if (y5 <= ymin)
    ymin = y5;
  if (y5 >= ymax)
    ymax = y5;

  while ((boost::get<0>(y1Data.back()) - boost::get<0>(y1Data.front())) > PLOT_MAX_S * 1000000)
    {
      y1Data.pop_front();
      y2Data.pop_front();
      y3Data.pop_front();
      y4Data.pop_front();
      y5Data.pop_front();
    }

  y1Data.push_back(boost::make_tuple(x, y1));
  y2Data.push_back(boost::make_tuple(x, y2));
  y3Data.push_back(boost::make_tuple(x, y3));
  y4Data.push_back(boost::make_tuple(x, y4));
  y5Data.push_back(boost::make_tuple(x, y5));

  gp << "set xrange [" << xmin << ":" << xmax << "]\n "
     << "set yrange [" << ymin << ":" << ymax << "]\n";

  gp << "plot "
     << "'-' with lines title 'speed command'"
     << ", "
     << "'-' with lines title 'real speed'"
     << ", "
     << "'-' with lines title 'mode'"
     << ", "
     << "'-' with lines title '-'"
     << ", "
     << "'-' with lines title '-'"
     << "\n";

  gp.send1d(y1Data);
  gp.send1d(y2Data);
  gp.send1d(y3Data);
  gp.send1d(y4Data);
  gp.send1d(y5Data);
}
