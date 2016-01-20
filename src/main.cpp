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

#include "gnuplot-iostream.h"

#include "DxlServo.hh"
#include "FaceDetector.hh"
#include "FaceTracker.hh"
#include "PIDControl.hh"
#include "Timer.hh"

#define PLOT_MAX_S	30               // in second
#define TIMEOUT_S       10               // Tracking timeout in seconds

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
float       g_posCommand;	// Debug (for plotting curves)

enum eState
{
    IDLE,
    TRACKING
};

void	overlayFps(cv::Mat& frame, float dt)
{
  cv::putText(frame,
	      "fps: " + std::to_string(1.f / dt).substr(0, 4),
	      cv::Point(20, frame.rows - 20),
	      CV_FONT_HERSHEY_SCRIPT_COMPLEX,
	      0.75,
	      cvScalar(0, 0, 255));
}

void	overlayTrackingInfo(cv::Mat& frame, cv::Rect box, float errorX, float errorY)
{
  cv::rectangle(frame, box, cv::Scalar(127, 127, 0), 2, 2);
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

void    armCorrectPosition(cv::Mat& frame, cv::Rect& face, float dt, int &skipFrames)
{
    static PIDControl	pid1(0.0015, 0, 0, 0, 10);
    static PIDControl	pid2(0.0015, 0, 0, 0, 10);
    float               errorX = (frame.cols / 2.f) - (face.x + face.width / 2.f);
    float               errorY = (frame.rows / 2.f) - (face.y + face.height / 2.f);
    float		speedCommand;
    float		posCommand;

    pid1.set_dt(dt);
    pid1.set_input_filter_all(errorX);
    speedCommand = pid1.get_pid();
    posCommand = g_servo1.presentPos() + speedCommand * dt;
    if (!skipFrames)
      g_servo1.setGoalPos(posCommand);

    pid2.set_dt(dt);
    pid2.set_input_filter_all(-errorY);
    speedCommand = pid2.get_pid();
    posCommand = g_servo2.presentPos() + speedCommand * dt;
    if (!skipFrames)
      g_servo2.setGoalPos(posCommand);

    if (skipFrames)
      --skipFrames;

    overlayTrackingInfo(frame, face, errorX, errorY);

    g_speedSetpoint = speedCommand; // debug
    g_posCommand = posCommand;
}

float	idleGetSineOffset(float currentTime, float freq, float limit_low, float limit_high)
{
  return asin(2 * (g_servo1.presentPos() - ((limit_high - limit_low) / 2) - limit_low)
	      / (limit_high - limit_low))
         - (2 * M_PI * freq * currentTime);
}

void    idleActions(enum eState& state,
		    FaceDetector& faceDetect,
		    cv::Mat& frame,
		    cv::Rect& box,
		    float dt,
		    bool& reset)
{
    static float offset;
    float freq = 1.f / 50.f;
    float currentTime = g_timer.getTime() / 1000000.f;
    float limit_low = 0;
    float limit_high = 1;

    (void)dt;
    if (reset)
      {
	offset = idleGetSineOffset(currentTime, freq, limit_low, limit_high);
	reset = false;
      }
    g_servo1.setGoalPos((sin(2 * M_PI * freq * currentTime + offset)
			 * ((limit_high - limit_low) / 2))
			+ ((limit_high - limit_low) / 2)
			+ limit_low);
    box = faceDetect.detect(frame);
    if (box.area() > 0)
        state = TRACKING;
}

void    trackingActions(enum eState& state,
			FaceTracker& tracker,
			cv::Mat& frame,
			cv::Rect& box,
			float dt)
{
    static float    timeout = TIMEOUT_S;
    static bool     initialized = false;
    static int      skipFrames = 0;

    if (!initialized)
    {
        timeout = TIMEOUT_S;
        if (!tracker.init(frame, box))
        {
            std::cerr << "tracker init failed" << std::endl;
            state = IDLE;
            return;
        }
        else
            initialized = true;
	skipFrames = 1;
    }
    else
    {
        if (timeout <= 0)
            std::cout << "************ Timeout ! ************" << std::endl;
        if (!tracker.update(frame, box)
            || timeout <= 0)
        {
            tracker.release();
            initialized = false;
            state = IDLE;
            return;
        }
        armCorrectPosition(frame, box, dt, skipFrames);
	timeout -= dt;
    }
}

/*
 * Returns time elapsed between two calls in seconds
 */
float	getDeltaTime()
{
  static float	prevTime = g_timer.getTime();
  float		currentTime = g_timer.getTime();
  float		dt;

  dt = currentTime - prevTime;
  prevTime = currentTime;
  return dt / 1000000.f;
}

void	initServos()
{
    if (!g_servo1.init())
      throw std::exception();
    if (!g_servo2.init())
      throw std::exception();

    g_servo1.setCWAngleLimit(0);
    g_servo1.setCCWAngleLimit(1); // Servo now set to joint mode
    g_servo1.setMovingSpeed(0.5);

    g_servo2.setCWAngleLimit(0.25);
    g_servo2.setCCWAngleLimit(0.5); // Servo now set to joint mode
    g_servo2.setMovingSpeed(0.5);

    g_servo1.presentPos();
    g_servo2.presentPos();
    usleep(100000);
}

int	main()
{
    cv::VideoCapture        cap(0);
    FaceDetector            faceDetect("resources/haarcascade_frontalface_alt.xml");
    FaceTracker		    tracker;
    cv::Mat                 frame;
    cv::Rect                box;
    enum eState             state = IDLE;
    float                   dt;
    bool                    idleReset = true;

    if (!DxlServo::devInit(0))
        errx(EXIT_FAILURE, "error: could not initialize dxl serial device %d", 0);
    if (!cap.isOpened())
        errx(EXIT_FAILURE, "error: could not open video capture");
    initServos();
    cv::namedWindow("frame", cv::WINDOW_KEEPRATIO);
    while (cv::waitKey(30) < 0)
    {
        cap >> frame;
	dt = getDeltaTime();
        switch (state)
        {
            case IDLE:
            std::cout << "idle..." << std::endl;
	    idleActions(state, faceDetect, frame, box, dt, idleReset);
                break;
            case TRACKING:
                std::cout << "tracking..." << std::endl;
                trackingActions(state, tracker, frame, box, dt);
		idleReset = true;
                break;
        }
	overlayFps(frame, dt);
        cv::imshow("frame", frame);
	plot_stats(g_gp,
		   g_timer.getTime(),
		   g_speedSetpoint,
		   g_servo2.presentSpeed(),
		   g_servo2.presentPos(),
		   g_servo1.goalPos(),
		   dt);
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
     << "'-' with lines title 'position'"
     << ", "
     << "'-' with lines title 'position command'"
     << ", "
     << "'-' with lines title 'dt'"
     << "\n";

  gp.send1d(y1Data);
  gp.send1d(y2Data);
  gp.send1d(y3Data);
  gp.send1d(y4Data);
  gp.send1d(y5Data);
}
