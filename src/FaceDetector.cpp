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

#include "ConfigReader.hh"
#include "FaceDetector.hh"

extern ConfigReader	g_config;

FaceDetector::FaceDetector(std::string cascadeName)
{
    if (!_cascade.load(cascadeName))
        errx(EXIT_FAILURE, "error: could not load classifier cascade");
}

std::vector<cv::Rect>   FaceDetector::detectMulti(cv::Mat &frame)
{
    cv::Mat                 small(cvRound(frame.rows), cvRound(frame.cols), CV_8UC1);
    std::vector<cv::Rect>   faces;
    cv::Mat                 gray;

    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    resize(gray, small, small.size(), 0, 0, cv::INTER_LINEAR);  // Todo: optimize here
    cv::equalizeHist(gray, gray);                               // Todo: find out why that line
    _cascade.detectMultiScale(gray,
                              faces,
                              g_config("detectScaleFactor"),         // Todo: optimize
                              g_config("detectMinNeighbors"),
                              g_config("detectFlags"),               // cv::CASCADE_SCALE_IMAGE,
                              cv::Size(g_config("detectMinSizeX"),
				       g_config("detectMinSizeY"))); // Todo: optimize size ?
    return faces;
}

cv::Rect   FaceDetector::detect(cv::Mat &frame)
{
    std::vector<cv::Rect>   faces;
    cv::Rect                maxFace;
    int                     maxArea;

    faces = this->detectMulti(frame);
    if (faces.empty())
        return cv::Rect();
    maxArea = faces.front().area();
    maxFace = faces.front();
    for (auto it = faces.begin(); it != faces.end(); ++it)
    {
        if (it->area() > maxArea)
        {
            maxArea = it->area();
            maxFace = *it;
        }
    }
    return maxFace;
}
