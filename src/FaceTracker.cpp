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

#include "FaceTracker.hh"

FaceTracker::FaceTracker() {}

FaceTracker::~FaceTracker() {}

bool	FaceTracker::init(cv::Mat& frame, cv::Rect& ref)
{
  cv::Mat	grey(frame.rows, frame.cols, CV_8UC1);

  cv::cvtColor(frame, grey, CV_BGR2GRAY);

  _tracker.trackerEnabled = true;
  _tracker.alternating = false;
  _tracker.learningEnabled = true;

  _tracker.detectorCascade->varianceFilter->enabled = true;
  _tracker.detectorCascade->ensembleClassifier->enabled = true;
  _tracker.detectorCascade->nnClassifier->enabled = true;
  _tracker.detectorCascade->useShift = true;
  _tracker.detectorCascade->shift = 0.1;
  _tracker.detectorCascade->minScale = -10;
  _tracker.detectorCascade->maxScale = 10;
  _tracker.detectorCascade->minSize = 25;
  _tracker.detectorCascade->numTrees = 10;
  _tracker.detectorCascade->numFeatures = 13;
  _tracker.detectorCascade->nnClassifier->thetaTP = 0.65;
  _tracker.detectorCascade->nnClassifier->thetaFP = 0.5;

  _tracker.detectorCascade->imgWidth = grey.cols;
  _tracker.detectorCascade->imgHeight = grey.rows;
  _tracker.detectorCascade->imgWidthStep = grey.step;

  _tracker.selectObject(grey, &ref);

  return this->update(frame, ref);
}

bool	FaceTracker::update(cv::Mat& frame, cv::Rect& ref)
{
  _tracker.processImage(frame);
  if (_tracker.currBB != NULL)
  {
    ref = *_tracker.currBB;
  }
  else
    ref = cv::Rect();
  return _tracker.currConf != 0;
}

void	FaceTracker::release()
{
  _tracker.release();
}
