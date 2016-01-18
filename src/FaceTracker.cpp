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

FaceTracker::FaceTracker()
{
  _tracker = cv::Tracker::create(TRACKER_TYPE);    //  "MIL", "BOOSTING", "MEDIANFLOW", "TLD"
}

FaceTracker::~FaceTracker()
{
  _tracker.release();
}

bool	FaceTracker::init(cv::Mat& frame, cv::Rect& ref)
{
  bool	ret;

  _box2d = ref;
  ret = _tracker->init(frame, _box2d);
  ref = _box2d;
  return ret;
}

bool	FaceTracker::update(cv::Mat& frame, cv::Rect& ref)
{
  bool	ret;

  _box2d = ref;
  ret = _tracker->update(frame, _box2d);
  ref = _box2d;
  return ret;
}

void	FaceTracker::release()
{
  _tracker.release();
  _tracker = cv::Tracker::create(TRACKER_TYPE);
}
