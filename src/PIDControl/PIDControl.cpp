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

#include "PIDControl.hh"

PIDControl::PIDControl(float p,
                       float i,
                       float d,
                       float imax,
                       float input_filter_hz)
  : _dt(0),
    _integrator(0.0f),
    _input(0.0f),
    _derivative(0.0f)
{
  _kp = p;
  _ki = i;
  _kd = d;
  _imax = fabsf(imax);
  this->set_filt_hz(input_filter_hz);
  _reset_filter = true;    // reset input filter to first value received
}

void PIDControl::init(float p,
		      float i,
		      float d,
		      float imax,
		      float input_filter_hz)
{
  _kp = p;
  _ki = i;
  _kd = d;
  _imax = ::fabsf(imax);
  _filt_hz = input_filter_hz;
}

void PIDControl::set_dt(float dt)
{
  _dt = dt;
}

void PIDControl::set_filt_hz(float freq_hz)
{
  _filt_hz = ::fabsf(freq_hz);
  _filt_hz = ::fmax(_filt_hz, PID_FILT_HZ_MIN); // sanity check
}

void PIDControl::set_input_filter_all(float input)
{
  if (!isfinite(input))
    return;
  if (_reset_filter)
    {
      _reset_filter = false;
      _input = input;
      _derivative = 0.0f;
    }

  float input_filt_change = this->get_filt_alpha() * (input - _input);

  _input = _input + input_filt_change;
  if (_dt > 0.0f)
    _derivative = input_filt_change / _dt;
}

void PIDControl::set_input_filter_d(float input)
{
  if (!isfinite(input))
    return;
  if (_reset_filter)
    {
      _reset_filter = false;
      _derivative = 0.0f;
    }
  if (_dt > 0.0f)
    {
      float derivative = (input - _input) / _dt;

      _derivative = _derivative + this->get_filt_alpha() * (derivative - _derivative);
    }
  _input = input;
}

float PIDControl::get_p()
{
  return _input * _kp;
}

float PIDControl::get_i()
{
  _integrator += _input * _dt * _ki;
  if (_integrator < -_imax)
    _integrator = -_imax;
  else if (_integrator > _imax)
    _integrator = _imax;
  return _integrator;
}

float PIDControl::get_d()
{
  return _kd * _derivative;
}

float PIDControl::get_pid()
{
  return this->get_p() + this->get_i() + this->get_d();
}

void PIDControl::reset_I()
{
  _integrator = 0;
}

float PIDControl::get_filt_alpha() const
{
  if (is_zero(_filt_hz))
    return 1.0f;

  float rc = 1 / (2 * M_PI * _filt_hz);

  return _dt / (_dt + rc);
}
