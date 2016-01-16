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

#ifndef PIDCONTROL_H_
# define PIDCONTROL_H_

# include <math.h>

# define PID_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
# define PID_FILT_HZ_MIN      0.01f   // minimum input filter frequency

# define FLOAT_EPSILON 1.1920929e-07F

static inline bool is_zero(const float val)
{
  return ::fabsf(val) < FLOAT_EPSILON;
}

class PIDControl
{
public:
  PIDControl(float p,
	     float i,
	     float d,
	     float imax,
	     float input_filter_hz);

  void	init(float p,
	     float i,
	     float d,
	     float imax,
	     float input_filter_hz);

  // should be called before any other calls to get_p, get_i, get_d, get_pi or get_pid
  void	set_dt(float dt);                  // dt in seconds
  void	set_input_filter_all(float input); // filter all components
  void	set_input_filter_d(float input);   // filter only derivative component

  // get results from controller
  float	get_pid();
  float	get_p();
  float	get_i();
  float	get_d();

  void	reset_I();	// reset integrator
  void	reset_filter();

  float	get_kP() const			{ return _kp; }
  void	set_kP(const float v)		{ _kp = v; }

  float	get_kI() const			{ return _ki; }
  void	set_kI(const float v)		{ _ki = v; }

  float	get_kD() const			{ return _kd; }
  void	set_kD(const float v)		{ _kd = v; }

  float	get_imax() const		{ return _imax; }
  void	set_imax(const float v)		{ _imax = fabsf(v); }

  float	get_filt_hz() const		{ return _filt_hz; }
  void	set_filt_hz(const float v);

  float	get_filt_alpha() const;

  float	get_integrator() const		{ return _integrator; }
  void	set_integrator(float i)		{ _integrator = i; }

protected:
  float	_kp;
  float	_ki;
  float	_kd;
  float	_imax;
  float	_filt_hz;

  bool	_reset_filter;    // true when input filter should be reset during next call to set_input_*

  float	_dt;              // timestep in seconds
  float	_integrator;      // integrator value
  float	_input;           // last input for derivative
  float	_derivative;      // last derivative for low-pass filter
};

#endif // PIDCONTROL_H_
