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

#ifndef CONFIGREADER_H_
# define CONFIGREADER_H_

# include <string>

# define CONFIG_FILE_NAME "config.cfg"

class ConfigReader
{
public:
  int		servo1Id() const			{ return _servo1Id; }
  int		servo2Id() const			{ return _servo2Id; }

  float		servo1CWAngleLimit() const		{ return _servo1CWAngleLimit; }
  float		servo1CCWAngleLimit() const		{ return _servo1CCWAngleLimit; }
  float		servo1MovingSpeed() const		{ return _servo1MovingSpeed; }
  float		servo1CWComplianceMargin() const	{ return _servo1CWComplianceMargin; }
  float		servo1CCWComplianceMargin() const	{ return _servo1CCWComplianceMargin; }
  float		servo1CWComplianceSlope() const		{ return _servo1CWComplianceSlope; }
  float		servo1CCWComplianceSlope() const	{ return _servo1CCWComplianceSlope; }
  float		servo1Punch() const			{ return _servo1Punch; }
  float		pid1P() const				{ return _pid1P; }
  float		pid1I() const				{ return _pid1I; }
  float		pid1D() const				{ return _pid1D; }
  float		pid1Imax() const			{ return _pid1Imax; }
  float		pid1filter_hz() const			{ return _pid1filter_hz; }

  float		servo2CWAngleLimit() const		{ return _servo2CWAngleLimit; }
  float		servo2CCWAngleLimit() const		{ return _servo2CCWAngleLimit; }
  float		servo2MovingSpeed() const		{ return _servo2MovingSpeed; }
  float		servo2CWComplianceMargin() const	{ return _servo2CWComplianceMargin; }
  float		servo2CCWComplianceMargin() const	{ return _servo2CCWComplianceMargin; }
  float		servo2CWComplianceSlope() const		{ return _servo2CWComplianceSlope; }
  float		servo2CCWComplianceSlope() const	{ return _servo2CCWComplianceSlope; }
  float		servo2Punch() const			{ return _servo2Punch; }
  float		pid2P() const				{ return _pid2P; }
  float		pid2I() const				{ return _pid2I; }
  float		pid2D() const				{ return _pid2D; }
  float		pid2Imax() const			{ return _pid2Imax; }
  float		pid2filter_hz() const			{ return _pid2filter_hz; }

  float		plotMax_s() const			{ return _plotMax_s; }
  float		timeout_s() const			{ return _timeout_s; }

  int		captureIndex() const			{ return _captureIndex; }
  int		servoDevIndex() const			{ return _servoDevIndex; }

  float		idleFreq() const			{ return _idleFreq; }
  float		idleLimitLow() const			{ return _idleLimitLow; }
  float		idleLimitHigh() const			{ return _idleLimitHigh; }
  float		idleStartMovingVert_s() const		{ return _idleStartMovingVert_s; }
  float		idleVertPos() const			{ return _idleVertPos; }
  float		idleVertSpeed() const			{ return _idleVertSpeed; }

  double	detectScaleFactor() const		{ return _detectScaleFactor; }
  int		detectMinNeighbors() const		{ return _detectMinNeighbors; }
  int		detectFlags() const			{ return _detectFlags; }
  int		detectMinSizeX() const			{ return _detectMinSizeX; }
  int		detectMinSizeY() const			{ return _detectMinSizeY; }

  int		trackerTrackerEnabled() const				{ return _trackerTrackerEnabled; }
  int		trackerAlternating() const				{ return _trackerAlternating; }
  int		trackerLearningEnabled() const				{ return _trackerLearningEnabled; }
  int		trackerDetectorCascadeVarianceFilterEnabled() const	{ return _trackerDetectorCascadeVarianceFilterEnabled; }
  int		trackerDetectorCascadeEnsembleClassifierEnabled() const	{ return _trackerDetectorCascadeEnsembleClassifierEnabled; }
  int		trackerDetectorCascadeNNClassifierEnabled() const	{ return _trackerDetectorCascadeNNClassifierEnabled; }
  int		trackerDetectorCascadeUseShift() const			{ return _trackerDetectorCascadeUseShift; }
  float		trackerDetectorCascadeShift() const			{ return _trackerDetectorCascadeShift; }
  float		trackerDetectorCascadeMinScale() const			{ return _trackerDetectorCascadeMinScale; }
  float		trackerDetectorCascadeMaxScale() const			{ return _trackerDetectorCascadeMaxScale; }
  float		trackerDetectorCascadeMinSize() const			{ return _trackerDetectorCascadeMinSize; }
  int		trackerDetectorCascadeNumTrees() const			{ return _trackerDetectorCascadeNumTrees; }
  int		trackerDetectorCascadeNumFeatures() const		{ return _trackerDetectorCascadeNumFeatures; }
  float		trackerDetectorCascadeNNClassifierThetaTP() const	{ return _trackerDetectorCascadeNNClassifierThetaTP; }
  float		trackerDetectorCascadeNNClassifierThetaFP() const	{ return _trackerDetectorCascadeNNClassifierThetaFP; }

public:
  ConfigReader();
  ~ConfigReader();

private:
  ConfigReader(const ConfigReader &);
  ConfigReader &operator=(const ConfigReader &);

private:
  void		createConfigFile() const;
  void		processLine(const std::string& line, int n);

private:
  int		_servo1Id;
  int		_servo2Id;

  float		_servo1CWAngleLimit;
  float		_servo1CCWAngleLimit;
  float		_servo1MovingSpeed;
  float		_servo1CWComplianceMargin;
  float		_servo1CCWComplianceMargin;
  float		_servo1CWComplianceSlope;
  float		_servo1CCWComplianceSlope;
  float		_servo1Punch;
  float		_pid1P;
  float		_pid1I;
  float		_pid1D;
  float		_pid1Imax;
  float		_pid1filter_hz;

  float		_servo2CWAngleLimit;
  float		_servo2CCWAngleLimit;
  float		_servo2MovingSpeed;
  float		_servo2CWComplianceMargin;
  float		_servo2CCWComplianceMargin;
  float		_servo2CWComplianceSlope;
  float		_servo2CCWComplianceSlope;
  float		_servo2Punch;
  float		_pid2P;
  float		_pid2I;
  float		_pid2D;
  float		_pid2Imax;
  float		_pid2filter_hz;

  float		_plotMax_s;
  float		_timeout_s;

  int		_captureIndex;
  int		_servoDevIndex;

  float		_idleFreq;
  float		_idleLimitLow;
  float		_idleLimitHigh;
  float		_idleStartMovingVert_s;
  float		_idleVertPos;
  float		_idleVertSpeed;

  double	_detectScaleFactor;
  int		_detectMinNeighbors;
  int		_detectFlags;
  int		_detectMinSizeX;
  int		_detectMinSizeY;

  int		_trackerTrackerEnabled;
  int		_trackerAlternating;
  int		_trackerLearningEnabled;
  int		_trackerDetectorCascadeVarianceFilterEnabled;
  int		_trackerDetectorCascadeEnsembleClassifierEnabled;
  int		_trackerDetectorCascadeNNClassifierEnabled;
  int		_trackerDetectorCascadeUseShift;
  float		_trackerDetectorCascadeShift;
  float		_trackerDetectorCascadeMinScale;
  float		_trackerDetectorCascadeMaxScale;
  float		_trackerDetectorCascadeMinSize;
  int		_trackerDetectorCascadeNumTrees;
  int		_trackerDetectorCascadeNumFeatures;
  float		_trackerDetectorCascadeNNClassifierThetaTP;
  float		_trackerDetectorCascadeNNClassifierThetaFP;
};

#endif /* !CONFIGREADER_H_ */
