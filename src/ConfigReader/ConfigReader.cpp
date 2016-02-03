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

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

#include "ConfigReader.hh"
#include "ExecFinder.hh"

// Public

ConfigReader::ConfigReader()
  : _servo1Id(1),
    _servo2Id(2),
    _servo1CWAngleLimit(0),
    _servo1CCWAngleLimit(1),
    _servo1MovingSpeed(0.5),
    _servo1CWComplianceMargin(0),
    _servo1CCWComplianceMargin(0),
    _servo1CWComplianceSlope(254),
    _servo1CCWComplianceSlope(254),
    _servo1Punch(0x3FF),
    _pid1P(0.0015),
    _pid1I(0),
    _pid1D(0),
    _pid1Imax(0),
    _pid1filter_hz(10),
    _servo2CWAngleLimit(0.25),
    _servo2CCWAngleLimit(0.5),
    _servo2MovingSpeed(0.5),
    _servo2CWComplianceMargin(0),
    _servo2CCWComplianceMargin(0),
    _servo2CWComplianceSlope(254),
    _servo2CCWComplianceSlope(254),
    _servo2Punch(0x3FF),
    _pid2P(0.0015),
    _pid2I(0),
    _pid2D(0),
    _pid2Imax(0),
    _pid2filter_hz(10),
    _plotMax_s(30),
    _timeout_s(10),
    _captureIndex(0),
    _servoDevIndex(0),
    _idleFreq(1.f / 30.f),
    _idleLimitLow(0),
    _idleLimitHigh(1),
    _idleStartMovingVert_s(2),
    _idleVertPos(0.45),
    _idleVertSpeed(0.015),
    _detectScaleFactor(1.1),
    _detectMinNeighbors(5),
    _detectFlags(0),
    _detectMinSizeX(30),
    _detectMinSizeY(30),
    _trackerTrackerEnabled(true),
    _trackerAlternating(false),
    _trackerLearningEnabled(true),
    _trackerDetectorCascadeVarianceFilterEnabled(true),
    _trackerDetectorCascadeEnsembleClassifierEnabled(true),
    _trackerDetectorCascadeNNClassifierEnabled(true),
    _trackerDetectorCascadeUseShift(true),
    _trackerDetectorCascadeShift(0.1),
    _trackerDetectorCascadeMinScale(-10),
    _trackerDetectorCascadeMaxScale(10),
    _trackerDetectorCascadeMinSize(25),
    _trackerDetectorCascadeNumTrees(10),
    _trackerDetectorCascadeNumFeatures(13),
    _trackerDetectorCascadeNNClassifierThetaTP(0.65),
    _trackerDetectorCascadeNNClassifierThetaFP(0.5)
{
  std::ifstream	file;
  std::string	line;
  int		n;

  if (access((ExecFinder()() + CONFIG_FILE_NAME).c_str(), R_OK) == -1)
    {
      std::cout << "Creating configuration file..." << std::endl;
      this->createConfigFile();
    }
  file.open(ExecFinder()() + CONFIG_FILE_NAME);
  if (file.is_open())
    {
      n = 1;
      while (std::getline(file, line))
	{
	  this->processLine(line, n);
	  ++n;
	}
    }
  file.close();
}

ConfigReader::~ConfigReader() {}

// Private

void    ConfigReader::createConfigFile() const
{
  std::ofstream         file;

  file.open(ExecFinder()() + CONFIG_FILE_NAME, std::ios::out);

  file << "servo1Id=" << _servo1Id << std::endl;
  file << "servo2Id=" << _servo2Id << std::endl;
  file << std::endl;
  file << "servo1CWAngleLimit=" << _servo1CWAngleLimit << std::endl;
  file << "servo1CCWAngleLimit=" << _servo1CCWAngleLimit << std::endl;
  file << "servo1MovingSpeed=" << _servo1MovingSpeed << std::endl;
  file << "servo1CWComplianceMargin=" << _servo1CWComplianceMargin << std::endl;
  file << "servo1CCWComplianceMargin=" << _servo1CCWComplianceMargin << std::endl;
  file << "servo1CWComplianceSlope=" << _servo1CWComplianceSlope << std::endl;
  file << "servo1CCWComplianceSlope=" << _servo1CCWComplianceSlope << std::endl;
  file << "servo1Punch=" << _servo1Punch << std::endl;
  file << "pid1P=" << _pid1P << std::endl;
  file << "pid1I=" << _pid1I << std::endl;
  file << "pid1D=" << _pid1D << std::endl;
  file << "pid1Imax=" << _pid1Imax << std::endl;
  file << "pid1filter_hz=" << _pid1filter_hz << std::endl;
  file << std::endl;
  file << "servo2CWAngleLimit=" << _servo2CWAngleLimit << std::endl;
  file << "servo2CCWAngleLimit=" << _servo2CCWAngleLimit << std::endl;
  file << "servo2MovingSpeed=" << _servo2MovingSpeed << std::endl;
  file << "servo2CWComplianceMargin=" << _servo2CWComplianceMargin << std::endl;
  file << "servo2CCWComplianceMargin=" << _servo2CCWComplianceMargin << std::endl;
  file << "servo2CWComplianceSlope=" << _servo2CWComplianceSlope << std::endl;
  file << "servo2CCWComplianceSlope=" << _servo2CCWComplianceSlope << std::endl;
  file << "servo2Punch=" << _servo2Punch << std::endl;
  file << "pid2P=" << _pid2P << std::endl;
  file << "pid2I=" << _pid2I << std::endl;
  file << "pid2D=" << _pid2D << std::endl;
  file << "pid2Imax=" << _pid2Imax << std::endl;
  file << "pid2filter_hz=" << _pid2filter_hz << std::endl;
  file << std::endl;
  file << "plotMax_s=" << _plotMax_s << std::endl;
  file << "timeout_s=" << _timeout_s << std::endl;
  file << std::endl;
  file << "captureIndex=" << _captureIndex << std::endl;
  file << "servoDevIndex=" << _servoDevIndex << std::endl;
  file << std::endl;
  file << "idleFreq=" << _idleFreq << std::endl;
  file << "idleLimitLow=" << _idleLimitLow << std::endl;
  file << "idleLimitHigh=" << _idleLimitHigh << std::endl;
  file << "idleStartMovingVert_s=" << _idleStartMovingVert_s << std::endl;
  file << "idleVertPos" << _idleVertPos << std::endl;
  file << "idleVertSpeed" << _idleVertSpeed << std::endl;
  file << std::endl;
  file << "detectScaleFactor=" << _detectScaleFactor << std::endl;
  file << "detectMinNeighbors=" << _detectMinNeighbors << std::endl;
  file << "detectFlags=" << _detectFlags << std::endl;
  file << "detectMinSizeX=" << _detectMinSizeX << std::endl;
  file << "detectMinSizeY=" << _detectMinSizeY << std::endl;
  file << std::endl;
  file << "trackerTrackerEnabled=" << _trackerTrackerEnabled  << std::endl;
  file << "trackerAlternating=" << _trackerAlternating  << std::endl;
  file << "trackerLearningEnabled=" << _trackerLearningEnabled  << std::endl;
  file << "trackerDetectorCascadeVarianceFilterEnabled="
       << _trackerDetectorCascadeVarianceFilterEnabled  << std::endl;
  file << "trackerDetectorCascadeEnsembleClassifierEnabled="
       << _trackerDetectorCascadeEnsembleClassifierEnabled  << std::endl;
  file << "trackerDetectorCascadeNNClassifierEnabled="
       << _trackerDetectorCascadeNNClassifierEnabled  << std::endl;
  file << "trackerDetectorCascadeUseShift=" << _trackerDetectorCascadeUseShift  << std::endl;
  file << "trackerDetectorCascadeShift=" << _trackerDetectorCascadeShift  << std::endl;
  file << "trackerDetectorCascadeMinScale=" << _trackerDetectorCascadeMinScale  << std::endl;
  file << "trackerDetectorCascadeMaxScale=" << _trackerDetectorCascadeMaxScale  << std::endl;
  file << "trackerDetectorCascadeMinSize=" << _trackerDetectorCascadeMinSize  << std::endl;
  file << "trackerDetectorCascadeNumTrees=" << _trackerDetectorCascadeNumTrees  << std::endl;
  file << "trackerDetectorCascadeNumFeatures="
       << _trackerDetectorCascadeNumFeatures  << std::endl;
  file << "trackerDetectorCascadeNNClassifierThetaTP="
       << _trackerDetectorCascadeNNClassifierThetaTP  << std::endl;
  file << "trackerDetectorCascadeNNClassifierThetaFP="
       << _trackerDetectorCascadeNNClassifierThetaFP  << std::endl;

  file.close();
}

void	ConfigReader::processLine(const std::string &line, int n)
{
  std::string	token;

  if (line[0] == '#' || line == "")
    return;
  token = line.substr(0, line.find('='));
  try {
    if (token == "servo1Id")
      _servo1Id = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "servo2Id")
      _servo2Id = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "servo1CWAngleLimit")
      _servo1CWAngleLimit = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo1CCWAngleLimit")
      _servo1CCWAngleLimit = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo1MovingSpeed")
      _servo1MovingSpeed = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo1CWComplianceMargin")
      _servo1CWComplianceMargin = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo1CCWComplianceMargin")
      _servo1CCWComplianceMargin = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo1CWComplianceSlope")
      _servo1CWComplianceSlope = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo1CCWComplianceSlope")
      _servo1CCWComplianceSlope = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo1Punch")
      _servo1Punch = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid1P")
      _pid1P = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid1I")
      _pid1I = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid1D")
      _pid1D = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid1Imax")
      _pid1Imax = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid1filter_hz")
      _pid1filter_hz = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo2CWAngleLimit")
      _servo2CWAngleLimit = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo2CCWAngleLimit")
      _servo2CCWAngleLimit = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo2MovingSpeed")
      _servo2MovingSpeed = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo2CWComplianceMargin")
      _servo2CWComplianceMargin = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo2CCWComplianceMargin")
      _servo2CCWComplianceMargin = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo2CWComplianceSlope")
      _servo2CWComplianceSlope = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo2CCWComplianceSlope")
      _servo2CCWComplianceSlope = std::stof(line.substr(line.find('=') + 1));
    else if (token == "servo2Punch")
      _servo2Punch = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid2P")
      _pid2P = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid2I")
      _pid2I = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid2D")
      _pid2D = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid2Imax")
      _pid2Imax = std::stof(line.substr(line.find('=') + 1));
    else if (token == "pid2filter_hz")
      _pid2filter_hz = std::stof(line.substr(line.find('=') + 1));
    else if (token == "plotMax_s")
      _plotMax_s = std::stof(line.substr(line.find('=') + 1));
    else if (token == "timeout_s")
      _timeout_s = std::stof(line.substr(line.find('=') + 1));
    else if (token == "captureIndex")
      _captureIndex = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "servoDevIndex")
      _servoDevIndex = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "idleFreq")
      _idleFreq = std::stof(line.substr(line.find('=') + 1));
    else if (token == "idleLimitLow")
      _idleLimitLow = std::stof(line.substr(line.find('=') + 1));
    else if (token == "idleLimitHigh")
      _idleLimitHigh = std::stof(line.substr(line.find('=') + 1));
    else if (token == "idleStartMovingVert_s")
      _idleStartMovingVert_s = std::stof(line.substr(line.find('=') + 1));
    else if (token == "idleVertPos")
      _idleVertPos = std::stof(line.substr(line.find('=') + 1));
    else if (token == "idleVertSpeed")
      _idleVertSpeed = std::stof(line.substr(line.find('=') + 1));
    else if (token == "detectScaleFactor")
      _detectScaleFactor = std::stof(line.substr(line.find('=') + 1));
    else if (token == "detectMinNeighbors")
      _detectMinNeighbors = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "detectFlags")
      _detectFlags = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "detectMinSizeX")
      _detectMinSizeX = std::stof(line.substr(line.find('=') + 1));
    else if (token == "detectMinSizeY")
      _detectMinSizeY = std::stof(line.substr(line.find('=') + 1));
    else if (token == "trackerTrackerEnabled")
      _trackerTrackerEnabled = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "trackerAlternating")
      _trackerAlternating = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "trackerLearningEnabled")
      _trackerLearningEnabled = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeVarianceFilterEnabled")
      _trackerDetectorCascadeVarianceFilterEnabled = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeEnsembleClassifierEnabled")
      _trackerDetectorCascadeEnsembleClassifierEnabled = std::stoi(line.substr(line.find('=')
									       + 1));
    else if (token == "trackerDetectorCascadeNNClassifierEnabled")
      _trackerDetectorCascadeNNClassifierEnabled = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeUseShift")
      _trackerDetectorCascadeUseShift = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeShift")
      _trackerDetectorCascadeShift = std::stof(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeMinScale")
      _trackerDetectorCascadeMinScale = std::stof(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeMaxScale")
      _trackerDetectorCascadeMaxScale = std::stof(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeMinSize")
      _trackerDetectorCascadeMinSize = std::stof(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeNumTrees")
      _trackerDetectorCascadeNumTrees = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeNumFeatures")
      _trackerDetectorCascadeNumFeatures = std::stoi(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeNNClassifierThetaTP")
      _trackerDetectorCascadeNNClassifierThetaTP = std::stof(line.substr(line.find('=') + 1));
    else if (token == "trackerDetectorCascadeNNClassifierThetaFP")
      _trackerDetectorCascadeNNClassifierThetaFP = std::stof(line.substr(line.find('=') + 1));
    else
      std::cerr << "Error in configuration file: line "
		<< n << ": invalid token '" << token << "'"
		<< std::endl;
  } catch (std::invalid_argument e) {
    std::cerr << "Error in configuration file: line "
	      << n << ": non numeric value." << std::endl;
  } catch (std::out_of_range e) {
    std::cerr << "Error in configuration file: line "
	      << n << ": out of range value." << std::endl;
  }
}
