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
{
  this->registerParam("servo1Id", 1);
  this->registerParam("servo2Id", 2);
  this->registerParam("servo1CWAngleLimit", 0);
  this->registerParam("servo1CCWAngleLimit", 1);
  this->registerParam("servo1MovingSpeed", 0.5);
  this->registerParam("servo1CWComplianceMargin", 0);
  this->registerParam("servo1CCWComplianceMargin", 0);
  this->registerParam("servo1CWComplianceSlope", 254);
  this->registerParam("servo1CCWComplianceSlope", 254);
  this->registerParam("servo1Punch", 0x3FF);
  this->registerParam("pid1P", 0.0015);
  this->registerParam("pid1I", 0);
  this->registerParam("pid1D", 0);
  this->registerParam("pid1Imax", 0);
  this->registerParam("pid1filter_hz", 10);
  this->registerParam("servo2CWAngleLimit", 0.25);
  this->registerParam("servo2CCWAngleLimit", 0.5);
  this->registerParam("servo2MovingSpeed", 0.5);
  this->registerParam("servo2CWComplianceMargin", 0);
  this->registerParam("servo2CCWComplianceMargin", 0);
  this->registerParam("servo2CWComplianceSlope", 254);
  this->registerParam("servo2CCWComplianceSlope", 254);
  this->registerParam("servo2Punch", 0x3FF);
  this->registerParam("pid2P", 0.0015);
  this->registerParam("pid2I", 0);
  this->registerParam("pid2D", 0);
  this->registerParam("pid2Imax", 0);
  this->registerParam("pid2filter_hz", 10);
  this->registerParam("plotMax_s", 30);
  this->registerParam("timeout_s", 10);
  this->registerParam("captureIndex", 0);
  this->registerParam("servoDevIndex", 0);
  this->registerParam("idleFreq", 1.f / 30.f);
  this->registerParam("idleLimitLow", 0);
  this->registerParam("idleLimitHigh", 1);
  this->registerParam("idleStartMovingVert_s", 2);
  this->registerParam("idleVertPos", 0.45);
  this->registerParam("idleVertSpeed", 0.015);
  this->registerParam("detectScaleFactor", 1.1);
  this->registerParam("detectMinNeighbors", 5);
  this->registerParam("detectFlags", 0);
  this->registerParam("detectMinSizeX", 30);
  this->registerParam("detectMinSizeY", 30);
  this->registerParam("trackerTrackerEnabled", true);
  this->registerParam("trackerAlternating", false);
  this->registerParam("trackerLearningEnabled", true);
  this->registerParam("trackerDetectorCascadeVarianceFilterEnabled", true);
  this->registerParam("trackerDetectorCascadeEnsembleClassifierEnabled", true);
  this->registerParam("trackerDetectorCascadeNNClassifierEnabled", true);
  this->registerParam("trackerDetectorCascadeUseShift", true);
  this->registerParam("trackerDetectorCascadeShift", 0.1);
  this->registerParam("trackerDetectorCascadeMinScale", -10);
  this->registerParam("trackerDetectorCascadeMaxScale", 10);
  this->registerParam("trackerDetectorCascadeMinSize", 25);
  this->registerParam("trackerDetectorCascadeNumTrees", 10);
  this->registerParam("trackerDetectorCascadeNumFeatures", 13);
  this->registerParam("trackerDetectorCascadeNNClassifierThetaTP", 0.65);
  this->registerParam("trackerDetectorCascadeNNClassifierThetaFP", 0.5);
  this->initConfig();
}

ConfigReader::~ConfigReader() {}

float	ConfigReader::operator()(std::string key) const
{
  if (_params.find(key) == _params.end())
    throw std::exception();
  return _params.find(key)->second;
}

void	ConfigReader::registerParam(std::string key, float defaultValue)
{
  _params[key] = defaultValue;
}

// Private

void	ConfigReader::initConfig()
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

void    ConfigReader::createConfigFile() const
{
  std::ofstream         file;

  file.open(ExecFinder()() + CONFIG_FILE_NAME, std::ios::out);

  for (auto it = _params.begin(); it != _params.end(); ++it)
    {
      file << it->first << "="
	   << it->second  << std::endl;
    }
  file.close();
}

/*
 * Todo: trim spacing chars
 */
void	ConfigReader::processLine(const std::string &line, int n)
{
  std::string	token;

  if (line[0] == '#' || line == "")
    return;
  token = line.substr(0, line.find('='));
  try {
    if (_params.find(token) != _params.end())
      _params[token] = std::stof(line.substr(line.find('=') + 1));
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
