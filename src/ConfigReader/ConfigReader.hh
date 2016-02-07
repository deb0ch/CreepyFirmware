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

# include <map>
# include <string>

# define CONFIG_FILE_NAME "config.cfg"

/*
 * Todo: handle int and string parameters
 */
class ConfigReader
{
public:
  float		operator()(std::string key) const;

  void		registerParam(std::string key, float defaultValue = 0);

public:
  ConfigReader();
  ~ConfigReader();

private:
  ConfigReader(const ConfigReader &);
  ConfigReader &operator=(const ConfigReader &);

private:
  void		initConfig();
  void		createConfigFile() const;
  void		processLine(const std::string& line, int n);

private:
  std::map<std::string, float>	_params;
};

#endif /* !CONFIGREADER_H_ */
