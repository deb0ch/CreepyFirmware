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

#ifdef _WIN32

# include <Windows.h>

#elif __linux__

# define _XOPEN_SOURCE 500
# include <unistd.h>

#endif /* !__linux__ */

#include "ExecFinderException.hh"
#include "ExecFinder.hh"

// Public

#ifdef _WIN32
# define BUFFSIZE 1024

const std::string &	ExecFinder::operator()()
{
  if (_path == "")
  {
    char	buf[BUFFSIZE];
    int		ret;

    if ((ret = GetModuleFileName(NULL, buf, BUFFSIZE - 1)) == 0)
      throw ExecFinderException("readlink error: ", errno);
    buf[ret] = '\0';
    _path = buf;
    _path = _path.substr(0, _path.rfind('/') + 1);
  }
  return (_path);
}

#elif __linux__
# define BUFFSIZE 1024

const std::string &	ExecFinder::operator()()
{
  if (_path == "")
  {
    char	buf[BUFFSIZE];
    int		ret;

    if ((ret = readlink("/proc/self/exe", buf, BUFFSIZE - 1)) == -1)
      throw ExecFinderException("readlink error: ", errno);
    buf[ret] = '\0';
    _path = buf;
    _path = _path.substr(0, _path.rfind('/') + 1);
  }
  return (_path);
}
#endif /* !__linux__ */

ExecFinder::ExecFinder()
  : _path("")
{}

ExecFinder::~ExecFinder() {}

// Private
