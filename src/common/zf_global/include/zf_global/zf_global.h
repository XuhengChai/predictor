/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/**
 * @file
 * @brief Global defines.
 * @return
 */

#ifndef ZF_GLOBAL_H
#define ZF_GLOBAL_H

#include <string>
#include <vector>
#include <list>
#include <iostream>

#define ZF_DECL_EXPORT __declspec(dllexport)
#define ZF_DECL_IMPORT __declspec(dllimport)

#define NS_ZF zf
#define BEGIN_NS_ZF \
  namespace NS_ZF   \
  {
#define END_NS_ZF }

#define NS_INTERNAL pvt
#define NS_IN NS_INTERNAL
#define BEGIN_NS_INTERNAL \
  namespace NS_INTERNAL   \
  {
#define END_NS_INTERNAL }
#define BEGIN_NS_ZF_INTERNAL BEGIN_NS_ZF BEGIN_NS_INTERNAL
#define END_NS_ZF_INTERNAL END_NS_INTERNAL END_NS_ZF

#define NS_DRIVER driver
#define BEGIN_NS_DRIVER \
  namespace NS_DRIVER   \
  {
#define END_NS_DRIVER }
#define BEGIN_NS_ZF_DRIVER BEGIN_NS_ZF BEGIN_NS_DRIVER
#define END_NS_ZF_DRIVER END_NS_DRIVER END_NS_ZF

BEGIN_NS_ZF

typedef wchar_t WChar;
typedef signed char SChar;
typedef unsigned char UChar;
typedef unsigned short UShort;
typedef unsigned int UInt;
typedef float Single;
typedef double Double;
typedef std::size_t USize;
typedef std::int8_t Int8;
typedef std::uint8_t UInt8;
typedef std::int16_t Int16;
typedef std::uint16_t UInt16;
typedef std::int32_t Int32;
typedef std::uint32_t UInt32;
typedef std::int64_t Int64;
typedef std::uint64_t UInt64;
typedef Single Float32;
typedef Double Float64;

using StringList = std::list<std::string>;
using StringVector = std::vector<std::string>;
using SingleVector = std::vector<Single>;
using UIntVector = std::vector<UInt>;

#define FONT_COLOR_RED "\033[31m"
#define FONT_COLOR_GREEN "\033[32m"
#define FONT_COLOR_YELLOW "\033[33m"
#define FONT_COLOR_DEFAULT "\033[0m"// close all

class newline_writer
    : public std::ostream
{
  bool need_newline = true;
public:
  newline_writer(std::streambuf *sbuf)
      : std::ios(sbuf), std::ostream(sbuf)
  {
  }
  newline_writer(newline_writer &&other)
      : newline_writer(other.rdbuf())
  {
    other.need_newline = false;
  }
  ~newline_writer() { this->need_newline &&*this << FONT_COLOR_DEFAULT << '\n'; }
};

// newline_writer LOG_SCREEN()
// {
//   return newline_writer(std::cout.rdbuf());
// }

#define LOG_ERROR() newline_writer(std::cout.rdbuf()) << FONT_COLOR_RED << "[ERROR]: "
#define LOG_WARN() newline_writer(std::cout.rdbuf()) << FONT_COLOR_YELLOW << "[WARN]: "
#define LOG_INFO() newline_writer(std::cout.rdbuf()) << "[INFO ]: "
#define LOG_DEBUG() newline_writer(std::cout.rdbuf()) << "[DEBUG]: "

#define CHECK(x) \
  if (!(x))      \
  LOG_INFO() << "Check failed: " << #x

END_NS_ZF

#endif // !ZF_GLOBAL_H
