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
 * @brief thread safe logger.
 * @run
 */

#ifndef ZF_UTIL_STRING_H
#define ZF_UTIL_STRING_H

#include "zf_global/zf_global.h"

#define NS_STRING_UTIL string_util
#define BEGIN_NS_STRING_UTIL \
  namespace NS_STRING_UTIL   \
  {
#define END_NS_STRING_UTIL }
#define BEGIN_NS_ZF_STRING_UTIL BEGIN_NS_ZF BEGIN_NS_STRING_UTIL
#define END_NS_ZF_STRING_UTIL END_NS_STRING_UTIL END_NS_ZF
#define NS_ZF_STRING_UTIL NS_ZF::NS_STRING_UTIL


BEGIN_NS_ZF_STRING_UTIL

inline std::size_t Hash(const std::string& key) {
  return std::hash<std::string>{}(key);
}

inline bool IsSubStr(const std::string &sub, const std::string &str)
{
    return str.find(sub) != std::string::npos;
}

inline int ExtractNumberFromString(const std::string& str) {
	std::string number = "";
	for (char c : str) {
		if (std::isdigit(c)) {
			number += c;
		}
	}
	if (!number.empty()) {
		return std::stoi(number);
	}
	else {
		return -1;
	}
}

inline std::string RemoveNumberFromString(const std::string& str) {
	std::string result = "";
	for (char c : str) {
		if (!std::isdigit(c)) {
			result += c;
		}
	}
	return result;
}

inline std::vector<std::string> RemoveEmptyStringsFromList(const std::vector<std::string>& lst) {
	std::vector<std::string> result;
	for (const std::string& item : lst) {
		if (!item.empty()) {
			result.push_back(item);
		}
	}
	return result;
}

END_NS_ZF_STRING_UTIL

#endif // !ZF_UTIL_STRING_H
