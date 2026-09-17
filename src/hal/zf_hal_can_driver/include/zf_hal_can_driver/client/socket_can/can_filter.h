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
 * @brief Can filter.
 * @run
 */

#ifndef ZF_HAL_CAN_FILTER_H
#define ZF_HAL_CAN_FILTER_H

#include "zf_hal_can_driver/hal_can_driver_global.h"

#include <linux/can.h>
#include <array>
#include <cstring>
#include <string>
#include <vector>

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

class CanFilterList
{
public:
    /**
     * @brief Default constructor
     */
    CanFilterList() = default;

    /**
     * @brief \copydoc ParseFilters(const std::string & str)
     */
    explicit CanFilterList(const char *str);

    /**
     * @brief \copydoc ParseFilters(const std::string & str)
     */
    explicit CanFilterList(const std::string &str);

    /// Parse CAN filters string:\n
    /// Filters:\n
    /// Comma separated filters can be specified for each given CAN interface.\n
    /// <can_id>:<can_mask>\n
    ///         (matches when <received_can_id> & mask == can_id & mask)\n
    /// <can_id>~<can_mask>\n
    ///         (matches when <received_can_id> & mask != can_id & mask)\n
    /// #<error_mask>\n
    ///         (set error frame filter, see include/linux/can/error.h)\n
    /// [j|J]\n
    ///         (join the given CAN filters - logical AND semantic)\n
    ///
    /// CAN IDs, masks and data content are given and expected in hexadecimal values.
    /// When can_id and can_mask are both 8 digits, they are assumed to be 29 bit EFF.
    /// \see https://manpages.ubuntu.com/manpages/jammy/man1/candump.1.html
    /// \param[in] str Input to be parsed.
    /// \return Populated CanFilterList structure.
    /// \throw std::runtime_error if string couldn't be parsed.
    static CanFilterList ParseFilters(const std::string &str);

    void SetFilters(const std::string &str);
    void SetFilters(int32_t fd, const std::string &str);

    /**
     * @brief \copydoc Set SocketCAN filters
     * @param[in] fd File descriptor of the socket
     * @param[in] f_list List of filters to be applied.
     */
    int SetCanFilter(int32_t fd, const std::vector<struct can_filter> &f_list);
    int SetCanFilter(int32_t fd);

    /**
     * @brief \copydoc Set SocketCAN error filters
     * @param[in] fd File descriptor of the socket
     * @param[in] err_mask Error mask to be applied as a filter
     */
    int SetCanErrFilter(int32_t fd, can_err_mask_t err_mask);
    int SetCanErrFilter(int32_t fd);

    /**
     * @brief \copydoc Set filters joining option for SocketCAN. If set, all filters
     * @param[in] fd File descriptor of the socket
     * @param[in] join_filters Should the filters be joined?
     */
    int SetCanFilterJoin(int32_t fd, bool join_filters);
    int SetCanFilterJoin(int32_t fd);

private:
    std::vector<struct can_filter> m_vFilters;
    can_err_mask_t m_errorMask = 0;
    bool m_bJoinFilters = false;
};

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_CAN_FILTER_H