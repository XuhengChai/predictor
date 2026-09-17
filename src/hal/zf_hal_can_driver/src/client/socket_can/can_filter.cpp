#include "zf_hal_can_driver/client/socket_can/can_filter.h"

#include <sstream>
#include <sys/socket.h>
#include <linux/can/raw.h>

BEGIN_NS_ZF_DRIVER_CANBUS

CanFilterList::CanFilterList(const char *str)
{
    SetFilters(str);
}

CanFilterList::CanFilterList(const std::string &str)
{
    // *this = ParseFilters(str);
    SetFilters(str);
}

////////////////////////////////////////////////////////////////////////////////
CanFilterList CanFilterList::ParseFilters(const std::string &str)
{
    CanFilterList filter_list;
    filter_list.SetFilters(str);
    return filter_list;
}

void CanFilterList::SetFilters(
    const std::string &str)
{
    this->m_errorMask = 0;
    this->m_bJoinFilters = false;
    this->m_vFilters.clear();

    std::istringstream input(str);
    std::string fstr;

    while (getline(input, fstr, ','))
    {
        // trim leading and trailing whitespaces
        fstr = fstr.substr(
            fstr.find_first_not_of(" \t"),
            fstr.find_last_not_of(" \t") - fstr.find_first_not_of(" \t") + 1);

        struct can_filter filter;
        if (std::sscanf(fstr.c_str(), "%x:%x", &filter.can_id, &filter.can_mask) == 2)
        {
            filter.can_mask &= ~CAN_ERR_FLAG;
            if (fstr.size() > 8 && fstr[8] == ':')
            {
                filter.can_id |= CAN_EFF_FLAG;
            }
            this->m_vFilters.push_back(filter);
        }
        else if (std::sscanf(fstr.c_str(), "%x~%x", &filter.can_id, &filter.can_mask) == 2)
        {
            filter.can_id |= CAN_INV_FILTER;
            filter.can_mask &= ~CAN_ERR_FLAG;
            if (fstr.size() > 8 && fstr[8] == '~')
            {
                filter.can_id |= CAN_EFF_FLAG;
            }
            this->m_vFilters.push_back(filter);
        }
        else if (fstr == "j" || fstr == "J")
        {
            this->m_bJoinFilters = true;
        }
        else if (std::sscanf(fstr.c_str(), "#%x", &this->m_errorMask) != 1)
        {
            LOG_ERROR()<<("Error during filter parsing: " + fstr).c_str();
            // throw std::runtime_error("Error during filter parsing: " + fstr);
        }
    }
}

void CanFilterList::SetFilters(int32_t fd, const std::string &str)
{
    SetFilters(str);
    SetCanFilter(fd, m_vFilters);
    SetCanErrFilter(fd, m_errorMask);
    SetCanFilterJoin(fd, m_bJoinFilters);
}
int CanFilterList::SetCanFilter(int32_t fd)
{
    return SetCanFilter(fd, m_vFilters);
}

int CanFilterList::SetCanErrFilter(int32_t fd)
{
    return SetCanErrFilter(fd, m_errorMask);
}

int CanFilterList::SetCanFilterJoin(int32_t fd)
{
    return SetCanFilterJoin(fd, m_bJoinFilters);
}

int CanFilterList::SetCanFilter(int32_t fd, const std::vector<struct can_filter> &f_list)
{
    int ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, f_list.empty() ? NULL : f_list.data(),
                         sizeof(can_filter) * f_list.size());
    if (ret < 0)
    {
        LOG_ERROR() << "Failed to set up CAN filters: ";
    }
    return ret;
}

int CanFilterList::SetCanErrFilter(int32_t fd, can_err_mask_t err_mask)
{
    int ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask,
                         sizeof(err_mask));
    if (ret < 0)
    {
        LOG_ERROR() << "Failed to set up CAN error filters: ";
    }
    return ret;
}

int CanFilterList::SetCanFilterJoin(int32_t fd, bool join_filters)
{
    auto join = static_cast<int>(join_filters);
    int ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS, &join,
                         sizeof(join));
    if (ret < 0)
    {
        LOG_ERROR() << "Failed to set up joined CAN filters: ";
    }
    return ret;
}

END_NS_ZF_DRIVER_CANBUS