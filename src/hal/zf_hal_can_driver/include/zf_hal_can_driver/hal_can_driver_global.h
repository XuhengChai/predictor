#include "zf_global/common/zf_global_error_code.h"

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HAL_CAN_DRIVER_EXPORT __attribute__ ((dllexport))
    #define HAL_CAN_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define HAL_CAN_DRIVER_EXPORT __declspec(dllexport)
    #define HAL_CAN_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef HAL_CAN_DRIVER_BUILDING_LIBRARY
    #define HAL_CAN_DRIVER_PUBLIC HAL_CAN_DRIVER_EXPORT
  #else
    #define HAL_CAN_DRIVER_PUBLIC HAL_CAN_DRIVER_IMPORT
  #endif
  #define HAL_CAN_DRIVER_PUBLIC_TYPE HAL_CAN_DRIVER_PUBLIC
  #define HAL_CAN_DRIVER_LOCAL
#else
  #define HAL_CAN_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define HAL_CAN_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define HAL_CAN_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define HAL_CAN_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HAL_CAN_DRIVER_PUBLIC
    #define HAL_CAN_DRIVER_LOCAL
  #endif
  #define HAL_CAN_DRIVER_PUBLIC_TYPE
#endif

#define NS_CANBUS                       canbus
#define BEGIN_NS_CANBUS                 namespace NS_CANBUS{
#define END_NS_CANBUS                   }
#define BEGIN_NS_ZF_DRIVER_CANBUS       BEGIN_NS_ZF_DRIVER BEGIN_NS_CANBUS //zf::driver::canbus
#define END_NS_ZF_DRIVER_CANBUS         END_NS_ZF_DRIVER END_NS_CANBUS

// template <typename T>
// std::ostream& printMsg(std::ostream& os, T&& t) {
//     return os << std::forward<T>(t);
// }
// #define PRINT_MSG   []() -> newline_writer  {                                
//   return newline_writer(std::cout.rdbuf()); 
//   }
BEGIN_NS_ZF_DRIVER_CANBUS

#define CHECK_EQ(x, y) CHECK((x) == (y))

END_NS_ZF_DRIVER_CANBUS