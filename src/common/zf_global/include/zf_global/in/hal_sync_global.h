#ifndef ZF_HAL_SYNC_GLOBAL_H
#define ZF_HAL_SYNC_GLOBAL_H

#include "zf_global/common/zf_global_error_code.h"

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define HAL_SYNCHRONIZATION_EXPORT __attribute__((dllexport))
#define HAL_SYNCHRONIZATION_IMPORT __attribute__((dllimport))
#else
#define HAL_SYNCHRONIZATION_EXPORT __declspec(dllexport)
#define HAL_SYNCHRONIZATION_IMPORT __declspec(dllimport)
#endif
#ifdef HAL_SYNCHRONIZATION_BUILDING_LIBRARY
#define HAL_SYNCHRONIZATION_PUBLIC HAL_SYNCHRONIZATION_EXPORT
#else
#define HAL_SYNCHRONIZATION_PUBLIC HAL_SYNCHRONIZATION_IMPORT
#endif
#define HAL_SYNCHRONIZATION_PUBLIC_TYPE HAL_SYNCHRONIZATION_PUBLIC
#define HAL_SYNCHRONIZATION_LOCAL
#else
#define HAL_SYNCHRONIZATION_EXPORT __attribute__((visibility("default")))
#define HAL_SYNCHRONIZATION_IMPORT
#if __GNUC__ >= 4
#define HAL_SYNCHRONIZATION_PUBLIC __attribute__((visibility("default")))
#define HAL_SYNCHRONIZATION_LOCAL __attribute__((visibility("hidden")))
#else
#define HAL_SYNCHRONIZATION_PUBLIC
#define HAL_SYNCHRONIZATION_LOCAL
#endif
#define HAL_SYNCHRONIZATION_PUBLIC_TYPE
#endif

#define NS_HAL_SYNC synchronization
#define BEGIN_NS_HAL_SYNC \
  namespace NS_HAL_SYNC   \
  {
#define END_NS_HAL_SYNC }
#define BEGIN_NS_ZF_HAL_SYNC BEGIN_NS_ZF_DRIVER BEGIN_NS_HAL_SYNC // zf::driver::synchronization
#define END_NS_ZF_HAL_SYNC END_NS_ZF_DRIVER END_NS_HAL_SYNC

BEGIN_NS_ZF_HAL_SYNC

enum ERadarType
{
	RADAR_AC1000T = 0,
	VEHICLE = RADAR_AC1000T,
	RADAR_5G4T = 1,
	RADAR_IPM = 2
};

enum ESyncPolicy
{
  SYNC_NONE = 0,
  SYNC_CAMERA_5G4T = 1,
  SYNC_CAMERA_5G4T_VEHICLE = 2,
  SYNC_ALL = 3,
  SYNC_ALL_TWO_CAMS = 4,
  SYNC_CAMERA_5G4T_IPM = 100,
};

static const char gk_radar5G4TS5C0[] = "SRR_S5_C0"; // MSG NAME
static const char gk_radar5G4TS1B0[] = "SRR_S1_B0"; // MSG NAME

END_NS_ZF_HAL_SYNC

#endif /* ZF_HAL_SYNC_GLOBAL_H */
