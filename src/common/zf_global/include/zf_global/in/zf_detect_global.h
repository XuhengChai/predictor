#ifndef ZF_DETECTION_GLOBAL_H
#define ZF_DETECTION_GLOBAL_H

#include "zf_global/zf_global.h"

#define NS_DETECTION                            detect
#define BEGIN_NS_DETECTION                      namespace NS_DETECTION{
#define END_NS_DETECTION                        }
#define NS_ZF_DETECTION                         NS_ZF::NS_DETECTION //zf::driver::DETECTION
#define BEGIN_NS_ZF_DETECTION                   BEGIN_NS_ZF BEGIN_NS_DETECTION //zf::driver::DETECTION
#define END_NS_ZF_DETECTION                     END_NS_ZF END_NS_DETECTION

#endif /* ZF_DETECTION_GLOBAL_H */
