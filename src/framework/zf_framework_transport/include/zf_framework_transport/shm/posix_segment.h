/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef ZF_TRANSPORT_SHM_POSIX_SEGMENT_H_
#define ZF_TRANSPORT_SHM_POSIX_SEGMENT_H_

#include <string>

#include "zf_framework_transport/shm/segment.h"

BEGIN_NS_ZF_FRAMEWORK

class PosixSegment : public Segment {
 public:
  explicit PosixSegment(uint64_t channel_id);
  virtual ~PosixSegment();

  static const char* Type() { return "posix"; }

protected:
  virtual bool CreateShm(int shmid) override;
  virtual bool OpenShm(int shmid) override;
  virtual void Reset() override;
  virtual bool Remove() override;
  virtual bool OpenOnly() override;
  virtual bool OpenOrCreate() override;
};

END_NS_ZF_FRAMEWORK

#endif  // ZF_TRANSPORT_SHM_POSIX_SEGMENT_H_
