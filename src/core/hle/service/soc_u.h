// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <string>
#include "core/hle/service/service.h"

namespace citrs {
struct SOCUContext;
}

namespace Service {
namespace SOC {

class SOC_U final : public ServiceFramework<SOC_U> {
public:
    SOC_U();
    ~SOC_U();

private:
  void InitializeSockets(Kernel::HLERequestContext& ctx);
  void Socket(Kernel::HLERequestContext& ctx);
  void Connect(Kernel::HLERequestContext& ctx);
  void Listen(Kernel::HLERequestContext& ctx);
  void Accept(Kernel::HLERequestContext& ctx);
  void Bind(Kernel::HLERequestContext& ctx);
  // ...
  void RecvFrom(Kernel::HLERequestContext& ctx);
  // ...
  void SendTo(Kernel::HLERequestContext& ctx);
  // ...
  void Close(Kernel::HLERequestContext& ctx);
  // ...
  void Fcntl(Kernel::HLERequestContext& ctx);
  // ...
  void GetHostID(Kernel::HLERequestContext& ctx);

    citrs::SOCUContext *context;
};

void InstallInterfaces(SM::ServiceManager& service_manager);

} // namespace SOC
} // namespace Service
