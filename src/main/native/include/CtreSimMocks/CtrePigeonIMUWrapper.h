
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"

namespace SnobotSim
{

class CtrePigeonIMUWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    explicit CtrePigeonIMUWrapper(int aDeviceId);
    const int mDeviceId;

    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
};

} // namespace SnobotSim
