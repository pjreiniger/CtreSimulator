
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"

namespace SnobotSim
{

class CtreCANCoderWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    CtreCANCoderWrapper(int aDeviceId);

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);
};

} // namespace SnobotSim
