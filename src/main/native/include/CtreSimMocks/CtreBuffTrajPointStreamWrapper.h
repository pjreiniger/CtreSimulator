
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"

namespace SnobotSim
{

class CtreBuffTrajPointStreamWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    CtreBuffTrajPointStreamWrapper();

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);
};

} // namespace SnobotSim
