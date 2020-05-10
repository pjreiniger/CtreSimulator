
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/BuffTrajPointStream_CCI.h"

namespace SnobotSim
{

class CtreBuffTrajPointStreamWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    CtreBuffTrajPointStreamWrapper();

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);

    //////////////////////////////////////////
    void Clear();
    void Write(double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos, uint32_t timeDur, bool useAuxPID);
    void Lookup(void** outObject);
};

} // namespace SnobotSim
