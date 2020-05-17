
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/BuffTrajPointStream_CCI.h"
#include "simulation/SimDeviceSim.h"

namespace SnobotSim
{

class CtreBuffTrajPointStreamWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    CtreBuffTrajPointStreamWrapper();
    const int mDeviceId;

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);

    //////////////////////////////////////////
    void Clear();
    void Write(double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos, uint32_t timeDur, bool useAuxPID);
    void Lookup(void** outObject);

protected:
    hal::SimDevice m_simDevice;

    hal::SimDouble m_Lookup_outObject;
    hal::SimDouble m_Write_arbFeedFwd;
    hal::SimDouble m_Write_auxiliaryArbFeedFwd;
    hal::SimDouble m_Write_auxiliaryPos;
    hal::SimDouble m_Write_auxiliaryVel;
    hal::SimDouble m_Write_isLastPoint;
    hal::SimDouble m_Write_position;
    hal::SimDouble m_Write_profileSlotSelect0;
    hal::SimDouble m_Write_profileSlotSelect1;
    hal::SimDouble m_Write_timeDur;
    hal::SimDouble m_Write_useAuxPID;
    hal::SimDouble m_Write_velocity;
    hal::SimDouble m_Write_zeroPos;
};

} // namespace SnobotSim
