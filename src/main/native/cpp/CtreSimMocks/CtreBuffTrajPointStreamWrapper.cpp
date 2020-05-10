

#include "CtreSimMocks/CtreBuffTrajPointStreamWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size) \
    uint8_t buffer[size]; /* NOLINT */  \
    std::memset(&buffer[0], 0, size);   \
    Receive(paramName, buffer, size);   \
    uint32_t buffer_pos = 0;

std::vector<SnobotSim::CTRE_CallbackFunc> gBuffTrajPointStreamCallbacks;

void SnobotSim::SetBuffTrajPointStreamCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gBuffTrajPointStreamCallbacks.clear();
    gBuffTrajPointStreamCallbacks.push_back(callback);
}

SnobotSim::CtreBuffTrajPointStreamWrapper::CtreBuffTrajPointStreamWrapper()
{
    Send("Create");
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gBuffTrajPointStreamCallbacks.empty())
    {
        gBuffTrajPointStreamCallbacks[0](aName.c_str(), 0, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    if (!gBuffTrajPointStreamCallbacks.empty())
    {
        gBuffTrajPointStreamCallbacks[0](aName.c_str(), 0, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Clear()
{
    Send("Clear");
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Write(double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos, uint32_t timeDur, bool useAuxPID)
{
    Send("Write", position, velocity, arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos, timeDur, useAuxPID);
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Lookup(void** outObject)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    // RECEIVE_HELPER("Lookup", sizeof(*outObject));
    // PoplateReceiveResults(buffer, outObject, buffer_pos);
}
