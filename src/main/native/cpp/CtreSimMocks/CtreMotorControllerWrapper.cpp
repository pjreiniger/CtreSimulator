
#include "CtreSimMocks/CtreMotorControllerWrapper.h"

#include <vector>

#include "CtreSimMocks/MockHooks.h"

std::vector<SnobotSim::CTRE_CallbackFunc> gMotorControllerCallbacks;

void SnobotSim::SetMotControllerCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gMotorControllerCallbacks.clear();
    gMotorControllerCallbacks.push_back(callback);
}

SnobotSim::CtreMotorControllerWrapper::CtreMotorControllerWrapper(int aDeviceId) :
        mDeviceId(aDeviceId & 0x3F)
{
    Send("Create");
}

void SnobotSim::CtreMotorControllerWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gMotorControllerCallbacks.empty())
    {
        gMotorControllerCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreMotorControllerWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    if (!gMotorControllerCallbacks.empty())
    {
        gMotorControllerCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}
