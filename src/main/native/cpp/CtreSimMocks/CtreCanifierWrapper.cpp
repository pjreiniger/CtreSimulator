
#include "CtreSimMocks/CtreCanifierWrapper.h"

#include <vector>

#include "CtreSimMocks/MockHooks.h"

std::vector<SnobotSim::CTRE_CallbackFunc> gCanifierCallbacks;

void SnobotSim::SetCanifierCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gCanifierCallbacks.clear();
    gCanifierCallbacks.push_back(callback);
}

SnobotSim::CtreCanifierWrapper::CtreCanifierWrapper(int aDeviceId) :
        mDeviceId(aDeviceId)
{
    Send("Create");
}

void SnobotSim::CtreCanifierWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gCanifierCallbacks.empty())
    {
        gCanifierCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreCanifierWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    if (!gCanifierCallbacks.empty())
    {
        gCanifierCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}
