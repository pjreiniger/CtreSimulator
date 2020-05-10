

#include "CtreSimMocks/CtrePigeonIMUWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

std::vector<SnobotSim::CTRE_CallbackFunc> gPigeonCallbacks;

void SnobotSim::SetPigeonCallback(SnobotSim::CTRE_CallbackFunc callback)
{
    gPigeonCallbacks.clear();
    gPigeonCallbacks.push_back(callback);
}

SnobotSim::CtrePigeonIMUWrapper::CtrePigeonIMUWrapper(int aDeviceId) :
        mDeviceId(aDeviceId & 0x3F)
{
    Send("Create");
}

void SnobotSim::CtrePigeonIMUWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gPigeonCallbacks.empty())
    {
        gPigeonCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtrePigeonIMUWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gPigeonCallbacks.empty())
    {
        gPigeonCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}
