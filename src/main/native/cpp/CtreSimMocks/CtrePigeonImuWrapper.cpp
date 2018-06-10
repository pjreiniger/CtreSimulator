

#include "CtreSimMocks/CtrePigeonImuWrapper.h"

#include <vector>

#include "CtreSimMocks/MockHooks.h"

std::vector<SnobotSim::CTRE_CallbackFunc> gPigeonCallbacks;

void SnobotSim::SetPigeonCallback(SnobotSim::CTRE_CallbackFunc callback)
{
    gPigeonCallbacks.clear();
    gPigeonCallbacks.push_back(callback);
}

SnobotSim::CtrePigeonImuWrapper::CtrePigeonImuWrapper(int aDeviceId) :
        mDeviceId(aDeviceId & 0x3F)
{
    Send("Create");
}

void SnobotSim::CtrePigeonImuWrapper::Send(const std::string& aName,
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

void SnobotSim::CtrePigeonImuWrapper::Receive(const std::string& aName,
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
