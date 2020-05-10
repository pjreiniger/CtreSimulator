
#include "ctre/phoenix/cci/BuffTrajPointStream_CCI.h"

#include <cstring>
#include <vector>

#include "CtreSimMocks/CtreBuffTrajPointStreamWrapper.h"
#include "CtreSimMocks/MockHooks.h"

namespace
{
SnobotSim::CtreBuffTrajPointStreamWrapper* ConvertToWrapper(void* param)
{
    return reinterpret_cast<SnobotSim::CtreBuffTrajPointStreamWrapper*>(param);
}
}

extern "C" {

void* c_BuffTrajPointStream_Create1()
{
    auto* output = new SnobotSim::CtreBuffTrajPointStreamWrapper();
    return output;
}

void c_BuffTrajPointStream_DestroyAll(void)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}

ctre::phoenix::ErrorCode c_BuffTrajPointStream_Destroy(void* handle)
{
    auto* wrapper = ConvertToWrapper(handle);
    delete wrapper;
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_BuffTrajPointStream_Clear(void* handle)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("Clear");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_BuffTrajPointStream_Write(void* handle, double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos, uint32_t timeDur, bool useAuxPID)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("Clear", position, velocity, arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd,
            profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos, timeDur, useAuxPID);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_BuffTrajPointStream_Lookup(void* handle, void** outObject)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

} // extern "C"
