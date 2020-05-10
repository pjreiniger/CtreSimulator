#include "ctre/phoenix/cci/PigeonIMU_CCI.h"

#include <cstring>
#include <vector>

#include "CtreSimMocks/CtrePigeonIMUWrapper.h"
#include "CtreSimUtils/MockHooks.h"

namespace
{
SnobotSim::CtrePigeonIMUWrapper* ConvertToWrapper(void* param)
{
    return reinterpret_cast<SnobotSim::CtrePigeonIMUWrapper*>(param);
}
} // namespace

extern "C" {

void* c_PigeonIMU_Create2(int talonDeviceID)
{
    auto* output = new SnobotSim::CtrePigeonIMUWrapper(talonDeviceID);
    return output;
}

void* c_PigeonIMU_Create1(int deviceNumber)
{
    auto* output = new SnobotSim::CtrePigeonIMUWrapper(deviceNumber);
    return output;
}

ctre::phoenix::ErrorCode c_PigeonIMU_Destroy(void* handle)
{
    auto* wrapper = ConvertToWrapper(handle);
    delete wrapper;
    return (ctre::phoenix::ErrorCode)0;
}

void c_PigeonIMU_DestroyAll(void)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetDescription(void* handle, char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    ConvertToWrapper(handle)->GetDescription(toFill, toFillByteSz, numBytesFilled);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetParameter(void* handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSetParameter(param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter(void* handle, int param, double* value, int ordinal, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetParameter(param, value, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter_6(void* handle, int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetParameter_6(param, valueToSend, valueRecieved, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetCustomParam(void* handle, int newValue, int paramIndex, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSetCustomParam(newValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetCustomParam(void* handle, int* readValue, int paramIndex, int timoutMs)
{
    ConvertToWrapper(handle)->ConfigGetCustomParam(readValue, paramIndex, timoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigFactoryDefault(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigFactoryDefault();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetYaw(void* handle, double angleDeg, int timeoutMs)
{
    ConvertToWrapper(handle)->SetYaw(angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_AddYaw(void* handle, double angleDeg, int timeoutMs)
{
    ConvertToWrapper(handle)->AddYaw(angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetYawToCompass(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->SetYawToCompass();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeading(void* handle, double angleDeg, int timeoutMs)
{
    ConvertToWrapper(handle)->SetFusedHeading(angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_AddFusedHeading(void* handle, double angleDeg, int timeoutMs)
{
    ConvertToWrapper(handle)->AddFusedHeading(angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeadingToCompass(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->SetFusedHeadingToCompass();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetAccumZAngle(void* handle, double angleDeg, int timeoutMs)
{
    ConvertToWrapper(handle)->SetAccumZAngle(angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetTemperatureCompensationDisable(void* handle, int bTempCompDisable, int timeoutMs)
{
    ConvertToWrapper(handle)->SetTemperatureCompensationDisable(bTempCompDisable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassDeclination(void* handle, double angleDegOffset, int timeoutMs)
{
    ConvertToWrapper(handle)->SetCompassDeclination(angleDegOffset);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassAngle(void* handle, double angleDeg, int timeoutMs)
{
    ConvertToWrapper(handle)->SetCompassAngle(angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_EnterCalibrationMode(void* handle, int calMode, int timeoutMs)
{
    ConvertToWrapper(handle)->EnterCalibrationMode(calMode);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetGeneralStatus(void* handle, int* state, int* currentMode, int* calibrationError, int* bCalIsBooting, double* tempC, int* upTimeSec, int* noMotionBiasCount, int* tempCompensationCount, int* lastError)
{
    ConvertToWrapper(handle)->GetGeneralStatus(state, currentMode, calibrationError, bCalIsBooting, tempC, upTimeSec, noMotionBiasCount, tempCompensationCount, lastError);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetLastError(void* handle)
{
    return ConvertToWrapper(handle)->GetLastError();
}

ctre::phoenix::ErrorCode c_PigeonIMU_Get6dQuaternion(void* handle, double wxyz[4])
{
    ConvertToWrapper(handle)->Get6dQuaternion(wxyz);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetYawPitchRoll(void* handle, double ypr[3])
{
    ConvertToWrapper(handle)->GetYawPitchRoll(ypr);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetAccumGyro(void* handle, double xyz_deg[3])
{
    ConvertToWrapper(handle)->GetAccumGyro(xyz_deg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetAbsoluteCompassHeading(void* handle, double* value)
{
    ConvertToWrapper(handle)->GetAbsoluteCompassHeading(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassHeading(void* handle, double* value)
{
    ConvertToWrapper(handle)->GetCompassHeading(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassFieldStrength(void* handle, double* value)
{
    ConvertToWrapper(handle)->GetCompassFieldStrength(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetTemp(void* handle, double* value)
{
    ConvertToWrapper(handle)->GetTemp(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetState(void* handle, int* state)
{
    ConvertToWrapper(handle)->GetState(state);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetUpTime(void* handle, int* value)
{
    ConvertToWrapper(handle)->GetUpTime(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetRawMagnetometer(void* handle, short rm_xyz[3]) // NOLINT(runtime/int)
{
    ConvertToWrapper(handle)->GetRawMagnetometer(rm_xyz);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedMagnetometer(void* handle, short bm_xyz[3]) // NOLINT(runtime/int)
{
    ConvertToWrapper(handle)->GetBiasedMagnetometer(bm_xyz);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedAccelerometer(void* handle, short ba_xyz[3]) // NOLINT(runtime/int)
{
    ConvertToWrapper(handle)->GetBiasedAccelerometer(ba_xyz);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetRawGyro(void* handle, double xyz_dps[3])
{
    ConvertToWrapper(handle)->GetRawGyro(xyz_dps);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetAccelerometerAngles(void* handle, double tiltAngles[3])
{
    ConvertToWrapper(handle)->GetAccelerometerAngles(tiltAngles);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading2(void* handle, int* bIsFusing, int* bIsValid, double* value, int* lastError)
{
    ConvertToWrapper(handle)->GetFusedHeading2(bIsFusing, bIsValid, value, lastError);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading1(void* handle, double* value)
{
    ConvertToWrapper(handle)->GetFusedHeading1(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetResetCount(void* handle, int* value)
{
    ConvertToWrapper(handle)->GetResetCount(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetResetFlags(void* handle, int* value)
{
    ConvertToWrapper(handle)->GetResetFlags(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFirmwareVersion(void* handle, int* firmwareVers)
{
    ConvertToWrapper(handle)->GetFirmwareVersion(firmwareVers);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_HasResetOccurred(void* handle, bool* hasReset)
{
    ConvertToWrapper(handle)->HasResetOccurred(hasReset);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetLastError(void* handle, int value)
{
    ConvertToWrapper(handle)->SetLastError(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFaults(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetFaults(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetStickyFaults(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetStickyFaults(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ClearStickyFaults(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->ClearStickyFaults();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetStatusFramePeriod(void* handle, int frame, uint8_t periodMs, int timeoutMs)
{
    ConvertToWrapper(handle)->SetStatusFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetStatusFramePeriod(void* handle, int frame, int* periodMs, int timeoutMs)
{
    ConvertToWrapper(handle)->GetStatusFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetControlFramePeriod(void* handle, int frame, int periodMs)
{
    ConvertToWrapper(handle)->SetControlFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

} // extern "C"
