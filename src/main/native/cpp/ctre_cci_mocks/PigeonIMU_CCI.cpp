
#include "ctre/phoenix/cci/PigeonIMU_CCI.h"

#include <cstring>
#include <vector>

#include "CtreSimMocks/CtrePigeonIMUWrapper.h"
#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size)        \
    auto* wrapper = ConvertToWrapper(handle);  \
    uint8_t buffer[size]; /* NOLINT */         \
    std::memset(&buffer[0], 0, size);          \
    wrapper->Receive(paramName, buffer, size); \
    uint32_t buffer_pos = 0;

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
    RECEIVE_HELPER("GetDescription", 1);
    buffer_pos += 1; // Removes compiler warning
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetParameter(void* handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSetParameter", param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter(void* handle, int param, double* value, int ordinal, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal) + sizeof(timeoutMs));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter_6(void* handle, int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal) + sizeof(timeoutMs));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetCustomParam(void* handle, int newValue, int paramIndex, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSetCustomParam", newValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetCustomParam(void* handle, int* readValue, int paramIndex, int timoutMs)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex) + sizeof(timoutMs));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);
    PoplateReceiveResults(buffer, &timoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigFactoryDefault(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigFactoryDefault");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetYaw(void* handle, double angleDeg, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetYaw", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_AddYaw(void* handle, double angleDeg, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("AddYaw", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetYawToCompass(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetYawToCompass");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeading(void* handle, double angleDeg, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetFusedHeading", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_AddFusedHeading(void* handle, double angleDeg, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("AddFusedHeading", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeadingToCompass(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetFusedHeadingToCompass");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetAccumZAngle(void* handle, double angleDeg, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetAccumZAngle", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetTemperatureCompensationDisable(void* handle, int bTempCompDisable, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetTemperatureCompensationDisable", bTempCompDisable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassDeclination(void* handle, double angleDegOffset, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetCompassDeclination", angleDegOffset);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassAngle(void* handle, double angleDeg, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetCompassAngle", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_EnterCalibrationMode(void* handle, int calMode, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("EnterCalibrationMode", calMode);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetGeneralStatus(void* handle, int* state, int* currentMode, int* calibrationError, int* bCalIsBooting, double* tempC, int* upTimeSec, int* noMotionBiasCount, int* tempCompensationCount, int* lastError)
{
    RECEIVE_HELPER("GetGeneralStatus", sizeof(*state) + sizeof(*currentMode) + sizeof(*calibrationError) + sizeof(*bCalIsBooting) + sizeof(*tempC) + sizeof(*upTimeSec) + sizeof(*noMotionBiasCount) + sizeof(*tempCompensationCount) + sizeof(*lastError));
    PoplateReceiveResults(buffer, state, buffer_pos);
    PoplateReceiveResults(buffer, currentMode, buffer_pos);
    PoplateReceiveResults(buffer, calibrationError, buffer_pos);
    PoplateReceiveResults(buffer, bCalIsBooting, buffer_pos);
    PoplateReceiveResults(buffer, tempC, buffer_pos);
    PoplateReceiveResults(buffer, upTimeSec, buffer_pos);
    PoplateReceiveResults(buffer, noMotionBiasCount, buffer_pos);
    PoplateReceiveResults(buffer, tempCompensationCount, buffer_pos);
    PoplateReceiveResults(buffer, lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetLastError(void* handle)
{
    int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)lastError;
}

ctre::phoenix::ErrorCode c_PigeonIMU_Get6dQuaternion(void* handle, double wxyz[4])
{
    RECEIVE_HELPER("Get6dQuaternion", sizeof(wxyz[0]) + sizeof(wxyz[1]) + sizeof(wxyz[2]) + sizeof(wxyz[3]));
    PoplateReceiveResults(buffer, &wxyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[2], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[3], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetYawPitchRoll(void* handle, double ypr[3])
{
    RECEIVE_HELPER("GetYawPitchRoll", sizeof(ypr[0]) + sizeof(ypr[1]) + sizeof(ypr[2]));
    PoplateReceiveResults(buffer, &ypr[0], buffer_pos);
    PoplateReceiveResults(buffer, &ypr[1], buffer_pos);
    PoplateReceiveResults(buffer, &ypr[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetAccumGyro(void* handle, double xyz_deg[3])
{
    RECEIVE_HELPER("GetAccumGyro", sizeof(xyz_deg[0]) + sizeof(xyz_deg[1]) + sizeof(xyz_deg[2]));
    PoplateReceiveResults(buffer, &xyz_deg[0], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_deg[1], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_deg[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetAbsoluteCompassHeading(void* handle, double* value)
{
    RECEIVE_HELPER("GetAbsoluteCompassHeading", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassHeading(void* handle, double* value)
{
    RECEIVE_HELPER("GetCompassHeading", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassFieldStrength(void* handle, double* value)
{
    RECEIVE_HELPER("GetCompassFieldStrength", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetTemp(void* handle, double* value)
{
    RECEIVE_HELPER("GetTemp", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetState(void* handle, int* state)
{
    RECEIVE_HELPER("GetState", sizeof(*state));
    PoplateReceiveResults(buffer, state, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetUpTime(void* handle, int* value)
{
    RECEIVE_HELPER("GetUpTime", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetRawMagnetometer(void* handle, short rm_xyz[3]) // NOLINT(runtime/int)
{
    RECEIVE_HELPER("GetRawMagnetometer", sizeof(rm_xyz[0]) + sizeof(rm_xyz[1]) + sizeof(rm_xyz[2]));
    PoplateReceiveResults(buffer, &rm_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &rm_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &rm_xyz[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedMagnetometer(void* handle, short bm_xyz[3]) // NOLINT(runtime/int)
{
    RECEIVE_HELPER("GetBiasedMagnetometer", sizeof(bm_xyz[0]) + sizeof(bm_xyz[1]) + sizeof(bm_xyz[2]));
    PoplateReceiveResults(buffer, &bm_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &bm_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &bm_xyz[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedAccelerometer(void* handle, short ba_xyz[3]) // NOLINT(runtime/int)
{
    RECEIVE_HELPER("GetBiasedAccelerometer", sizeof(ba_xyz[0]) + sizeof(ba_xyz[1]) + sizeof(ba_xyz[2]));
    PoplateReceiveResults(buffer, &ba_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &ba_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &ba_xyz[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetRawGyro(void* handle, double xyz_dps[3])
{
    RECEIVE_HELPER("GetRawGyro", sizeof(xyz_dps[0]) + sizeof(xyz_dps[1]) + sizeof(xyz_dps[2]));
    PoplateReceiveResults(buffer, &xyz_dps[0], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_dps[1], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_dps[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetAccelerometerAngles(void* handle, double tiltAngles[3])
{
    RECEIVE_HELPER("GetAccelerometerAngles", sizeof(tiltAngles[0]) + sizeof(tiltAngles[1]) + sizeof(tiltAngles[2]));
    PoplateReceiveResults(buffer, &tiltAngles[0], buffer_pos);
    PoplateReceiveResults(buffer, &tiltAngles[1], buffer_pos);
    PoplateReceiveResults(buffer, &tiltAngles[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading2(void* handle, int* bIsFusing, int* bIsValid, double* value, int* lastError)
{
    RECEIVE_HELPER("GetFusedHeading2", sizeof(*bIsFusing) + sizeof(*bIsValid) + sizeof(*value) + sizeof(*lastError));
    PoplateReceiveResults(buffer, bIsFusing, buffer_pos);
    PoplateReceiveResults(buffer, bIsValid, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading1(void* handle, double* value)
{
    RECEIVE_HELPER("GetFusedHeading1", sizeof(double) * 3);
    PoplateReceiveResults(buffer, &value[0], buffer_pos);
    PoplateReceiveResults(buffer, &value[1], buffer_pos);
    PoplateReceiveResults(buffer, &value[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetResetCount(void* handle, int* value)
{
    RECEIVE_HELPER("GetResetCount", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetResetFlags(void* handle, int* value)
{
    RECEIVE_HELPER("GetResetFlags", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFirmwareVersion(void* handle, int* firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*firmwareVers));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_HasResetOccurred(void* handle, bool* hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*hasReset));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetLastError(void* handle, int value)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetLastError", value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFaults(void* handle, int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetStickyFaults(void* handle, int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ClearStickyFaults(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ClearStickyFaults");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetStatusFramePeriod(void* handle, int frame, uint8_t periodMs, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetStatusFramePeriod", frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetStatusFramePeriod(void* handle, int frame, int* periodMs, int timeoutMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(int));
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetControlFramePeriod(void* handle, int frame, int periodMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetControlFramePeriod", frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

} // extern "C"
