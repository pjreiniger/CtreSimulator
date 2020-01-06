
#include "ctre/phoenix/cci/PigeonIMU_CCI.h"

#include <cstring>
#include <vector>

#include "CtreSimMocks/CtrePigeonImuWrapper.h"
#include "CtreSimMocks/MockHooks.h"

typedef SnobotSim::CtrePigeonImuWrapper PigeonImuSimulatorWrapper;

#define RECEIVE_HELPER(paramName, size)                                  \
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle); \
    uint8_t buffer[size]; /* NOLINT */                                   \
    std::memset(&buffer[0], 0, size);                                    \
    wrapper->Receive(paramName, buffer, size);                           \
    uint32_t buffer_pos = 0;


PigeonImuSimulatorWrapper* ConvertToPigeonWrapper(void* param)
{
    return reinterpret_cast<PigeonImuSimulatorWrapper*>(param);
}

extern "C"{

void *c_PigeonIMU_Create2(int talonDeviceID)
{
	SnobotSim::CtrePigeonImuWrapper* output = new SnobotSim::CtrePigeonImuWrapper(talonDeviceID);
    return output;
}

void *c_PigeonIMU_Create1(int deviceNumber)
{
	SnobotSim::CtrePigeonImuWrapper* output = new SnobotSim::CtrePigeonImuWrapper(deviceNumber);
    return output;
}

ctre::phoenix::ErrorCode c_PigeonIMU_Destroy(void *handle)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    delete wrapper;
    return (ctre::phoenix::ErrorCode)0;
}

void c_PigeonIMU_DestroyAll()
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetDescription(void *handle, char * toFill, int toFillByteSz, size_t * numBytesFilled)
{
    RECEIVE_HELPER("GetDescription", 1);
    buffer_pos += 1; // Removes compiler warning
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetParameter(void *handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("ConfigSetParameter", param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter(void *handle, int param, double *value, int ordinal, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter_6(void *handle, int32_t param, int32_t valueToSend, int32_t * valueRecieved,uint8_t * subValue, int32_t ordinal, int32_t timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetCustomParam(void *handle, int newValue, int paramIndex, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("ConfigSetCustomParam", newValue, paramIndex, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetCustomParam(void *handle, int *readValue, int paramIndex, int timoutMs)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ConfigFactoryDefault(void *handle, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("ConfigFactoryDefault");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetYaw(void *handle, double angleDeg, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetYaw", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_AddYaw(void *handle, double angleDeg, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("AddYaw", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetYawToCompass(void *handle, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetYawToCompass");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeading(void *handle, double angleDeg, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetFusedHeading", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_AddFusedHeading(void *handle, double angleDeg, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("AddFusedHeading", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeadingToCompass(void *handle, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetFusedHeadingToCompass");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetAccumZAngle(void *handle, double angleDeg, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetAccumZAngle", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetTemperatureCompensationDisable(void *handle, int bTempCompDisable, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetTemperatureCompensationDisable", bTempCompDisable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassDeclination(void *handle, double angleDegOffset, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetCompassDeclination", angleDegOffset);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassAngle(void *handle, double angleDeg, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetCompassAngle", angleDeg);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_EnterCalibrationMode(void *handle, int calMode, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("EnterCalibrationMode", calMode);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetGeneralStatus(void *handle, int *state, int *currentMode, int *calibrationError, int *bCalIsBooting, double *tempC, int *upTimeSec, int *noMotionBiasCount, int *tempCompensationCount, int *lastError)
{
    RECEIVE_HELPER("GetGeneralStatus", sizeof(int) * 8 + sizeof(double));
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

ctre::phoenix::ErrorCode c_PigeonIMU_GetLastError(void *handle)
{
	int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode) lastError;
}

ctre::phoenix::ErrorCode c_PigeonIMU_Get6dQuaternion(void *handle, double wxyz[4])
{
    RECEIVE_HELPER("Get6dQuaternion", sizeof(double) * 4);
    PoplateReceiveResults(buffer, &wxyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[2], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[3], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetYawPitchRoll(void *handle, double ypr[3])
{
    RECEIVE_HELPER("GetYawPitchRoll", sizeof(double) * 3);
    PoplateReceiveResults(buffer, &ypr[0], buffer_pos);
    PoplateReceiveResults(buffer, &ypr[1], buffer_pos);
    PoplateReceiveResults(buffer, &ypr[2], buffer_pos);
    return ctre::phoenix::NotImplemented;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetAccumGyro(void *handle, double xyz_deg[3])
{
    RECEIVE_HELPER("GetAccumGyro", sizeof(double) * 3);
    PoplateReceiveResults(buffer, &xyz_deg[0], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_deg[1], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_deg[2], buffer_pos);
    return ctre::phoenix::NotImplemented;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetAbsoluteCompassHeading(void *handle, double *value)
{
    RECEIVE_HELPER("GetAbsoluteCompassHeading", sizeof(double));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassHeading(void *handle, double *value)
{
    RECEIVE_HELPER("GetCompassHeading", sizeof(double));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassFieldStrength(void *handle, double *value)
{
    RECEIVE_HELPER("GetCompassFieldStrength", sizeof(double));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetTemp(void *handle, double *value)
{
    RECEIVE_HELPER("GetTemp", sizeof(double));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetState(void *handle, int *state)
{
    RECEIVE_HELPER("GetState", sizeof(int));
    PoplateReceiveResults(buffer, state, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetUpTime(void *handle, int *value)
{
    RECEIVE_HELPER("GetUpTime", sizeof(int));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetRawMagnetometer(void *handle, short rm_xyz[3])
{
    RECEIVE_HELPER("GetRawMagnetometer", sizeof(short) * 3); // NOLINT
    PoplateReceiveResults(buffer, &rm_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &rm_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &rm_xyz[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedMagnetometer(void *handle, short bm_xyz[3])
{
    RECEIVE_HELPER("GetBiasedMagnetometer", sizeof(short) * 3); // NOLINT
    PoplateReceiveResults(buffer, &bm_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &bm_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &bm_xyz[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedAccelerometer(void *handle, short ba_xyz[3])
{
    RECEIVE_HELPER("GetBiasedAccelerometer", sizeof(short) * 3); // NOLINT
    PoplateReceiveResults(buffer, &ba_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &ba_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &ba_xyz[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetRawGyro(void *handle, double xyz_dps[3])
{
    RECEIVE_HELPER("GetRawGyro", sizeof(double) * 3);
    PoplateReceiveResults(buffer, &xyz_dps[0], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_dps[1], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_dps[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetAccelerometerAngles(void *handle, double tiltAngles[3])
{
    RECEIVE_HELPER("GetAccelerometerAngles", sizeof(double) * 3);
    PoplateReceiveResults(buffer, &tiltAngles[0], buffer_pos);
    PoplateReceiveResults(buffer, &tiltAngles[1], buffer_pos);
    PoplateReceiveResults(buffer, &tiltAngles[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading2(void *handle, int *bIsFusing, int *bIsValid, double *value, int *lastError)
{
    RECEIVE_HELPER("GetFusedHeading2", sizeof(double));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *bIsFusing = 5;
    *bIsValid = 5;
    *lastError = ctre::phoenix::NotImplemented;

    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading1(void *handle, double *value)
{
    RECEIVE_HELPER("GetFusedHeading1", sizeof(double) * 3);
    PoplateReceiveResults(buffer, &value[0], buffer_pos);
    PoplateReceiveResults(buffer, &value[1], buffer_pos);
    PoplateReceiveResults(buffer, &value[2], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetResetCount(void *handle, int *value)
{
    RECEIVE_HELPER("GetResetCount", sizeof(int));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetResetFlags(void *handle, int *value)
{
    RECEIVE_HELPER("GetResetFlags", sizeof(int));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFirmwareVersion(void *handle, int * firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(int));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_HasResetOccurred(void *handle, bool * hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(bool));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetLastError(void *handle, int value)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetLastError", value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetFaults(void *handle, int * param)
{
    RECEIVE_HELPER("GetFaults", sizeof(int));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetStickyFaults(void *handle, int * param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(int));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_ClearStickyFaults(void *handle, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("ClearStickyFaults", timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetStatusFramePeriod(void *handle, int frame, uint8_t periodMs, int timeoutMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetStatusFramePeriod", frame, periodMs, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_GetStatusFramePeriod(void *handle, int frame, int *periodMs, int timeoutMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(int));
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_PigeonIMU_SetControlFramePeriod(void *handle, int frame, int periodMs)
{
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle);
    wrapper->Send("SetControlFramePeriod", frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

}  // extern "C"
