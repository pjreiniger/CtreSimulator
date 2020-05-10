

#include "CtreSimMocks/CtrePigeonIMUWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size) \
    uint8_t buffer[size]; /* NOLINT */  \
    std::memset(&buffer[0], 0, size);   \
    Receive(paramName, buffer, size);   \
    uint32_t buffer_pos = 0;

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

void SnobotSim::CtrePigeonIMUWrapper::GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    RECEIVE_HELPER("GetDescription", 1);
    buffer_pos += 1; // Removes compiler warning
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    Send("ConfigSetParameter", param, value, subValue, ordinal);
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigGetParameter(int param, double* value, int ordinal, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigSetCustomParam(int newValue, int paramIndex, int timeoutMs)
{
    Send("ConfigSetCustomParam", newValue, paramIndex);
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigGetCustomParam(int* readValue, int paramIndex, int timoutMs)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex) + sizeof(timoutMs));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);
    PoplateReceiveResults(buffer, &timoutMs, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigFactoryDefault(int timeoutMs)
{
    Send("ConfigFactoryDefault");
}

void SnobotSim::CtrePigeonIMUWrapper::SetYaw(double angleDeg, int timeoutMs)
{
    Send("SetYaw", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::AddYaw(double angleDeg, int timeoutMs)
{
    Send("AddYaw", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::SetYawToCompass(int timeoutMs)
{
    Send("SetYawToCompass");
}

void SnobotSim::CtrePigeonIMUWrapper::SetFusedHeading(double angleDeg, int timeoutMs)
{
    Send("SetFusedHeading", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::AddFusedHeading(double angleDeg, int timeoutMs)
{
    Send("AddFusedHeading", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::SetFusedHeadingToCompass(int timeoutMs)
{
    Send("SetFusedHeadingToCompass");
}

void SnobotSim::CtrePigeonIMUWrapper::SetAccumZAngle(double angleDeg, int timeoutMs)
{
    Send("SetAccumZAngle", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::SetTemperatureCompensationDisable(int bTempCompDisable, int timeoutMs)
{
    Send("SetTemperatureCompensationDisable", bTempCompDisable);
}

void SnobotSim::CtrePigeonIMUWrapper::SetCompassDeclination(double angleDegOffset, int timeoutMs)
{
    Send("SetCompassDeclination", angleDegOffset);
}

void SnobotSim::CtrePigeonIMUWrapper::SetCompassAngle(double angleDeg, int timeoutMs)
{
    Send("SetCompassAngle", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::EnterCalibrationMode(int calMode, int timeoutMs)
{
    Send("EnterCalibrationMode", calMode);
}

void SnobotSim::CtrePigeonIMUWrapper::GetGeneralStatus(int* state, int* currentMode, int* calibrationError, int* bCalIsBooting, double* tempC, int* upTimeSec, int* noMotionBiasCount, int* tempCompensationCount, int* lastError)
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
}

ctre::phoenix::ErrorCode SnobotSim::CtrePigeonIMUWrapper::GetLastError()
{
    int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)lastError;
}

void SnobotSim::CtrePigeonIMUWrapper::Get6dQuaternion(double wxyz[4])
{
    RECEIVE_HELPER("Get6dQuaternion", sizeof(wxyz[0]) + sizeof(wxyz[1]) + sizeof(wxyz[2]) + sizeof(wxyz[3]));
    PoplateReceiveResults(buffer, &wxyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[2], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[3], buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetYawPitchRoll(double ypr[3])
{
    RECEIVE_HELPER("GetYawPitchRoll", sizeof(ypr[0]) + sizeof(ypr[1]) + sizeof(ypr[2]));
    PoplateReceiveResults(buffer, &ypr[0], buffer_pos);
    PoplateReceiveResults(buffer, &ypr[1], buffer_pos);
    PoplateReceiveResults(buffer, &ypr[2], buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetAccumGyro(double xyz_deg[3])
{
    RECEIVE_HELPER("GetAccumGyro", sizeof(xyz_deg[0]) + sizeof(xyz_deg[1]) + sizeof(xyz_deg[2]));
    PoplateReceiveResults(buffer, &xyz_deg[0], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_deg[1], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_deg[2], buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetAbsoluteCompassHeading(double* value)
{
    RECEIVE_HELPER("GetAbsoluteCompassHeading", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetCompassHeading(double* value)
{
    RECEIVE_HELPER("GetCompassHeading", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetCompassFieldStrength(double* value)
{
    RECEIVE_HELPER("GetCompassFieldStrength", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetTemp(double* value)
{
    RECEIVE_HELPER("GetTemp", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetState(int* state)
{
    RECEIVE_HELPER("GetState", sizeof(*state));
    PoplateReceiveResults(buffer, state, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetUpTime(int* value)
{
    RECEIVE_HELPER("GetUpTime", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetRawMagnetometer(short rm_xyz[3]) // NOLINT(runtime/int)
{
    RECEIVE_HELPER("GetRawMagnetometer", sizeof(rm_xyz[0]) + sizeof(rm_xyz[1]) + sizeof(rm_xyz[2]));
    PoplateReceiveResults(buffer, &rm_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &rm_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &rm_xyz[2], buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetBiasedMagnetometer(short bm_xyz[3]) // NOLINT(runtime/int)
{
    RECEIVE_HELPER("GetBiasedMagnetometer", sizeof(bm_xyz[0]) + sizeof(bm_xyz[1]) + sizeof(bm_xyz[2]));
    PoplateReceiveResults(buffer, &bm_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &bm_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &bm_xyz[2], buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetBiasedAccelerometer(short ba_xyz[3]) // NOLINT(runtime/int)
{
    RECEIVE_HELPER("GetBiasedAccelerometer", sizeof(ba_xyz[0]) + sizeof(ba_xyz[1]) + sizeof(ba_xyz[2]));
    PoplateReceiveResults(buffer, &ba_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &ba_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &ba_xyz[2], buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetRawGyro(double xyz_dps[3])
{
    RECEIVE_HELPER("GetRawGyro", sizeof(xyz_dps[0]) + sizeof(xyz_dps[1]) + sizeof(xyz_dps[2]));
    PoplateReceiveResults(buffer, &xyz_dps[0], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_dps[1], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_dps[2], buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetAccelerometerAngles(double tiltAngles[3])
{
    RECEIVE_HELPER("GetAccelerometerAngles", sizeof(tiltAngles[0]) + sizeof(tiltAngles[1]) + sizeof(tiltAngles[2]));
    PoplateReceiveResults(buffer, &tiltAngles[0], buffer_pos);
    PoplateReceiveResults(buffer, &tiltAngles[1], buffer_pos);
    PoplateReceiveResults(buffer, &tiltAngles[2], buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetFusedHeading2(int* bIsFusing, int* bIsValid, double* value, int* lastError)
{
    RECEIVE_HELPER("GetFusedHeading2", sizeof(*bIsFusing) + sizeof(*bIsValid) + sizeof(*value) + sizeof(*lastError));
    PoplateReceiveResults(buffer, bIsFusing, buffer_pos);
    PoplateReceiveResults(buffer, bIsValid, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, lastError, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetFusedHeading1(double* value)
{
    RECEIVE_HELPER("GetFusedHeading1", sizeof(double) * 3);
    PoplateReceiveResults(buffer, &value[0], buffer_pos);
    PoplateReceiveResults(buffer, &value[1], buffer_pos);
    PoplateReceiveResults(buffer, &value[2], buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetResetCount(int* value)
{
    RECEIVE_HELPER("GetResetCount", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetResetFlags(int* value)
{
    RECEIVE_HELPER("GetResetFlags", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetFirmwareVersion(int* firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*firmwareVers));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::HasResetOccurred(bool* hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*hasReset));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::SetLastError(int value)
{
    Send("SetLastError", value);
}

void SnobotSim::CtrePigeonIMUWrapper::GetFaults(int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::GetStickyFaults(int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::ClearStickyFaults(int timeoutMs)
{
    Send("ClearStickyFaults");
}

void SnobotSim::CtrePigeonIMUWrapper::SetStatusFramePeriod(int frame, uint8_t periodMs, int timeoutMs)
{
    Send("SetStatusFramePeriod", frame, periodMs);
}

void SnobotSim::CtrePigeonIMUWrapper::GetStatusFramePeriod(int frame, int* periodMs, int timeoutMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
}

void SnobotSim::CtrePigeonIMUWrapper::SetControlFramePeriod(int frame, int periodMs)
{
    Send("SetControlFramePeriod", frame, periodMs);
}
