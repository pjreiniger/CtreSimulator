

#include "CtreSimMocks/CtrePigeonIMUWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size) \
    uint8_t buffer[size]; /* NOLINT */  \
    std::memset(&buffer[0], 0, size);   \
    Receive(paramName, buffer, size);   \
    uint32_t buffer_pos = 0;

std::vector<SnobotSim::CTRE_CallbackFunc> gPigeonIMUCallbacks;

void SnobotSim::SetPigeonIMUCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gPigeonIMUCallbacks.clear();
    gPigeonIMUCallbacks.push_back(callback);
}

SnobotSim::CtrePigeonIMUWrapper::CtrePigeonIMUWrapper(int aDeviceId) :
        mDeviceId(aDeviceId & 0x3F),
                m_simDevice(std::string("CtrePigeonIMUWrapper " + std::to_string(aDeviceId)).c_str(), aDeviceId)
{

    m_6dQuaternion_wxyz = m_simDevice.CreateDouble("6dQuaternion_wxyz", false, 0);
    m_AbsoluteCompassHeading_value = m_simDevice.CreateDouble("AbsoluteCompassHeading_value", false, 0);
    m_AccelerometerAngles_tiltAngles = m_simDevice.CreateDouble("AccelerometerAngles_tiltAngles", false, 0);
    m_AccumGyro_xyz_deg = m_simDevice.CreateDouble("AccumGyro_xyz_deg", false, 0);
    m_AccumZAngle_angleDeg = m_simDevice.CreateDouble("AccumZAngle_angleDeg", false, 0);
    m_AddFusedHeading_angleDeg = m_simDevice.CreateDouble("AddFusedHeading_angleDeg", false, 0);
    m_AddYaw_angleDeg = m_simDevice.CreateDouble("AddYaw_angleDeg", false, 0);
    m_BiasedAccelerometer_ba_xyz = m_simDevice.CreateDouble("BiasedAccelerometer_ba_xyz", false, 0);
    m_BiasedMagnetometer_bm_xyz = m_simDevice.CreateDouble("BiasedMagnetometer_bm_xyz", false, 0);
    m_CompassAngle_angleDeg = m_simDevice.CreateDouble("CompassAngle_angleDeg", false, 0);
    m_CompassDeclination_angleDegOffset = m_simDevice.CreateDouble("CompassDeclination_angleDegOffset", false, 0);
    m_CompassFieldStrength_value = m_simDevice.CreateDouble("CompassFieldStrength_value", false, 0);
    m_CompassHeading_value = m_simDevice.CreateDouble("CompassHeading_value", false, 0);
    m_ConfigGetCustomParam_paramIndex = m_simDevice.CreateDouble("ConfigGetCustomParam_paramIndex", false, 0);
    m_ConfigGetCustomParam_readValue = m_simDevice.CreateDouble("ConfigGetCustomParam_readValue", false, 0);
    m_ConfigGetParameter_6_ordinal = m_simDevice.CreateDouble("ConfigGetParameter_6_ordinal", false, 0);
    m_ConfigGetParameter_6_param = m_simDevice.CreateDouble("ConfigGetParameter_6_param", false, 0);
    m_ConfigGetParameter_6_subValue = m_simDevice.CreateDouble("ConfigGetParameter_6_subValue", false, 0);
    m_ConfigGetParameter_6_valueRecieved = m_simDevice.CreateDouble("ConfigGetParameter_6_valueRecieved", false, 0);
    m_ConfigGetParameter_6_valueToSend = m_simDevice.CreateDouble("ConfigGetParameter_6_valueToSend", false, 0);
    m_ConfigGetParameter_ordinal = m_simDevice.CreateDouble("ConfigGetParameter_ordinal", false, 0);
    m_ConfigGetParameter_param = m_simDevice.CreateDouble("ConfigGetParameter_param", false, 0);
    m_ConfigGetParameter_value = m_simDevice.CreateDouble("ConfigGetParameter_value", false, 0);
    m_ConfigSetCustomParam_newValue = m_simDevice.CreateDouble("ConfigSetCustomParam_newValue", false, 0);
    m_ConfigSetCustomParam_paramIndex = m_simDevice.CreateDouble("ConfigSetCustomParam_paramIndex", false, 0);
    m_ConfigSetParameter_ordinal = m_simDevice.CreateDouble("ConfigSetParameter_ordinal", false, 0);
    m_ConfigSetParameter_param = m_simDevice.CreateDouble("ConfigSetParameter_param", false, 0);
    m_ConfigSetParameter_subValue = m_simDevice.CreateDouble("ConfigSetParameter_subValue", false, 0);
    m_ConfigSetParameter_value = m_simDevice.CreateDouble("ConfigSetParameter_value", false, 0);
    m_ControlFramePeriod_frame = m_simDevice.CreateDouble("ControlFramePeriod_frame", false, 0);
    m_ControlFramePeriod_periodMs = m_simDevice.CreateDouble("ControlFramePeriod_periodMs", false, 0);
    m_Create1_deviceNumber = m_simDevice.CreateDouble("Create1_deviceNumber", false, 0);
    m_Create2_talonDeviceID = m_simDevice.CreateDouble("Create2_talonDeviceID", false, 0);
    m_Description_numBytesFilled = m_simDevice.CreateDouble("Description_numBytesFilled", false, 0);
    m_Description_toFill = m_simDevice.CreateDouble("Description_toFill", false, 0);
    m_Description_toFillByteSz = m_simDevice.CreateDouble("Description_toFillByteSz", false, 0);
    m_EnterCalibrationMode_calMode = m_simDevice.CreateDouble("EnterCalibrationMode_calMode", false, 0);
    m_Faults_param = m_simDevice.CreateDouble("Faults_param", false, 0);
    m_FirmwareVersion_firmwareVers = m_simDevice.CreateDouble("FirmwareVersion_firmwareVers", false, 0);
    m_FusedHeading1_value = m_simDevice.CreateDouble("FusedHeading1_value", false, 0);
    m_FusedHeading2_bIsFusing = m_simDevice.CreateDouble("FusedHeading2_bIsFusing", false, 0);
    m_FusedHeading2_bIsValid = m_simDevice.CreateDouble("FusedHeading2_bIsValid", false, 0);
    m_FusedHeading2_lastError = m_simDevice.CreateDouble("FusedHeading2_lastError", false, 0);
    m_FusedHeading2_value = m_simDevice.CreateDouble("FusedHeading2_value", false, 0);
    m_FusedHeading_angleDeg = m_simDevice.CreateDouble("FusedHeading_angleDeg", false, 0);
    m_GeneralStatus_bCalIsBooting = m_simDevice.CreateDouble("GeneralStatus_bCalIsBooting", false, 0);
    m_GeneralStatus_calibrationError = m_simDevice.CreateDouble("GeneralStatus_calibrationError", false, 0);
    m_GeneralStatus_currentMode = m_simDevice.CreateDouble("GeneralStatus_currentMode", false, 0);
    m_GeneralStatus_lastError = m_simDevice.CreateDouble("GeneralStatus_lastError", false, 0);
    m_GeneralStatus_noMotionBiasCount = m_simDevice.CreateDouble("GeneralStatus_noMotionBiasCount", false, 0);
    m_GeneralStatus_state = m_simDevice.CreateDouble("GeneralStatus_state", false, 0);
    m_GeneralStatus_tempC = m_simDevice.CreateDouble("GeneralStatus_tempC", false, 0);
    m_GeneralStatus_tempCompensationCount = m_simDevice.CreateDouble("GeneralStatus_tempCompensationCount", false, 0);
    m_GeneralStatus_upTimeSec = m_simDevice.CreateDouble("GeneralStatus_upTimeSec", false, 0);
    m_HasResetOccurred_hasReset = m_simDevice.CreateDouble("HasResetOccurred_hasReset", false, 0);
    m_LastError_value = m_simDevice.CreateDouble("LastError_value", false, 0);
    m_RawGyro_xyz_dps = m_simDevice.CreateDouble("RawGyro_xyz_dps", false, 0);
    m_RawMagnetometer_rm_xyz = m_simDevice.CreateDouble("RawMagnetometer_rm_xyz", false, 0);
    m_ResetCount_value = m_simDevice.CreateDouble("ResetCount_value", false, 0);
    m_ResetFlags_value = m_simDevice.CreateDouble("ResetFlags_value", false, 0);
    m_State_state = m_simDevice.CreateDouble("State_state", false, 0);
    m_StatusFramePeriod_frame = m_simDevice.CreateDouble("StatusFramePeriod_frame", false, 0);
    m_StatusFramePeriod_periodMs = m_simDevice.CreateDouble("StatusFramePeriod_periodMs", false, 0);
    m_StickyFaults_param = m_simDevice.CreateDouble("StickyFaults_param", false, 0);
    m_Temp_value = m_simDevice.CreateDouble("Temp_value", false, 0);
    m_TemperatureCompensationDisable_bTempCompDisable = m_simDevice.CreateDouble("TemperatureCompensationDisable_bTempCompDisable", false, 0);
    m_UpTime_value = m_simDevice.CreateDouble("UpTime_value", false, 0);
    m_YawPitchRoll_ypr = m_simDevice.CreateDouble("YawPitchRoll_ypr", false, 0);
    m_Yaw_angleDeg = m_simDevice.CreateDouble("Yaw_angleDeg", false, 0);

    Send("Create");
}

void SnobotSim::CtrePigeonIMUWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gPigeonIMUCallbacks.empty())
    {
        gPigeonIMUCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtrePigeonIMUWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    if (!gPigeonIMUCallbacks.empty())
    {
        gPigeonIMUCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtrePigeonIMUWrapper::GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    RECEIVE_HELPER("GetDescription", sizeof(*toFill) + sizeof(toFillByteSz) + sizeof(*numBytesFilled));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, &toFillByteSz, buffer_pos);
    PoplateReceiveResults(buffer, numBytesFilled, buffer_pos);

//    *toFillByteSz = m_Description_toFillByteSz.Get();
    *toFill = m_Description_toFill.Get();
    *numBytesFilled = m_Description_numBytesFilled.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal)
{
    m_ConfigSetParameter_value.Set(value);
    m_ConfigSetParameter_subValue.Set(subValue);
    m_ConfigSetParameter_param.Set(param);
    m_ConfigSetParameter_ordinal.Set(ordinal);

    Send("ConfigSetParameter", param, value, subValue, ordinal);
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigGetParameter(int param, double* value, int ordinal)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);

    *value = m_ConfigGetParameter_value.Get();
//    *param = m_ConfigGetParameter_param.Get();
//    *ordinal = m_ConfigGetParameter_ordinal.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);

//    *valueToSend = m_ConfigGetParameter_6_valueToSend.Get();
    *valueRecieved = m_ConfigGetParameter_6_valueRecieved.Get();
    *subValue = m_ConfigGetParameter_6_subValue.Get();
//    *param = m_ConfigGetParameter_6_param.Get();
//    *ordinal = m_ConfigGetParameter_6_ordinal.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigSetCustomParam(int newValue, int paramIndex)
{
    m_ConfigSetCustomParam_paramIndex.Set(paramIndex);
    m_ConfigSetCustomParam_newValue.Set(newValue);

    Send("ConfigSetCustomParam", newValue, paramIndex);
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigGetCustomParam(int* readValue, int paramIndex)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);

    *readValue = m_ConfigGetCustomParam_readValue.Get();
//    *paramIndex = m_ConfigGetCustomParam_paramIndex.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::ConfigFactoryDefault()
{

    Send("ConfigFactoryDefault");
}

void SnobotSim::CtrePigeonIMUWrapper::SetYaw(double angleDeg)
{
    m_Yaw_angleDeg.Set(angleDeg);

    Send("SetYaw", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::AddYaw(double angleDeg)
{
    m_AddYaw_angleDeg.Set(angleDeg);

    Send("AddYaw", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::SetYawToCompass()
{

    Send("SetYawToCompass");
}

void SnobotSim::CtrePigeonIMUWrapper::SetFusedHeading(double angleDeg)
{
    m_FusedHeading_angleDeg.Set(angleDeg);

    Send("SetFusedHeading", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::AddFusedHeading(double angleDeg)
{
    m_AddFusedHeading_angleDeg.Set(angleDeg);

    Send("AddFusedHeading", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::SetFusedHeadingToCompass()
{

    Send("SetFusedHeadingToCompass");
}

void SnobotSim::CtrePigeonIMUWrapper::SetAccumZAngle(double angleDeg)
{
    m_AccumZAngle_angleDeg.Set(angleDeg);

    Send("SetAccumZAngle", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::SetTemperatureCompensationDisable(int bTempCompDisable)
{
    m_TemperatureCompensationDisable_bTempCompDisable.Set(bTempCompDisable);

    Send("SetTemperatureCompensationDisable", bTempCompDisable);
}

void SnobotSim::CtrePigeonIMUWrapper::SetCompassDeclination(double angleDegOffset)
{
    m_CompassDeclination_angleDegOffset.Set(angleDegOffset);

    Send("SetCompassDeclination", angleDegOffset);
}

void SnobotSim::CtrePigeonIMUWrapper::SetCompassAngle(double angleDeg)
{
    m_CompassAngle_angleDeg.Set(angleDeg);

    Send("SetCompassAngle", angleDeg);
}

void SnobotSim::CtrePigeonIMUWrapper::EnterCalibrationMode(int calMode)
{
    m_EnterCalibrationMode_calMode.Set(calMode);

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

    *upTimeSec = m_GeneralStatus_upTimeSec.Get();
    *tempCompensationCount = m_GeneralStatus_tempCompensationCount.Get();
    *tempC = m_GeneralStatus_tempC.Get();
    *state = m_GeneralStatus_state.Get();
    *noMotionBiasCount = m_GeneralStatus_noMotionBiasCount.Get();
    *lastError = m_GeneralStatus_lastError.Get();
    *currentMode = m_GeneralStatus_currentMode.Get();
    *calibrationError = m_GeneralStatus_calibrationError.Get();
    *bCalIsBooting = m_GeneralStatus_bCalIsBooting.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::Get6dQuaternion(double wxyz[4])
{
    RECEIVE_HELPER("Get6dQuaternion", sizeof(wxyz[0]) + sizeof(wxyz[1]) + sizeof(wxyz[2]) + sizeof(wxyz[3]));
    PoplateReceiveResults(buffer, &wxyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[2], buffer_pos);
    PoplateReceiveResults(buffer, &wxyz[3], buffer_pos);

    *wxyz = m_6dQuaternion_wxyz.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetYawPitchRoll(double ypr[3])
{
    RECEIVE_HELPER("GetYawPitchRoll", sizeof(ypr[0]) + sizeof(ypr[1]) + sizeof(ypr[2]));
    PoplateReceiveResults(buffer, &ypr[0], buffer_pos);
    PoplateReceiveResults(buffer, &ypr[1], buffer_pos);
    PoplateReceiveResults(buffer, &ypr[2], buffer_pos);

    *ypr = m_YawPitchRoll_ypr.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetAccumGyro(double xyz_deg[3])
{
    RECEIVE_HELPER("GetAccumGyro", sizeof(xyz_deg[0]) + sizeof(xyz_deg[1]) + sizeof(xyz_deg[2]));
    PoplateReceiveResults(buffer, &xyz_deg[0], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_deg[1], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_deg[2], buffer_pos);

    *xyz_deg = m_AccumGyro_xyz_deg.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetAbsoluteCompassHeading(double* value)
{
    RECEIVE_HELPER("GetAbsoluteCompassHeading", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_AbsoluteCompassHeading_value.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetCompassHeading(double* value)
{
    RECEIVE_HELPER("GetCompassHeading", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_CompassHeading_value.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetCompassFieldStrength(double* value)
{
    RECEIVE_HELPER("GetCompassFieldStrength", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_CompassFieldStrength_value.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetTemp(double* value)
{
    RECEIVE_HELPER("GetTemp", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_Temp_value.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetState(int* state)
{
    RECEIVE_HELPER("GetState", sizeof(*state));
    PoplateReceiveResults(buffer, state, buffer_pos);

    *state = m_State_state.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetUpTime(int* value)
{
    RECEIVE_HELPER("GetUpTime", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_UpTime_value.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetRawMagnetometer(short rm_xyz[3])
{
    RECEIVE_HELPER("GetRawMagnetometer", sizeof(rm_xyz[0]) + sizeof(rm_xyz[1]) + sizeof(rm_xyz[2]));
    PoplateReceiveResults(buffer, &rm_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &rm_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &rm_xyz[2], buffer_pos);

    *rm_xyz = m_RawMagnetometer_rm_xyz.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetBiasedMagnetometer(short bm_xyz[3])
{
    RECEIVE_HELPER("GetBiasedMagnetometer", sizeof(bm_xyz[0]) + sizeof(bm_xyz[1]) + sizeof(bm_xyz[2]));
    PoplateReceiveResults(buffer, &bm_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &bm_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &bm_xyz[2], buffer_pos);

    *bm_xyz = m_BiasedMagnetometer_bm_xyz.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetBiasedAccelerometer(short ba_xyz[3])
{
    RECEIVE_HELPER("GetBiasedAccelerometer", sizeof(ba_xyz[0]) + sizeof(ba_xyz[1]) + sizeof(ba_xyz[2]));
    PoplateReceiveResults(buffer, &ba_xyz[0], buffer_pos);
    PoplateReceiveResults(buffer, &ba_xyz[1], buffer_pos);
    PoplateReceiveResults(buffer, &ba_xyz[2], buffer_pos);

    *ba_xyz = m_BiasedAccelerometer_ba_xyz.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetRawGyro(double xyz_dps[3])
{
    RECEIVE_HELPER("GetRawGyro", sizeof(xyz_dps[0]) + sizeof(xyz_dps[1]) + sizeof(xyz_dps[2]));
    PoplateReceiveResults(buffer, &xyz_dps[0], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_dps[1], buffer_pos);
    PoplateReceiveResults(buffer, &xyz_dps[2], buffer_pos);

    *xyz_dps = m_RawGyro_xyz_dps.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetAccelerometerAngles(double tiltAngles[3])
{
    RECEIVE_HELPER("GetAccelerometerAngles", sizeof(tiltAngles[0]) + sizeof(tiltAngles[1]) + sizeof(tiltAngles[2]));
    PoplateReceiveResults(buffer, &tiltAngles[0], buffer_pos);
    PoplateReceiveResults(buffer, &tiltAngles[1], buffer_pos);
    PoplateReceiveResults(buffer, &tiltAngles[2], buffer_pos);

    *tiltAngles = m_AccelerometerAngles_tiltAngles.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetFusedHeading2(int* bIsFusing, int* bIsValid, double* value, int* lastError)
{
    RECEIVE_HELPER("GetFusedHeading2", sizeof(*bIsFusing) + sizeof(*bIsValid) + sizeof(*value) + sizeof(*lastError));
    PoplateReceiveResults(buffer, bIsFusing, buffer_pos);
    PoplateReceiveResults(buffer, bIsValid, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, lastError, buffer_pos);

    *value = m_FusedHeading2_value.Get();
    *lastError = m_FusedHeading2_lastError.Get();
    *bIsValid = m_FusedHeading2_bIsValid.Get();
    *bIsFusing = m_FusedHeading2_bIsFusing.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetFusedHeading1(double* value)
{
    RECEIVE_HELPER("GetFusedHeading1", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_FusedHeading1_value.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetResetCount(int* value)
{
    RECEIVE_HELPER("GetResetCount", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_ResetCount_value.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetResetFlags(int* value)
{
    RECEIVE_HELPER("GetResetFlags", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_ResetFlags_value.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetFirmwareVersion(int* firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*firmwareVers));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);

    *firmwareVers = m_FirmwareVersion_firmwareVers.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::HasResetOccurred(bool* hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*hasReset));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);

    *hasReset = m_HasResetOccurred_hasReset.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::SetLastError(int value)
{
    m_LastError_value.Set(value);

    Send("SetLastError", value);
}

void SnobotSim::CtrePigeonIMUWrapper::GetFaults(int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_Faults_param.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::GetStickyFaults(int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_StickyFaults_param.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::ClearStickyFaults()
{

    Send("ClearStickyFaults");
}

void SnobotSim::CtrePigeonIMUWrapper::SetStatusFramePeriod(int frame, uint8_t periodMs)
{
    m_StatusFramePeriod_periodMs.Set(periodMs);
    m_StatusFramePeriod_frame.Set(frame);

    Send("SetStatusFramePeriod", frame, periodMs);
}

void SnobotSim::CtrePigeonIMUWrapper::GetStatusFramePeriod(int frame, int* periodMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);

    *periodMs = m_StatusFramePeriod_periodMs.Get();
//    *frame = m_StatusFramePeriod_frame.Get();
}

void SnobotSim::CtrePigeonIMUWrapper::SetControlFramePeriod(int frame, int periodMs)
{
    m_ControlFramePeriod_periodMs.Set(periodMs);
    m_ControlFramePeriod_frame.Set(frame);

    Send("SetControlFramePeriod", frame, periodMs);
}

ctre::phoenix::ErrorCode SnobotSim::CtrePigeonIMUWrapper::GetLastError()
{
    int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)lastError;
}
