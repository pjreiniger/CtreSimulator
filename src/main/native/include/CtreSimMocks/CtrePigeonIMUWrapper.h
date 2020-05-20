
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/PigeonIMU_CCI.h"
#include "simulation/SimDeviceSim.h"

namespace SnobotSim
{

class CtrePigeonIMUWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    explicit CtrePigeonIMUWrapper(int aDeviceId);
    const int mDeviceId;

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);

    ctre::phoenix::ErrorCode GetLastError();

    //////////////////////////////////////////
    void GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled);
    void ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal);
    void ConfigGetParameter(int param, double* value, int ordinal);
    void ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal);
    void ConfigSetCustomParam(int newValue, int paramIndex);
    void ConfigGetCustomParam(int* readValue, int paramIndex);
    void ConfigFactoryDefault();
    void SetYaw(double angleDeg);
    void AddYaw(double angleDeg);
    void SetYawToCompass();
    void SetFusedHeading(double angleDeg);
    void AddFusedHeading(double angleDeg);
    void SetFusedHeadingToCompass();
    void SetAccumZAngle(double angleDeg);
    void SetTemperatureCompensationDisable(int bTempCompDisable);
    void SetCompassDeclination(double angleDegOffset);
    void SetCompassAngle(double angleDeg);
    void EnterCalibrationMode(int calMode);
    void GetGeneralStatus(int* state, int* currentMode, int* calibrationError, int* bCalIsBooting, double* tempC, int* upTimeSec, int* noMotionBiasCount, int* tempCompensationCount, int* lastError);
    void Get6dQuaternion(double wxyz[4]);
    void GetYawPitchRoll(double ypr[3]);
    void GetAccumGyro(double xyz_deg[3]);
    void GetAbsoluteCompassHeading(double* value);
    void GetCompassHeading(double* value);
    void GetCompassFieldStrength(double* value);
    void GetTemp(double* value);
    void GetState(int* state);
    void GetUpTime(int* value);
    void GetRawMagnetometer(short rm_xyz[3]);
    void GetBiasedMagnetometer(short bm_xyz[3]);
    void GetBiasedAccelerometer(short ba_xyz[3]);
    void GetRawGyro(double xyz_dps[3]);
    void GetAccelerometerAngles(double tiltAngles[3]);
    void GetFusedHeading2(int* bIsFusing, int* bIsValid, double* value, int* lastError);
    void GetFusedHeading1(double* value);
    void GetResetCount(int* value);
    void GetResetFlags(int* value);
    void GetFirmwareVersion(int* firmwareVers);
    void HasResetOccurred(bool* hasReset);
    void SetLastError(int value);
    void GetFaults(int* param);
    void GetStickyFaults(int* param);
    void ClearStickyFaults();
    void SetStatusFramePeriod(int frame, uint8_t periodMs);
    void GetStatusFramePeriod(int frame, int* periodMs);
    void SetControlFramePeriod(int frame, int periodMs);

protected:
    hal::SimDevice m_simDevice;

    hal::SimDouble m_6dQuaternion_wxyz;
    hal::SimDouble m_AbsoluteCompassHeading_value;
    hal::SimDouble m_AccelerometerAngles_tiltAngles;
    hal::SimDouble m_AccumGyro_xyz_deg;
    hal::SimDouble m_AccumZAngle_angleDeg;
    hal::SimDouble m_AddFusedHeading_angleDeg;
    hal::SimDouble m_AddYaw_angleDeg;
    hal::SimDouble m_BiasedAccelerometer_ba_xyz;
    hal::SimDouble m_BiasedMagnetometer_bm_xyz;
    hal::SimDouble m_CompassAngle_angleDeg;
    hal::SimDouble m_CompassDeclination_angleDegOffset;
    hal::SimDouble m_CompassFieldStrength_value;
    hal::SimDouble m_CompassHeading_value;
    hal::SimDouble m_ConfigGetCustomParam_paramIndex;
    hal::SimDouble m_ConfigGetCustomParam_readValue;
    hal::SimDouble m_ConfigGetParameter_6_ordinal;
    hal::SimDouble m_ConfigGetParameter_6_param;
    hal::SimDouble m_ConfigGetParameter_6_subValue;
    hal::SimDouble m_ConfigGetParameter_6_valueRecieved;
    hal::SimDouble m_ConfigGetParameter_6_valueToSend;
    hal::SimDouble m_ConfigGetParameter_ordinal;
    hal::SimDouble m_ConfigGetParameter_param;
    hal::SimDouble m_ConfigGetParameter_value;
    hal::SimDouble m_ConfigSetCustomParam_newValue;
    hal::SimDouble m_ConfigSetCustomParam_paramIndex;
    hal::SimDouble m_ConfigSetParameter_ordinal;
    hal::SimDouble m_ConfigSetParameter_param;
    hal::SimDouble m_ConfigSetParameter_subValue;
    hal::SimDouble m_ConfigSetParameter_value;
    hal::SimDouble m_ControlFramePeriod_frame;
    hal::SimDouble m_ControlFramePeriod_periodMs;
    hal::SimDouble m_Create1_deviceNumber;
    hal::SimDouble m_Create2_talonDeviceID;
    hal::SimDouble m_Description_numBytesFilled;
    hal::SimDouble m_Description_toFill;
    hal::SimDouble m_Description_toFillByteSz;
    hal::SimDouble m_EnterCalibrationMode_calMode;
    hal::SimDouble m_Faults_param;
    hal::SimDouble m_FirmwareVersion_firmwareVers;
    hal::SimDouble m_FusedHeading1_value;
    hal::SimDouble m_FusedHeading2_bIsFusing;
    hal::SimDouble m_FusedHeading2_bIsValid;
    hal::SimDouble m_FusedHeading2_lastError;
    hal::SimDouble m_FusedHeading2_value;
    hal::SimDouble m_FusedHeading_angleDeg;
    hal::SimDouble m_GeneralStatus_bCalIsBooting;
    hal::SimDouble m_GeneralStatus_calibrationError;
    hal::SimDouble m_GeneralStatus_currentMode;
    hal::SimDouble m_GeneralStatus_lastError;
    hal::SimDouble m_GeneralStatus_noMotionBiasCount;
    hal::SimDouble m_GeneralStatus_state;
    hal::SimDouble m_GeneralStatus_tempC;
    hal::SimDouble m_GeneralStatus_tempCompensationCount;
    hal::SimDouble m_GeneralStatus_upTimeSec;
    hal::SimDouble m_HasResetOccurred_hasReset;
    hal::SimDouble m_LastError_value;
    hal::SimDouble m_RawGyro_xyz_dps_0;
    hal::SimDouble m_RawGyro_xyz_dps_1;
    hal::SimDouble m_RawGyro_xyz_dps_2;
    hal::SimDouble m_RawMagnetometer_rm_xyz;
    hal::SimDouble m_ResetCount_value;
    hal::SimDouble m_ResetFlags_value;
    hal::SimDouble m_State_state;
    hal::SimDouble m_StatusFramePeriod_frame;
    hal::SimDouble m_StatusFramePeriod_periodMs;
    hal::SimDouble m_StickyFaults_param;
    hal::SimDouble m_Temp_value;
    hal::SimDouble m_TemperatureCompensationDisable_bTempCompDisable;
    hal::SimDouble m_UpTime_value;
    hal::SimDouble m_YawPitchRoll_ypr_0;
    hal::SimDouble m_YawPitchRoll_ypr_1;
    hal::SimDouble m_YawPitchRoll_ypr_2;
    hal::SimDouble m_Yaw_angleDeg;
};

} // namespace SnobotSim
