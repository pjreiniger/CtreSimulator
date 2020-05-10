
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/PigeonIMU_CCI.h"

namespace SnobotSim
{

class CtrePigeonIMUWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    explicit CtrePigeonIMUWrapper(int aDeviceId);
    const int mDeviceId;

    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);

    //////////////////////////////////////////////////////////////////
    void GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled);
    void ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal);
    void ConfigGetParameter(int param, double* value, int ordinal);
    void ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal);
    void ConfigSetCustomParam(int newValue, int paramIndex);
    void ConfigGetCustomParam(int* readValue, int paramIndex, int timoutMs);
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
    ctre::phoenix::ErrorCode GetLastError();
    void Get6dQuaternion(double wxyz[4]);
    void GetYawPitchRoll(double ypr[3]);
    void GetAccumGyro(double xyz_deg[3]);
    void GetAbsoluteCompassHeading(double* value);
    void GetCompassHeading(double* value);
    void GetCompassFieldStrength(double* value);
    void GetTemp(double* value);
    void GetState(int* state);
    void GetUpTime(int* value);
    void GetRawMagnetometer(short rm_xyz[3]); // NOLINT(runtime/int)
    void GetBiasedMagnetometer(short bm_xyz[3]); // NOLINT(runtime/int)
    void GetBiasedAccelerometer(short ba_xyz[3]); // NOLINT(runtime/int)
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
};

} // namespace SnobotSim
