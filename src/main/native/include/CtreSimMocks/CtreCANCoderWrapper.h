
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/CANCoder_CCI.h"

namespace SnobotSim
{

class CtreCANCoderWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    explicit CtreCANCoderWrapper(int aDeviceId);
    const int mDeviceId;

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);

    ctre::phoenix::ErrorCode GetLastError();

    //////////////////////////////////////////
    void GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled);
    void GetLastUnitString(char* toFill, int toFillByteSz, int* numBytesFilled);
    void GetLastTimestamp(double* timestamp);
    void GetBusVoltage(double* batteryVoltage);
    void GetMagnetFieldStrength(ctre::phoenix::sensors::MagnetFieldStrength* magnetFieldStrength);
    void GetPosition(double* pos);
    void SetPosition(double pos);
    void SetPositionToAbsolute();
    void ConfigSensorDirection(int bDirection);
    void GetVelocity(double* vel);
    void GetAbsolutePosition(double* pos);
    void ConfigVelocityMeasurementPeriod(int period);
    void ConfigVelocityMeasurementWindow(int window);
    void ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange);
    void ConfigMagnetOffset(double offsetDegrees);
    void ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy);
    void ConfigFeedbackCoefficient(double sensorCoefficient, const char* unitString, ctre::phoenix::sensors::SensorTimeBase sensortimeBase);
    void ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal);
    void ConfigGetParameter(int param, double* value, int ordinal);
    void ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal);
    void ConfigSetCustomParam(int newValue, int paramIndex);
    void ConfigGetCustomParam(int* readValue, int paramIndex);
    void ConfigFactoryDefault();
    void GetFaults(int* param);
    void GetStickyFaults(int* param);
    void ClearStickyFaults();
    void GetFirmwareVersion(int* firmwareVers);
    void HasResetOccurred(bool* hasReset);
    void SetStatusFramePeriod(int frame, uint8_t periodMs);
    void GetStatusFramePeriod(int frame, int* periodMs);
};

} // namespace SnobotSim
