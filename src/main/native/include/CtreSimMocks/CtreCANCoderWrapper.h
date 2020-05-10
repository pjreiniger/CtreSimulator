
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

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);

    //////////////////////////////////////////////////

    void GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled);
    ctre::phoenix::ErrorCode GetLastError();
    void GetLastUnitString(char* toFill, int toFillByteSz, int* numBytesFilled);
    void GetLastTimestamp(double* timestamp);

    void GetBusVoltage(double* batteryVoltage);
    void GetMagnetFieldStrength(ctre::phoenix::sensors::MagnetFieldStrength* magnetFieldStrength);
    void GetPosition(double* pos);
    void SetPosition(double pos, int timeoutMs);
    void SetPositionToAbsolute(int timeoutMs);
    void ConfigSensorDirection(int bDirection, int timeoutMs);
    void GetVelocity(double* vel);
    void GetAbsolutePosition(double* pos);
    void ConfigVelocityMeasurementPeriod(int period, int timeoutMs);
    void ConfigVelocityMeasurementWindow(int window, int timeoutMs);
    void ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange, int timeoutMs);
    void ConfigMagnetOffset(double offsetDegrees, int timeoutMs);
    void ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy, int timeoutMs);
    void ConfigFeedbackCoefficient(double sensorCoefficient, const char* unitString, ctre::phoenix::sensors::SensorTimeBase sensortimeBase, int timeoutMs);
    void ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal, int timeoutMs);
    void ConfigGetParameter(int param, double* value, int ordinal, int timeoutMs);
    void ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs);
    void ConfigSetCustomParam(int newValue, int paramIndex, int timeoutMs);
    void ConfigGetCustomParam(int* readValue, int paramIndex, int timoutMs);
    void ConfigFactoryDefault(int timeoutMs);
    void GetFaults(int* param);
    void GetStickyFaults(int* param);
    void ClearStickyFaults(int timeoutMs);
    void GetFirmwareVersion(int* firmwareVers);
    void HasResetOccurred(bool* hasReset);
    void SetStatusFramePeriod(int frame, uint8_t periodMs, int timeoutMs);
    void GetStatusFramePeriod(int frame, int* periodMs, int timeoutMs);
};

} // namespace SnobotSim
