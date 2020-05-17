
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/CANCoder_CCI.h"
#include "simulation/SimDeviceSim.h"

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

protected:
    hal::SimDevice m_simDevice;

    hal::SimDouble m_AbsolutePosition_pos;
    hal::SimDouble m_BusVoltage_batteryVoltage;
    hal::SimDouble m_ConfigAbsoluteSensorRange_absoluteSensorRange;
    hal::SimDouble m_ConfigFeedbackCoefficient_sensorCoefficient;
    hal::SimDouble m_ConfigFeedbackCoefficient_sensortimeBase;
    hal::SimDouble m_ConfigFeedbackCoefficient_unitString;
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
    hal::SimDouble m_ConfigMagnetOffset_offsetDegrees;
    hal::SimDouble m_ConfigSensorDirection_bDirection;
    hal::SimDouble m_ConfigSensorInitializationStrategy_initializationStrategy;
    hal::SimDouble m_ConfigSetCustomParam_newValue;
    hal::SimDouble m_ConfigSetCustomParam_paramIndex;
    hal::SimDouble m_ConfigSetParameter_ordinal;
    hal::SimDouble m_ConfigSetParameter_param;
    hal::SimDouble m_ConfigSetParameter_subValue;
    hal::SimDouble m_ConfigSetParameter_value;
    hal::SimDouble m_ConfigVelocityMeasurementPeriod_period;
    hal::SimDouble m_ConfigVelocityMeasurementWindow_window;
    hal::SimDouble m_Create1_deviceNumber;
    hal::SimDouble m_Description_numBytesFilled;
    hal::SimDouble m_Description_toFill;
    hal::SimDouble m_Description_toFillByteSz;
    hal::SimDouble m_Faults_param;
    hal::SimDouble m_FirmwareVersion_firmwareVers;
    hal::SimDouble m_HasResetOccurred_hasReset;
    hal::SimDouble m_LastTimestamp_timestamp;
    hal::SimDouble m_LastUnitString_numBytesFilled;
    hal::SimDouble m_LastUnitString_toFill;
    hal::SimDouble m_LastUnitString_toFillByteSz;
    hal::SimDouble m_MagnetFieldStrength_magnetFieldStrength;
    hal::SimDouble m_Position_pos;
    hal::SimDouble m_StatusFramePeriod_frame;
    hal::SimDouble m_StatusFramePeriod_periodMs;
    hal::SimDouble m_StickyFaults_param;
    hal::SimDouble m_Velocity_vel;
};

} // namespace SnobotSim
