
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/CANifier_CCI.h"
#include "simulation/SimDeviceSim.h"

namespace SnobotSim
{

class CtreCANifierWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    explicit CtreCANifierWrapper(int aDeviceId);
    const int mDeviceId;

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);

    ctre::phoenix::ErrorCode GetLastError();

    //////////////////////////////////////////
    void GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled);
    void SetLEDOutput(uint32_t dutyCycle, uint32_t ledChannel);
    void SetGeneralOutputs(uint32_t outputsBits, uint32_t isOutputBits);
    void SetGeneralOutput(uint32_t outputPin, bool outputValue, bool outputEnable);
    void SetPWMOutput(uint32_t pwmChannel, uint32_t dutyCycle);
    void EnablePWMOutput(uint32_t pwmChannel, bool bEnable);
    void GetGeneralInputs(bool allPins[], uint32_t capacity);
    void GetGeneralInput(uint32_t inputPin, bool* measuredInput);
    void GetPWMInput(uint32_t pwmChannel, double dutyCycleAndPeriod[2]);
    void GetBusVoltage(double* batteryVoltage);
    void GetQuadraturePosition(int* pos);
    void SetQuadraturePosition(int pos);
    void GetQuadratureVelocity(int* vel);
    void GetQuadratureSensor(int* pos, int* vel);
    void ConfigVelocityMeasurementPeriod(int period);
    void ConfigVelocityMeasurementWindow(int window);
    void ConfigClearPositionOnLimitF(bool clearPositionOnLimitF);
    void ConfigClearPositionOnLimitR(bool clearPositionOnLimitR);
    void ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx);
    void SetLastError(int error);
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
    void SetControlFramePeriod(int frame, int periodMs);

protected:
    hal::SimDevice m_simDevice;

    hal::SimDouble m_BusVoltage_batteryVoltage;
    hal::SimDouble m_ConfigClearPositionOnLimitF_clearPositionOnLimitF;
    hal::SimDouble m_ConfigClearPositionOnLimitR_clearPositionOnLimitR;
    hal::SimDouble m_ConfigClearPositionOnQuadIdx_clearPositionOnQuadIdx;
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
    hal::SimDouble m_ConfigVelocityMeasurementPeriod_period;
    hal::SimDouble m_ConfigVelocityMeasurementWindow_window;
    hal::SimDouble m_ControlFramePeriod_frame;
    hal::SimDouble m_ControlFramePeriod_periodMs;
    hal::SimDouble m_Create1_deviceNumber;
    hal::SimDouble m_Description_numBytesFilled;
    hal::SimDouble m_Description_toFill;
    hal::SimDouble m_Description_toFillByteSz;
    hal::SimDouble m_EnablePWMOutput_bEnable;
    hal::SimDouble m_EnablePWMOutput_pwmChannel;
    hal::SimDouble m_Faults_param;
    hal::SimDouble m_FirmwareVersion_firmwareVers;
    hal::SimDouble m_GeneralInput_inputPin;
    hal::SimDouble m_GeneralInput_measuredInput;
    hal::SimDouble m_GeneralInputs_allPins;
    hal::SimDouble m_GeneralInputs_capacity;
    hal::SimDouble m_GeneralOutput_outputEnable;
    hal::SimDouble m_GeneralOutput_outputPin;
    hal::SimDouble m_GeneralOutput_outputValue;
    hal::SimDouble m_GeneralOutputs_isOutputBits;
    hal::SimDouble m_GeneralOutputs_outputsBits;
    hal::SimDouble m_HasResetOccurred_hasReset;
    hal::SimDouble m_LEDOutput_dutyCycle;
    hal::SimDouble m_LEDOutput_ledChannel;
    hal::SimDouble m_LastError_error;
    hal::SimDouble m_PWMInput_dutyCycleAndPeriod;
    hal::SimDouble m_PWMInput_pwmChannel;
    hal::SimDouble m_PWMOutput_dutyCycle;
    hal::SimDouble m_PWMOutput_pwmChannel;
    hal::SimDouble m_QuadraturePosition_pos;
    hal::SimDouble m_QuadratureSensor_pos;
    hal::SimDouble m_QuadratureSensor_vel;
    hal::SimDouble m_QuadratureVelocity_vel;
    hal::SimDouble m_StatusFramePeriod_frame;
    hal::SimDouble m_StatusFramePeriod_periodMs;
    hal::SimDouble m_StickyFaults_param;
};

} // namespace SnobotSim
