
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/CANifier_CCI.h"

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

    /////////////////////////////////////////////////////////////
    void GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled);
    void SetLEDOutput(uint32_t dutyCycle, uint32_t ledChannel);
    void SetGeneralOutputs(uint32_t outputsBits, uint32_t isOutputBits);
    void SetGeneralOutput(uint32_t outputPin, bool outputValue, bool outputEnable);
    void SetPWMOutput(uint32_t pwmChannel, uint32_t dutyCycle);
    void EnablePWMOutput(uint32_t pwmChannel, bool bEnable);
    void GetGeneralInputs(bool allPins[], uint32_t capacity);
    void GetGeneralInput(uint32_t inputPin, bool* measuredInput);
    void GetPWMInput(uint32_t pwmChannel, double dutyCycleAndPeriod[2]);
    ctre::phoenix::ErrorCode GetLastError();
    void GetBusVoltage(double* batteryVoltage);
    void GetQuadraturePosition(int* pos);
    void SetQuadraturePosition(int pos, int timeoutMs);
    void GetQuadratureVelocity(int* vel);
    void GetQuadratureSensor(int* pos, int* vel);
    void ConfigVelocityMeasurementPeriod(int period, int timeoutMs);
    void ConfigVelocityMeasurementWindow(int window, int timeoutMs);
    void ConfigClearPositionOnLimitF(bool clearPositionOnLimitF, int timeoutMs);
    void ConfigClearPositionOnLimitR(bool clearPositionOnLimitR, int timeoutMs);
    void ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx, int timeoutMs);
    void SetLastError(int error);
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
    void SetControlFramePeriod(int frame, int periodMs);
};

} // namespace SnobotSim
