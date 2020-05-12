
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
};

} // namespace SnobotSim
