

#include "CtreSimMocks/CtreCANifierWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size) \
    uint8_t buffer[size]; /* NOLINT */  \
    std::memset(&buffer[0], 0, size);   \
    Receive(paramName, buffer, size);   \
    uint32_t buffer_pos = 0;

std::vector<SnobotSim::CTRE_CallbackFunc> gCANifierCallbacks;

void SnobotSim::SetCANifierCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gCANifierCallbacks.clear();
    gCANifierCallbacks.push_back(callback);
}

SnobotSim::CtreCANifierWrapper::CtreCANifierWrapper(int aDeviceId) :
        mDeviceId(aDeviceId)
{
    Send("Create");
}

void SnobotSim::CtreCANifierWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gCANifierCallbacks.empty())
    {
        gCANifierCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreCANifierWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    if (!gCANifierCallbacks.empty())
    {
        gCANifierCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreCANifierWrapper::GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}

void SnobotSim::CtreCANifierWrapper::SetLEDOutput(uint32_t dutyCycle, uint32_t ledChannel)
{
    Send("SetLEDOutput", dutyCycle, ledChannel);
}

void SnobotSim::CtreCANifierWrapper::SetGeneralOutputs(uint32_t outputsBits, uint32_t isOutputBits)
{
    Send("SetGeneralOutputs", outputsBits, isOutputBits);
}

void SnobotSim::CtreCANifierWrapper::SetGeneralOutput(uint32_t outputPin, bool outputValue, bool outputEnable)
{
    Send("SetGeneralOutput", outputPin, outputValue, outputEnable);
}

void SnobotSim::CtreCANifierWrapper::SetPWMOutput(uint32_t pwmChannel, uint32_t dutyCycle)
{
    Send("SetPWMOutput", pwmChannel, dutyCycle);
}

void SnobotSim::CtreCANifierWrapper::EnablePWMOutput(uint32_t pwmChannel, bool bEnable)
{
    Send("EnablePWMOutput", pwmChannel, bEnable);
}

void SnobotSim::CtreCANifierWrapper::GetGeneralInputs(bool allPins[], uint32_t capacity)
{
    RECEIVE_HELPER("GetGeneralInputs", sizeof(bool) * 11);
    for (unsigned int i = 0; i < capacity; ++i)
    {
        PoplateReceiveResults(buffer, &allPins[i], buffer_pos);
    }
}

void SnobotSim::CtreCANifierWrapper::GetGeneralInput(uint32_t inputPin, bool* measuredInput)
{
    RECEIVE_HELPER("GetGeneralInput", sizeof(inputPin) + sizeof(*measuredInput));
    PoplateReceiveResults(buffer, &inputPin, buffer_pos);
    PoplateReceiveResults(buffer, measuredInput, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::GetPWMInput(uint32_t pwmChannel, double dutyCycleAndPeriod[2])
{
    RECEIVE_HELPER("GetPWMInput", sizeof(pwmChannel) + sizeof(dutyCycleAndPeriod[0]) + sizeof(dutyCycleAndPeriod[1]));
    PoplateReceiveResults(buffer, &pwmChannel, buffer_pos);
    PoplateReceiveResults(buffer, &dutyCycleAndPeriod[0], buffer_pos);
    PoplateReceiveResults(buffer, &dutyCycleAndPeriod[1], buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::GetBusVoltage(double* batteryVoltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*batteryVoltage));
    PoplateReceiveResults(buffer, batteryVoltage, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::GetQuadraturePosition(int* pos)
{
    RECEIVE_HELPER("GetQuadraturePosition", sizeof(*pos));
    PoplateReceiveResults(buffer, pos, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::SetQuadraturePosition(int pos)
{
    Send("SetQuadraturePosition", pos);
}

void SnobotSim::CtreCANifierWrapper::GetQuadratureVelocity(int* vel)
{
    RECEIVE_HELPER("GetQuadratureVelocity", sizeof(*vel));
    PoplateReceiveResults(buffer, vel, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::GetQuadratureSensor(int* pos, int* vel)
{
    RECEIVE_HELPER("GetQuadratureSensor", sizeof(*pos) + sizeof(*vel));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::ConfigVelocityMeasurementPeriod(int period)
{
    Send("ConfigVelocityMeasurementPeriod", period);
}

void SnobotSim::CtreCANifierWrapper::ConfigVelocityMeasurementWindow(int window)
{
    Send("ConfigVelocityMeasurementWindow", window);
}

void SnobotSim::CtreCANifierWrapper::ConfigClearPositionOnLimitF(bool clearPositionOnLimitF)
{
    Send("ConfigClearPositionOnLimitF", clearPositionOnLimitF);
}

void SnobotSim::CtreCANifierWrapper::ConfigClearPositionOnLimitR(bool clearPositionOnLimitR)
{
    Send("ConfigClearPositionOnLimitR", clearPositionOnLimitR);
}

void SnobotSim::CtreCANifierWrapper::ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx)
{
    Send("ConfigClearPositionOnQuadIdx", clearPositionOnQuadIdx);
}

void SnobotSim::CtreCANifierWrapper::SetLastError(int error)
{
    Send("SetLastError", error);
}

void SnobotSim::CtreCANifierWrapper::ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal)
{
    Send("ConfigSetParameter", param, value, subValue, ordinal);
}

void SnobotSim::CtreCANifierWrapper::ConfigGetParameter(int param, double* value, int ordinal)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::ConfigSetCustomParam(int newValue, int paramIndex)
{
    Send("ConfigSetCustomParam", newValue, paramIndex);
}

void SnobotSim::CtreCANifierWrapper::ConfigGetCustomParam(int* readValue, int paramIndex)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::ConfigFactoryDefault()
{
    Send("ConfigFactoryDefault");
}

void SnobotSim::CtreCANifierWrapper::GetFaults(int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::GetStickyFaults(int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::ClearStickyFaults()
{
    Send("ClearStickyFaults");
}

void SnobotSim::CtreCANifierWrapper::GetFirmwareVersion(int* firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*firmwareVers));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::HasResetOccurred(bool* hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*hasReset));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::SetStatusFramePeriod(int frame, uint8_t periodMs)
{
    Send("SetStatusFramePeriod", frame, periodMs);
}

void SnobotSim::CtreCANifierWrapper::GetStatusFramePeriod(int frame, int* periodMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
}

void SnobotSim::CtreCANifierWrapper::SetControlFramePeriod(int frame, int periodMs)
{
    Send("SetControlFramePeriod", frame, periodMs);
}

ctre::phoenix::ErrorCode SnobotSim::CtreCANifierWrapper::GetLastError()
{
    int error = 0;
    RECEIVE_HELPER("GetLastError", sizeof(error));
    PoplateReceiveResults(buffer, &error, buffer_pos);
    return (ctre::phoenix::ErrorCode)error;
}
