#include "ctre/phoenix/cci/CANifier_CCI.h"

#include <cstring>
#include <vector>

#include "CtreSimMocks/CtreCANifierWrapper.h"
#include "CtreSimUtils/MockHooks.h"

namespace
{
SnobotSim::CtreCANifierWrapper* ConvertToWrapper(void* param)
{
    return reinterpret_cast<SnobotSim::CtreCANifierWrapper*>(param);
}
} // namespace

extern "C" {

void* c_CANifier_Create1(int deviceNumber)
{
    auto* output = new SnobotSim::CtreCANifierWrapper(deviceNumber);
    return output;
}

void c_CANifier_DestroyAll(void)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}

ctre::phoenix::ErrorCode c_CANifier_Destroy(void* handle)
{
    auto* wrapper = ConvertToWrapper(handle);
    delete wrapper;
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetDescription(void* handle, char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    ConvertToWrapper(handle)->GetDescription(toFill, toFillByteSz, numBytesFilled);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetLEDOutput(void* handle, uint32_t dutyCycle, uint32_t ledChannel)
{
    ConvertToWrapper(handle)->SetLEDOutput(dutyCycle, ledChannel);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetGeneralOutputs(void* handle, uint32_t outputsBits, uint32_t isOutputBits)
{
    ConvertToWrapper(handle)->SetGeneralOutputs(outputsBits, isOutputBits);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetGeneralOutput(void* handle, uint32_t outputPin, bool outputValue, bool outputEnable)
{
    ConvertToWrapper(handle)->SetGeneralOutput(outputPin, outputValue, outputEnable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetPWMOutput(void* handle, uint32_t pwmChannel, uint32_t dutyCycle)
{
    ConvertToWrapper(handle)->SetPWMOutput(pwmChannel, dutyCycle);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_EnablePWMOutput(void* handle, uint32_t pwmChannel, bool bEnable)
{
    ConvertToWrapper(handle)->EnablePWMOutput(pwmChannel, bEnable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetGeneralInputs(void* handle, bool allPins[], uint32_t capacity)
{
    ConvertToWrapper(handle)->GetGeneralInputs(allPins, capacity);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetGeneralInput(void* handle, uint32_t inputPin, bool* measuredInput)
{
    ConvertToWrapper(handle)->GetGeneralInput(inputPin, measuredInput);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetPWMInput(void* handle, uint32_t pwmChannel, double dutyCycleAndPeriod[2])
{
    ConvertToWrapper(handle)->GetPWMInput(pwmChannel, dutyCycleAndPeriod);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetLastError(void* handle)
{
    return ConvertToWrapper(handle)->GetLastError();
}

ctre::phoenix::ErrorCode c_CANifier_GetBusVoltage(void* handle, double* batteryVoltage)
{
    ConvertToWrapper(handle)->GetBusVoltage(batteryVoltage);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetQuadraturePosition(void* handle, int* pos)
{
    ConvertToWrapper(handle)->GetQuadraturePosition(pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetQuadraturePosition(void* handle, int pos, int timeoutMs)
{
    ConvertToWrapper(handle)->SetQuadraturePosition(pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetQuadratureVelocity(void* handle, int* vel)
{
    ConvertToWrapper(handle)->GetQuadratureVelocity(vel);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetQuadratureSensor(void* handle, int* pos, int* vel)
{
    ConvertToWrapper(handle)->GetQuadratureSensor(pos, vel);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigVelocityMeasurementPeriod(void* handle, int period, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigVelocityMeasurementPeriod(period);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigVelocityMeasurementWindow(void* handle, int window, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigVelocityMeasurementWindow(window);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigClearPositionOnLimitF(void* handle, bool clearPositionOnLimitF, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigClearPositionOnLimitF(clearPositionOnLimitF);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigClearPositionOnLimitR(void* handle, bool clearPositionOnLimitR, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigClearPositionOnLimitR(clearPositionOnLimitR);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigClearPositionOnQuadIdx(void* handle, bool clearPositionOnQuadIdx, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigClearPositionOnQuadIdx(clearPositionOnQuadIdx);
    return (ctre::phoenix::ErrorCode)0;
}

void c_CANifier_SetLastError(void* handle, int error)
{
    ConvertToWrapper(handle)->SetLastError(error);
}

ctre::phoenix::ErrorCode c_CANifier_ConfigSetParameter(void* handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSetParameter(param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigGetParameter(void* handle, int param, double* value, int ordinal, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetParameter(param, value, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigGetParameter_6(void* handle, int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetParameter_6(param, valueToSend, valueRecieved, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigSetCustomParam(void* handle, int newValue, int paramIndex, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSetCustomParam(newValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigGetCustomParam(void* handle, int* readValue, int paramIndex, int timoutMs)
{
    ConvertToWrapper(handle)->ConfigGetCustomParam(readValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigFactoryDefault(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigFactoryDefault();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetFaults(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetFaults(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetStickyFaults(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetStickyFaults(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ClearStickyFaults(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->ClearStickyFaults();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetFirmwareVersion(void* handle, int* firmwareVers)
{
    ConvertToWrapper(handle)->GetFirmwareVersion(firmwareVers);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_HasResetOccurred(void* handle, bool* hasReset)
{
    ConvertToWrapper(handle)->HasResetOccurred(hasReset);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetStatusFramePeriod(void* handle, int frame, uint8_t periodMs, int timeoutMs)
{
    ConvertToWrapper(handle)->SetStatusFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetStatusFramePeriod(void* handle, int frame, int* periodMs, int timeoutMs)
{
    ConvertToWrapper(handle)->GetStatusFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetControlFramePeriod(void* handle, int frame, int periodMs)
{
    ConvertToWrapper(handle)->SetControlFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

} // extern "C"
