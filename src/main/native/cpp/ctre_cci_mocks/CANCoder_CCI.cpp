#include "ctre/phoenix/cci/CANCoder_CCI.h"

#include <cstring>
#include <vector>

#include "CtreSimMocks/CtreCANCoderWrapper.h"
#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size)        \
    auto* wrapper = ConvertToWrapper(handle);  \
    uint8_t buffer[size]; /* NOLINT */         \
    std::memset(&buffer[0], 0, size);          \
    wrapper->Receive(paramName, buffer, size); \
    uint32_t buffer_pos = 0;

namespace
{
SnobotSim::CtreCANCoderWrapper* ConvertToWrapper(void* param)
{
    return reinterpret_cast<SnobotSim::CtreCANCoderWrapper*>(param);
}
} // namespace

extern "C" {

void* c_CANCoder_Create1(int deviceNumber)
{
    auto* output = new SnobotSim::CtreCANCoderWrapper(deviceNumber);
    return output;
}

void c_CANCoder_DestroyAll(void)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}

ctre::phoenix::ErrorCode c_CANCoder_Destroy(void* handle)
{
    auto* wrapper = ConvertToWrapper(handle);
    delete wrapper;
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetDescription(void* handle, char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    RECEIVE_HELPER("GetDescription", 1);
    buffer_pos += 1; // Removes compiler warning
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetLastError(void* handle)
{
    int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)lastError;
}

ctre::phoenix::ErrorCode c_CANCoder_GetLastUnitString(void* handle, char* toFill, int toFillByteSz, int* numBytesFilled)
{
    ConvertToWrapper(handle)->GetLastUnitString(toFill, toFillByteSz, numBytesFilled);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetLastTimestamp(void* handle, double* timestamp)
{
    ConvertToWrapper(handle)->GetLastTimestamp(timestamp);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetBusVoltage(void* handle, double* batteryVoltage)
{
    ConvertToWrapper(handle)->GetBusVoltage(batteryVoltage);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetMagnetFieldStrength(void* handle, ctre::phoenix::sensors::MagnetFieldStrength* magnetFieldStrength)
{
    ConvertToWrapper(handle)->GetMagnetFieldStrength(magnetFieldStrength);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetPosition(void* handle, double* pos)
{
    ConvertToWrapper(handle)->GetPosition(pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_SetPosition(void* handle, double pos, int timeoutMs)
{
    ConvertToWrapper(handle)->SetPosition(pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_SetPositionToAbsolute(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->SetPositionToAbsolute();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigSensorDirection(void* handle, int bDirection, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSensorDirection(bDirection);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetVelocity(void* handle, double* vel)
{
    ConvertToWrapper(handle)->GetVelocity(vel);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetAbsolutePosition(void* handle, double* pos)
{
    ConvertToWrapper(handle)->GetAbsolutePosition(pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigVelocityMeasurementPeriod(void* handle, int period, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigVelocityMeasurementPeriod(period);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigVelocityMeasurementWindow(void* handle, int window, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigVelocityMeasurementWindow(window);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigAbsoluteSensorRange(void* handle, ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigAbsoluteSensorRange(absoluteSensorRange);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigMagnetOffset(void* handle, double offsetDegrees, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigMagnetOffset(offsetDegrees);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigSensorInitializationStrategy(void* handle, ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSensorInitializationStrategy(initializationStrategy);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigFeedbackCoefficient(void* handle, double sensorCoefficient, const char* unitString, ctre::phoenix::sensors::SensorTimeBase sensortimeBase, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigFeedbackCoefficient(sensorCoefficient, unitString, sensortimeBase);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigSetParameter(void* handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSetParameter(param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigGetParameter(void* handle, int param, double* value, int ordinal, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetParameter(param, value, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigGetParameter_6(void* handle, int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetParameter_6(param, valueToSend, valueRecieved, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigSetCustomParam(void* handle, int newValue, int paramIndex, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSetCustomParam(newValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigGetCustomParam(void* handle, int* readValue, int paramIndex, int timoutMs)
{
    ConvertToWrapper(handle)->ConfigGetCustomParam(readValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigFactoryDefault(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigFactoryDefault();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetFaults(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetFaults(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetStickyFaults(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetStickyFaults(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ClearStickyFaults(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->ClearStickyFaults();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetFirmwareVersion(void* handle, int* firmwareVers)
{
    ConvertToWrapper(handle)->GetFirmwareVersion(firmwareVers);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_HasResetOccurred(void* handle, bool* hasReset)
{
    ConvertToWrapper(handle)->HasResetOccurred(hasReset);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_SetStatusFramePeriod(void* handle, int frame, uint8_t periodMs, int timeoutMs)
{
    ConvertToWrapper(handle)->SetStatusFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetStatusFramePeriod(void* handle, int frame, int* periodMs, int timeoutMs)
{
    ConvertToWrapper(handle)->GetStatusFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

} // extern "C"
