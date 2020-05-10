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
    RECEIVE_HELPER("GetLastUnitString", sizeof(*toFill) + sizeof(toFillByteSz) + sizeof(*numBytesFilled));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, &toFillByteSz, buffer_pos);
    PoplateReceiveResults(buffer, numBytesFilled, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetLastTimestamp(void* handle, double* timestamp)
{
    RECEIVE_HELPER("GetLastTimestamp", sizeof(*timestamp));
    PoplateReceiveResults(buffer, timestamp, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetBusVoltage(void* handle, double* batteryVoltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*batteryVoltage));
    PoplateReceiveResults(buffer, batteryVoltage, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetMagnetFieldStrength(void* handle, ctre::phoenix::sensors::MagnetFieldStrength* magnetFieldStrength)
{
    RECEIVE_HELPER("GetMagnetFieldStrength", sizeof(*magnetFieldStrength));
    PoplateReceiveResults(buffer, magnetFieldStrength, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetPosition(void* handle, double* pos)
{
    RECEIVE_HELPER("GetPosition", sizeof(*pos));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_SetPosition(void* handle, double pos, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetPosition", pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_SetPositionToAbsolute(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetPositionToAbsolute");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigSensorDirection(void* handle, int bDirection, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSensorDirection", bDirection);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetVelocity(void* handle, double* vel)
{
    RECEIVE_HELPER("GetVelocity", sizeof(*vel));
    PoplateReceiveResults(buffer, vel, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetAbsolutePosition(void* handle, double* pos)
{
    RECEIVE_HELPER("GetAbsolutePosition", sizeof(*pos));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigVelocityMeasurementPeriod(void* handle, int period, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigVelocityMeasurementPeriod", period);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigVelocityMeasurementWindow(void* handle, int window, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigVelocityMeasurementWindow", window);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigAbsoluteSensorRange(void* handle, ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigAbsoluteSensorRange", absoluteSensorRange);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigMagnetOffset(void* handle, double offsetDegrees, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigMagnetOffset", offsetDegrees);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigSensorInitializationStrategy(void* handle, ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSensorInitializationStrategy", initializationStrategy);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigFeedbackCoefficient(void* handle, double sensorCoefficient, const char* unitString, ctre::phoenix::sensors::SensorTimeBase sensortimeBase, int timeoutMs)
{
    //    RECEIVE_HELPER("ConfigFeedbackCoefficient", sizeof(sensorCoefficient) + sizeof(*unitString) + sizeof(sensortimeBase));
    //    PoplateReceiveResults(buffer, &sensorCoefficient, buffer_pos);
    //    PoplateReceiveResults(buffer, unitString, buffer_pos);
    //    PoplateReceiveResults(buffer, &sensortimeBase, buffer_pos);
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigSetParameter(void* handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSetParameter", param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigGetParameter(void* handle, int param, double* value, int ordinal, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigGetParameter_6(void* handle, int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigSetCustomParam(void* handle, int newValue, int paramIndex, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSetCustomParam", newValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigGetCustomParam(void* handle, int* readValue, int paramIndex, int timoutMs)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex) + sizeof(timoutMs));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);
    PoplateReceiveResults(buffer, &timoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ConfigFactoryDefault(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigFactoryDefault");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetFaults(void* handle, int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetStickyFaults(void* handle, int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_ClearStickyFaults(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ClearStickyFaults");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetFirmwareVersion(void* handle, int* firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*firmwareVers));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_HasResetOccurred(void* handle, bool* hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*hasReset));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_SetStatusFramePeriod(void* handle, int frame, uint8_t periodMs, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetStatusFramePeriod", frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANCoder_GetStatusFramePeriod(void* handle, int frame, int* periodMs, int timeoutMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

} // extern "C"
