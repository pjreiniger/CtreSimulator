
#include "CtreSimMocks/CtreCANCoderWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size) \
    uint8_t buffer[size]; /* NOLINT */  \
    std::memset(&buffer[0], 0, size);   \
    Receive(paramName, buffer, size);   \
    uint32_t buffer_pos = 0;

SnobotSim::CtreCANCoderWrapper::CtreCANCoderWrapper(int deviceId)
{
    Send("Create");
}

void SnobotSim::CtreCANCoderWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}

void SnobotSim::CtreCANCoderWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}

void SnobotSim::CtreCANCoderWrapper::GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    RECEIVE_HELPER("GetDescription", 1);
    buffer_pos += 1; // Removes compiler warning
}

ctre::phoenix::ErrorCode SnobotSim::CtreCANCoderWrapper::GetLastError()
{
    int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)lastError;
}

void SnobotSim::CtreCANCoderWrapper::GetLastUnitString(char* toFill, int toFillByteSz, int* numBytesFilled)
{
    RECEIVE_HELPER("GetLastUnitString", sizeof(*toFill) + sizeof(toFillByteSz) + sizeof(*numBytesFilled));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, &toFillByteSz, buffer_pos);
    PoplateReceiveResults(buffer, numBytesFilled, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::GetLastTimestamp(double* timestamp)
{
    RECEIVE_HELPER("GetLastTimestamp", sizeof(*timestamp));
    PoplateReceiveResults(buffer, timestamp, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::GetBusVoltage(double* batteryVoltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*batteryVoltage));
    PoplateReceiveResults(buffer, batteryVoltage, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::GetMagnetFieldStrength(ctre::phoenix::sensors::MagnetFieldStrength* magnetFieldStrength)
{
    RECEIVE_HELPER("GetMagnetFieldStrength", sizeof(*magnetFieldStrength));
    PoplateReceiveResults(buffer, magnetFieldStrength, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::GetPosition(double* pos)
{
    RECEIVE_HELPER("GetPosition", sizeof(*pos));
    PoplateReceiveResults(buffer, pos, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::SetPosition(double pos, int timeoutMs)
{
    Send("SetPosition", pos);
}

void SnobotSim::CtreCANCoderWrapper::SetPositionToAbsolute(int timeoutMs)
{
    Send("SetPositionToAbsolute");
}

void SnobotSim::CtreCANCoderWrapper::ConfigSensorDirection(int bDirection, int timeoutMs)
{
    Send("ConfigSensorDirection", bDirection);
}

void SnobotSim::CtreCANCoderWrapper::GetVelocity(double* vel)
{
    RECEIVE_HELPER("GetVelocity", sizeof(*vel));
    PoplateReceiveResults(buffer, vel, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::GetAbsolutePosition(double* pos)
{
    RECEIVE_HELPER("GetAbsolutePosition", sizeof(*pos));
    PoplateReceiveResults(buffer, pos, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::ConfigVelocityMeasurementPeriod(int period, int timeoutMs)
{
    Send("ConfigVelocityMeasurementPeriod", period);
}

void SnobotSim::CtreCANCoderWrapper::ConfigVelocityMeasurementWindow(int window, int timeoutMs)
{
    Send("ConfigVelocityMeasurementWindow", window);
}

void SnobotSim::CtreCANCoderWrapper::ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange, int timeoutMs)
{
    Send("ConfigAbsoluteSensorRange", absoluteSensorRange);
}

void SnobotSim::CtreCANCoderWrapper::ConfigMagnetOffset(double offsetDegrees, int timeoutMs)
{
    Send("ConfigMagnetOffset", offsetDegrees);
}

void SnobotSim::CtreCANCoderWrapper::ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy, int timeoutMs)
{
    Send("ConfigSensorInitializationStrategy", initializationStrategy);
}

void SnobotSim::CtreCANCoderWrapper::ConfigFeedbackCoefficient(double sensorCoefficient, const char* unitString, ctre::phoenix::sensors::SensorTimeBase sensortimeBase, int timeoutMs)
{
    //    RECEIVE_HELPER("ConfigFeedbackCoefficient", sizeof(sensorCoefficient) + sizeof(*unitString) + sizeof(sensortimeBase));
    //    PoplateReceiveResults(buffer, &sensorCoefficient, buffer_pos);
    //    PoplateReceiveResults(buffer, unitString, buffer_pos);
    //    PoplateReceiveResults(buffer, &sensortimeBase, buffer_pos);
    LOG_UNSUPPORTED_CAN_FUNC("");
}

void SnobotSim::CtreCANCoderWrapper::ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    Send("ConfigSetParameter", param, value, subValue, ordinal);
}

void SnobotSim::CtreCANCoderWrapper::ConfigGetParameter(int param, double* value, int ordinal, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::ConfigSetCustomParam(int newValue, int paramIndex, int timeoutMs)
{
    Send("ConfigSetCustomParam", newValue, paramIndex);
}

void SnobotSim::CtreCANCoderWrapper::ConfigGetCustomParam(int* readValue, int paramIndex, int timoutMs)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex) + sizeof(timoutMs));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);
    PoplateReceiveResults(buffer, &timoutMs, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::ConfigFactoryDefault(int timeoutMs)
{
    Send("ConfigFactoryDefault");
}

void SnobotSim::CtreCANCoderWrapper::GetFaults(int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::GetStickyFaults(int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::ClearStickyFaults(int timeoutMs)
{
    Send("ClearStickyFaults");
}

void SnobotSim::CtreCANCoderWrapper::GetFirmwareVersion(int* firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*firmwareVers));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::HasResetOccurred(bool* hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*hasReset));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);
}

void SnobotSim::CtreCANCoderWrapper::SetStatusFramePeriod(int frame, uint8_t periodMs, int timeoutMs)
{
    Send("SetStatusFramePeriod", frame, periodMs);
}

void SnobotSim::CtreCANCoderWrapper::GetStatusFramePeriod(int frame, int* periodMs, int timeoutMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
}
