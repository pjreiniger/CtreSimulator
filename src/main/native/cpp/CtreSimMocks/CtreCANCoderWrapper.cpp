

#include "CtreSimMocks/CtreCANCoderWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size) \
    uint8_t buffer[size]; /* NOLINT */  \
    std::memset(&buffer[0], 0, size);   \
    Receive(paramName, buffer, size);   \
    uint32_t buffer_pos = 0;

std::vector<SnobotSim::CTRE_CallbackFunc> gCANCoderCallbacks;

void SnobotSim::SetCANCoderCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gCANCoderCallbacks.clear();
    gCANCoderCallbacks.push_back(callback);
}

SnobotSim::CtreCANCoderWrapper::CtreCANCoderWrapper(int aDeviceId) :
        mDeviceId(aDeviceId & 0x3F),
        m_simDevice(std::string("CtreCANCoderWrapper " + std::to_string(aDeviceId)).c_str(), aDeviceId)
{


    m_AbsolutePosition_pos = m_simDevice.CreateDouble("AbsolutePosition_pos", false, 0);
    m_BusVoltage_batteryVoltage = m_simDevice.CreateDouble("BusVoltage_batteryVoltage", false, 0);
    m_ConfigAbsoluteSensorRange_absoluteSensorRange = m_simDevice.CreateDouble("ConfigAbsoluteSensorRange_absoluteSensorRange", false, 0);
    m_ConfigFeedbackCoefficient_sensorCoefficient = m_simDevice.CreateDouble("ConfigFeedbackCoefficient_sensorCoefficient", false, 0);
    m_ConfigFeedbackCoefficient_sensortimeBase = m_simDevice.CreateDouble("ConfigFeedbackCoefficient_sensortimeBase", false, 0);
    m_ConfigFeedbackCoefficient_unitString = m_simDevice.CreateDouble("ConfigFeedbackCoefficient_unitString", false, 0);
    m_ConfigGetCustomParam_paramIndex = m_simDevice.CreateDouble("ConfigGetCustomParam_paramIndex", false, 0);
    m_ConfigGetCustomParam_readValue = m_simDevice.CreateDouble("ConfigGetCustomParam_readValue", false, 0);
    m_ConfigGetParameter_6_ordinal = m_simDevice.CreateDouble("ConfigGetParameter_6_ordinal", false, 0);
    m_ConfigGetParameter_6_param = m_simDevice.CreateDouble("ConfigGetParameter_6_param", false, 0);
    m_ConfigGetParameter_6_subValue = m_simDevice.CreateDouble("ConfigGetParameter_6_subValue", false, 0);
    m_ConfigGetParameter_6_valueRecieved = m_simDevice.CreateDouble("ConfigGetParameter_6_valueRecieved", false, 0);
    m_ConfigGetParameter_6_valueToSend = m_simDevice.CreateDouble("ConfigGetParameter_6_valueToSend", false, 0);
    m_ConfigGetParameter_ordinal = m_simDevice.CreateDouble("ConfigGetParameter_ordinal", false, 0);
    m_ConfigGetParameter_param = m_simDevice.CreateDouble("ConfigGetParameter_param", false, 0);
    m_ConfigGetParameter_value = m_simDevice.CreateDouble("ConfigGetParameter_value", false, 0);
    m_ConfigMagnetOffset_offsetDegrees = m_simDevice.CreateDouble("ConfigMagnetOffset_offsetDegrees", false, 0);
    m_ConfigSensorDirection_bDirection = m_simDevice.CreateDouble("ConfigSensorDirection_bDirection", false, 0);
    m_ConfigSensorInitializationStrategy_initializationStrategy = m_simDevice.CreateDouble("ConfigSensorInitializationStrategy_initializationStrategy", false, 0);
    m_ConfigSetCustomParam_newValue = m_simDevice.CreateDouble("ConfigSetCustomParam_newValue", false, 0);
    m_ConfigSetCustomParam_paramIndex = m_simDevice.CreateDouble("ConfigSetCustomParam_paramIndex", false, 0);
    m_ConfigSetParameter_ordinal = m_simDevice.CreateDouble("ConfigSetParameter_ordinal", false, 0);
    m_ConfigSetParameter_param = m_simDevice.CreateDouble("ConfigSetParameter_param", false, 0);
    m_ConfigSetParameter_subValue = m_simDevice.CreateDouble("ConfigSetParameter_subValue", false, 0);
    m_ConfigSetParameter_value = m_simDevice.CreateDouble("ConfigSetParameter_value", false, 0);
    m_ConfigVelocityMeasurementPeriod_period = m_simDevice.CreateDouble("ConfigVelocityMeasurementPeriod_period", false, 0);
    m_ConfigVelocityMeasurementWindow_window = m_simDevice.CreateDouble("ConfigVelocityMeasurementWindow_window", false, 0);
    m_Create1_deviceNumber = m_simDevice.CreateDouble("Create1_deviceNumber", false, 0);
    m_Description_numBytesFilled = m_simDevice.CreateDouble("Description_numBytesFilled", false, 0);
    m_Description_toFill = m_simDevice.CreateDouble("Description_toFill", false, 0);
    m_Description_toFillByteSz = m_simDevice.CreateDouble("Description_toFillByteSz", false, 0);
    m_Faults_param = m_simDevice.CreateDouble("Faults_param", false, 0);
    m_FirmwareVersion_firmwareVers = m_simDevice.CreateDouble("FirmwareVersion_firmwareVers", false, 0);
    m_HasResetOccurred_hasReset = m_simDevice.CreateDouble("HasResetOccurred_hasReset", false, 0);
    m_LastTimestamp_timestamp = m_simDevice.CreateDouble("LastTimestamp_timestamp", false, 0);
    m_LastUnitString_numBytesFilled = m_simDevice.CreateDouble("LastUnitString_numBytesFilled", false, 0);
    m_LastUnitString_toFill = m_simDevice.CreateDouble("LastUnitString_toFill", false, 0);
    m_LastUnitString_toFillByteSz = m_simDevice.CreateDouble("LastUnitString_toFillByteSz", false, 0);
    m_MagnetFieldStrength_magnetFieldStrength = m_simDevice.CreateDouble("MagnetFieldStrength_magnetFieldStrength", false, 0);
    m_Position_pos = m_simDevice.CreateDouble("Position_pos", false, 0);
    m_StatusFramePeriod_frame = m_simDevice.CreateDouble("StatusFramePeriod_frame", false, 0);
    m_StatusFramePeriod_periodMs = m_simDevice.CreateDouble("StatusFramePeriod_periodMs", false, 0);
    m_StickyFaults_param = m_simDevice.CreateDouble("StickyFaults_param", false, 0);
    m_Velocity_vel = m_simDevice.CreateDouble("Velocity_vel", false, 0);




    Send("Create");
}

void SnobotSim::CtreCANCoderWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gCANCoderCallbacks.empty())
    {
        gCANCoderCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreCANCoderWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    if (!gCANCoderCallbacks.empty())
    {
        gCANCoderCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreCANCoderWrapper::GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    RECEIVE_HELPER("GetDescription", sizeof(*toFill) + sizeof(toFillByteSz) + sizeof(*numBytesFilled));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, &toFillByteSz, buffer_pos);
    PoplateReceiveResults(buffer, numBytesFilled, buffer_pos);

    *toFillByteSz = m_Description_toFillByteSz.Get();
    *toFill = m_Description_toFill.Get();
    *numBytesFilled = m_Description_numBytesFilled.Get();
}

void SnobotSim::CtreCANCoderWrapper::GetLastUnitString(char* toFill, int toFillByteSz, int* numBytesFilled)
{
    RECEIVE_HELPER("GetLastUnitString", sizeof(*toFill) + sizeof(toFillByteSz) + sizeof(*numBytesFilled));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, &toFillByteSz, buffer_pos);
    PoplateReceiveResults(buffer, numBytesFilled, buffer_pos);

    *toFillByteSz = m_LastUnitString_toFillByteSz.Get();
    *toFill = m_LastUnitString_toFill.Get();
    *numBytesFilled = m_LastUnitString_numBytesFilled.Get();
}

void SnobotSim::CtreCANCoderWrapper::GetLastTimestamp(double* timestamp)
{
    RECEIVE_HELPER("GetLastTimestamp", sizeof(*timestamp));
    PoplateReceiveResults(buffer, timestamp, buffer_pos);

    *timestamp = m_LastTimestamp_timestamp.Get();
}

void SnobotSim::CtreCANCoderWrapper::GetBusVoltage(double* batteryVoltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*batteryVoltage));
    PoplateReceiveResults(buffer, batteryVoltage, buffer_pos);

    *batteryVoltage = m_BusVoltage_batteryVoltage.Get();
}

void SnobotSim::CtreCANCoderWrapper::GetMagnetFieldStrength(ctre::phoenix::sensors::MagnetFieldStrength* magnetFieldStrength)
{
    RECEIVE_HELPER("GetMagnetFieldStrength", sizeof(*magnetFieldStrength));
    PoplateReceiveResults(buffer, magnetFieldStrength, buffer_pos);

    *magnetFieldStrength = m_MagnetFieldStrength_magnetFieldStrength.Get();
}

void SnobotSim::CtreCANCoderWrapper::GetPosition(double* pos)
{
    RECEIVE_HELPER("GetPosition", sizeof(*pos));
    PoplateReceiveResults(buffer, pos, buffer_pos);

    *pos = m_Position_pos.Get();
}

void SnobotSim::CtreCANCoderWrapper::SetPosition(double pos)
{
    m_Position_pos.Set(pos);

    Send("SetPosition", pos);
}

void SnobotSim::CtreCANCoderWrapper::SetPositionToAbsolute()
{

    Send("SetPositionToAbsolute");
}

void SnobotSim::CtreCANCoderWrapper::ConfigSensorDirection(int bDirection)
{
    m_ConfigSensorDirection_bDirection.Set(bDirection);

    Send("ConfigSensorDirection", bDirection);
}

void SnobotSim::CtreCANCoderWrapper::GetVelocity(double* vel)
{
    RECEIVE_HELPER("GetVelocity", sizeof(*vel));
    PoplateReceiveResults(buffer, vel, buffer_pos);

    *vel = m_Velocity_vel.Get();
}

void SnobotSim::CtreCANCoderWrapper::GetAbsolutePosition(double* pos)
{
    RECEIVE_HELPER("GetAbsolutePosition", sizeof(*pos));
    PoplateReceiveResults(buffer, pos, buffer_pos);

    *pos = m_AbsolutePosition_pos.Get();
}

void SnobotSim::CtreCANCoderWrapper::ConfigVelocityMeasurementPeriod(int period)
{
    m_ConfigVelocityMeasurementPeriod_period.Set(period);

    Send("ConfigVelocityMeasurementPeriod", period);
}

void SnobotSim::CtreCANCoderWrapper::ConfigVelocityMeasurementWindow(int window)
{
    m_ConfigVelocityMeasurementWindow_window.Set(window);

    Send("ConfigVelocityMeasurementWindow", window);
}

void SnobotSim::CtreCANCoderWrapper::ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange)
{
    m_ConfigAbsoluteSensorRange_absoluteSensorRange.Set(absoluteSensorRange);

    Send("ConfigAbsoluteSensorRange", absoluteSensorRange);
}

void SnobotSim::CtreCANCoderWrapper::ConfigMagnetOffset(double offsetDegrees)
{
    m_ConfigMagnetOffset_offsetDegrees.Set(offsetDegrees);

    Send("ConfigMagnetOffset", offsetDegrees);
}

void SnobotSim::CtreCANCoderWrapper::ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy)
{
    m_ConfigSensorInitializationStrategy_initializationStrategy.Set(initializationStrategy);

    Send("ConfigSensorInitializationStrategy", initializationStrategy);
}

void SnobotSim::CtreCANCoderWrapper::ConfigFeedbackCoefficient(double sensorCoefficient, const char* unitString, ctre::phoenix::sensors::SensorTimeBase sensortimeBase)
{
    RECEIVE_HELPER("ConfigFeedbackCoefficient", sizeof(sensorCoefficient) + sizeof(*unitString) + sizeof(sensortimeBase));
    PoplateReceiveResults(buffer, &sensorCoefficient, buffer_pos);
    PoplateReceiveResults(buffer, unitString, buffer_pos);
    PoplateReceiveResults(buffer, &sensortimeBase, buffer_pos);

    *unitString = m_ConfigFeedbackCoefficient_unitString.Get();
    *sensortimeBase = m_ConfigFeedbackCoefficient_sensortimeBase.Get();
    *sensorCoefficient = m_ConfigFeedbackCoefficient_sensorCoefficient.Get();
}

void SnobotSim::CtreCANCoderWrapper::ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal)
{
    m_ConfigSetParameter_value.Set(value);
    m_ConfigSetParameter_subValue.Set(subValue);
    m_ConfigSetParameter_param.Set(param);
    m_ConfigSetParameter_ordinal.Set(ordinal);

    Send("ConfigSetParameter", param, value, subValue, ordinal);
}

void SnobotSim::CtreCANCoderWrapper::ConfigGetParameter(int param, double* value, int ordinal)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);

    *value = m_ConfigGetParameter_value.Get();
    *param = m_ConfigGetParameter_param.Get();
    *ordinal = m_ConfigGetParameter_ordinal.Get();
}

void SnobotSim::CtreCANCoderWrapper::ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);

    *valueToSend = m_ConfigGetParameter_6_valueToSend.Get();
    *valueRecieved = m_ConfigGetParameter_6_valueRecieved.Get();
    *subValue = m_ConfigGetParameter_6_subValue.Get();
    *param = m_ConfigGetParameter_6_param.Get();
    *ordinal = m_ConfigGetParameter_6_ordinal.Get();
}

void SnobotSim::CtreCANCoderWrapper::ConfigSetCustomParam(int newValue, int paramIndex)
{
    m_ConfigSetCustomParam_paramIndex.Set(paramIndex);
    m_ConfigSetCustomParam_newValue.Set(newValue);

    Send("ConfigSetCustomParam", newValue, paramIndex);
}

void SnobotSim::CtreCANCoderWrapper::ConfigGetCustomParam(int* readValue, int paramIndex)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);

    *readValue = m_ConfigGetCustomParam_readValue.Get();
    *paramIndex = m_ConfigGetCustomParam_paramIndex.Get();
}

void SnobotSim::CtreCANCoderWrapper::ConfigFactoryDefault()
{

    Send("ConfigFactoryDefault");
}

void SnobotSim::CtreCANCoderWrapper::GetFaults(int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_Faults_param.Get();
}

void SnobotSim::CtreCANCoderWrapper::GetStickyFaults(int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_StickyFaults_param.Get();
}

void SnobotSim::CtreCANCoderWrapper::ClearStickyFaults()
{

    Send("ClearStickyFaults");
}

void SnobotSim::CtreCANCoderWrapper::GetFirmwareVersion(int* firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*firmwareVers));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);

    *firmwareVers = m_FirmwareVersion_firmwareVers.Get();
}

void SnobotSim::CtreCANCoderWrapper::HasResetOccurred(bool* hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*hasReset));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);

    *hasReset = m_HasResetOccurred_hasReset.Get();
}

void SnobotSim::CtreCANCoderWrapper::SetStatusFramePeriod(int frame, uint8_t periodMs)
{
    m_StatusFramePeriod_periodMs.Set(periodMs);
    m_StatusFramePeriod_frame.Set(frame);

    Send("SetStatusFramePeriod", frame, periodMs);
}

void SnobotSim::CtreCANCoderWrapper::GetStatusFramePeriod(int frame, int* periodMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);

    *periodMs = m_StatusFramePeriod_periodMs.Get();
    *frame = m_StatusFramePeriod_frame.Get();
}

