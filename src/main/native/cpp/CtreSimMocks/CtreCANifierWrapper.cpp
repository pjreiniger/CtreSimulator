

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
        mDeviceId(aDeviceId & 0x3F),
        m_simDevice(std::string("CtreCANifierWrapper " + std::to_string(aDeviceId)).c_str(), aDeviceId)
{


    m_BusVoltage_batteryVoltage = m_simDevice.CreateDouble("BusVoltage_batteryVoltage", false, 0);
    m_ConfigClearPositionOnLimitF_clearPositionOnLimitF = m_simDevice.CreateDouble("ConfigClearPositionOnLimitF_clearPositionOnLimitF", false, 0);
    m_ConfigClearPositionOnLimitR_clearPositionOnLimitR = m_simDevice.CreateDouble("ConfigClearPositionOnLimitR_clearPositionOnLimitR", false, 0);
    m_ConfigClearPositionOnQuadIdx_clearPositionOnQuadIdx = m_simDevice.CreateDouble("ConfigClearPositionOnQuadIdx_clearPositionOnQuadIdx", false, 0);
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
    m_ConfigSetCustomParam_newValue = m_simDevice.CreateDouble("ConfigSetCustomParam_newValue", false, 0);
    m_ConfigSetCustomParam_paramIndex = m_simDevice.CreateDouble("ConfigSetCustomParam_paramIndex", false, 0);
    m_ConfigSetParameter_ordinal = m_simDevice.CreateDouble("ConfigSetParameter_ordinal", false, 0);
    m_ConfigSetParameter_param = m_simDevice.CreateDouble("ConfigSetParameter_param", false, 0);
    m_ConfigSetParameter_subValue = m_simDevice.CreateDouble("ConfigSetParameter_subValue", false, 0);
    m_ConfigSetParameter_value = m_simDevice.CreateDouble("ConfigSetParameter_value", false, 0);
    m_ConfigVelocityMeasurementPeriod_period = m_simDevice.CreateDouble("ConfigVelocityMeasurementPeriod_period", false, 0);
    m_ConfigVelocityMeasurementWindow_window = m_simDevice.CreateDouble("ConfigVelocityMeasurementWindow_window", false, 0);
    m_ControlFramePeriod_frame = m_simDevice.CreateDouble("ControlFramePeriod_frame", false, 0);
    m_ControlFramePeriod_periodMs = m_simDevice.CreateDouble("ControlFramePeriod_periodMs", false, 0);
    m_Create1_deviceNumber = m_simDevice.CreateDouble("Create1_deviceNumber", false, 0);
    m_Description_numBytesFilled = m_simDevice.CreateDouble("Description_numBytesFilled", false, 0);
    m_Description_toFill = m_simDevice.CreateDouble("Description_toFill", false, 0);
    m_Description_toFillByteSz = m_simDevice.CreateDouble("Description_toFillByteSz", false, 0);
    m_EnablePWMOutput_bEnable = m_simDevice.CreateDouble("EnablePWMOutput_bEnable", false, 0);
    m_EnablePWMOutput_pwmChannel = m_simDevice.CreateDouble("EnablePWMOutput_pwmChannel", false, 0);
    m_Faults_param = m_simDevice.CreateDouble("Faults_param", false, 0);
    m_FirmwareVersion_firmwareVers = m_simDevice.CreateDouble("FirmwareVersion_firmwareVers", false, 0);
    m_GeneralInput_inputPin = m_simDevice.CreateDouble("GeneralInput_inputPin", false, 0);
    m_GeneralInput_measuredInput = m_simDevice.CreateDouble("GeneralInput_measuredInput", false, 0);
    m_GeneralInputs_allPins = m_simDevice.CreateDouble("GeneralInputs_allPins", false, 0);
    m_GeneralInputs_capacity = m_simDevice.CreateDouble("GeneralInputs_capacity", false, 0);
    m_GeneralOutput_outputEnable = m_simDevice.CreateDouble("GeneralOutput_outputEnable", false, 0);
    m_GeneralOutput_outputPin = m_simDevice.CreateDouble("GeneralOutput_outputPin", false, 0);
    m_GeneralOutput_outputValue = m_simDevice.CreateDouble("GeneralOutput_outputValue", false, 0);
    m_GeneralOutputs_isOutputBits = m_simDevice.CreateDouble("GeneralOutputs_isOutputBits", false, 0);
    m_GeneralOutputs_outputsBits = m_simDevice.CreateDouble("GeneralOutputs_outputsBits", false, 0);
    m_HasResetOccurred_hasReset = m_simDevice.CreateDouble("HasResetOccurred_hasReset", false, 0);
    m_LEDOutput_dutyCycle = m_simDevice.CreateDouble("LEDOutput_dutyCycle", false, 0);
    m_LEDOutput_ledChannel = m_simDevice.CreateDouble("LEDOutput_ledChannel", false, 0);
    m_LastError_error = m_simDevice.CreateDouble("LastError_error", false, 0);
    m_PWMInput_dutyCycleAndPeriod_0 = m_simDevice.CreateDouble("PWMInput_dutyCycleAndPeriod_0", false, 0);
    m_PWMInput_dutyCycleAndPeriod_1 = m_simDevice.CreateDouble("PWMInput_dutyCycleAndPeriod_1", false, 0);
    m_PWMInput_pwmChannel = m_simDevice.CreateDouble("PWMInput_pwmChannel", false, 0);
    m_PWMOutput_dutyCycle = m_simDevice.CreateDouble("PWMOutput_dutyCycle", false, 0);
    m_PWMOutput_pwmChannel = m_simDevice.CreateDouble("PWMOutput_pwmChannel", false, 0);
    m_QuadraturePosition_pos = m_simDevice.CreateDouble("QuadraturePosition_pos", false, 0);
    m_QuadratureSensor_pos = m_simDevice.CreateDouble("QuadratureSensor_pos", false, 0);
    m_QuadratureSensor_vel = m_simDevice.CreateDouble("QuadratureSensor_vel", false, 0);
    m_QuadratureVelocity_vel = m_simDevice.CreateDouble("QuadratureVelocity_vel", false, 0);
    m_StatusFramePeriod_frame = m_simDevice.CreateDouble("StatusFramePeriod_frame", false, 0);
    m_StatusFramePeriod_periodMs = m_simDevice.CreateDouble("StatusFramePeriod_periodMs", false, 0);
    m_StickyFaults_param = m_simDevice.CreateDouble("StickyFaults_param", false, 0);




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
    RECEIVE_HELPER("GetDescription", sizeof(*toFill) + sizeof(toFillByteSz) + sizeof(*numBytesFilled));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, &toFillByteSz, buffer_pos);
    PoplateReceiveResults(buffer, numBytesFilled, buffer_pos);

    *toFillByteSz = m_Description_toFillByteSz.Get();
    *toFill = m_Description_toFill.Get();
    *numBytesFilled = m_Description_numBytesFilled.Get();
}

void SnobotSim::CtreCANifierWrapper::SetLEDOutput(uint32_t dutyCycle, uint32_t ledChannel)
{
    m_LEDOutput_ledChannel.Set(ledChannel);
    m_LEDOutput_dutyCycle.Set(dutyCycle);

    Send("SetLEDOutput", dutyCycle, ledChannel);
}

void SnobotSim::CtreCANifierWrapper::SetGeneralOutputs(uint32_t outputsBits, uint32_t isOutputBits)
{
    m_GeneralOutputs_outputsBits.Set(outputsBits);
    m_GeneralOutputs_isOutputBits.Set(isOutputBits);

    Send("SetGeneralOutputs", outputsBits, isOutputBits);
}

void SnobotSim::CtreCANifierWrapper::SetGeneralOutput(uint32_t outputPin, bool outputValue, bool outputEnable)
{
    m_GeneralOutput_outputValue.Set(outputValue);
    m_GeneralOutput_outputPin.Set(outputPin);
    m_GeneralOutput_outputEnable.Set(outputEnable);

    Send("SetGeneralOutput", outputPin, outputValue, outputEnable);
}

void SnobotSim::CtreCANifierWrapper::SetPWMOutput(uint32_t pwmChannel, uint32_t dutyCycle)
{
    m_PWMOutput_pwmChannel.Set(pwmChannel);
    m_PWMOutput_dutyCycle.Set(dutyCycle);

    Send("SetPWMOutput", pwmChannel, dutyCycle);
}

void SnobotSim::CtreCANifierWrapper::EnablePWMOutput(uint32_t pwmChannel, bool bEnable)
{
    m_EnablePWMOutput_pwmChannel.Set(pwmChannel);
    m_EnablePWMOutput_bEnable.Set(bEnable);

    Send("EnablePWMOutput", pwmChannel, bEnable);
}

void SnobotSim::CtreCANifierWrapper::GetGeneralInputs(bool allPins, uint32_t capacity)
{
    m_GeneralInputs_capacity.Set(capacity);
    m_GeneralInputs_allPins.Set(allPins);

    Send("GetGeneralInputs", allPins, capacity);
}

void SnobotSim::CtreCANifierWrapper::GetGeneralInput(uint32_t inputPin, bool* measuredInput)
{
    RECEIVE_HELPER("GetGeneralInput", sizeof(inputPin) + sizeof(*measuredInput));
    PoplateReceiveResults(buffer, &inputPin, buffer_pos);
    PoplateReceiveResults(buffer, measuredInput, buffer_pos);

    *measuredInput = m_GeneralInput_measuredInput.Get();
    *inputPin = m_GeneralInput_inputPin.Get();
}

void SnobotSim::CtreCANifierWrapper::GetPWMInput(uint32_t pwmChannel, double dutyCycleAndPeriod[2])
{
    m_PWMInput_pwmChannel.Set(pwmChannel);
    m_PWMInput_dutyCycleAndPeriod_1.Set(dutyCycleAndPeriod_1);
    m_PWMInput_dutyCycleAndPeriod_0.Set(dutyCycleAndPeriod_0);

    Send("GetPWMInput", pwmChannel, dutyCycleAndPeriod[0], dutyCycleAndPeriod[1]);
}

void SnobotSim::CtreCANifierWrapper::GetBusVoltage(double* batteryVoltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*batteryVoltage));
    PoplateReceiveResults(buffer, batteryVoltage, buffer_pos);

    *batteryVoltage = m_BusVoltage_batteryVoltage.Get();
}

void SnobotSim::CtreCANifierWrapper::GetQuadraturePosition(int* pos)
{
    RECEIVE_HELPER("GetQuadraturePosition", sizeof(*pos));
    PoplateReceiveResults(buffer, pos, buffer_pos);

    *pos = m_QuadraturePosition_pos.Get();
}

void SnobotSim::CtreCANifierWrapper::SetQuadraturePosition(int pos)
{
    m_QuadraturePosition_pos.Set(pos);

    Send("SetQuadraturePosition", pos);
}

void SnobotSim::CtreCANifierWrapper::GetQuadratureVelocity(int* vel)
{
    RECEIVE_HELPER("GetQuadratureVelocity", sizeof(*vel));
    PoplateReceiveResults(buffer, vel, buffer_pos);

    *vel = m_QuadratureVelocity_vel.Get();
}

void SnobotSim::CtreCANifierWrapper::GetQuadratureSensor(int* pos, int* vel)
{
    RECEIVE_HELPER("GetQuadratureSensor", sizeof(*pos) + sizeof(*vel));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);

    *vel = m_QuadratureSensor_vel.Get();
    *pos = m_QuadratureSensor_pos.Get();
}

void SnobotSim::CtreCANifierWrapper::ConfigVelocityMeasurementPeriod(int period)
{
    m_ConfigVelocityMeasurementPeriod_period.Set(period);

    Send("ConfigVelocityMeasurementPeriod", period);
}

void SnobotSim::CtreCANifierWrapper::ConfigVelocityMeasurementWindow(int window)
{
    m_ConfigVelocityMeasurementWindow_window.Set(window);

    Send("ConfigVelocityMeasurementWindow", window);
}

void SnobotSim::CtreCANifierWrapper::ConfigClearPositionOnLimitF(bool clearPositionOnLimitF)
{
    m_ConfigClearPositionOnLimitF_clearPositionOnLimitF.Set(clearPositionOnLimitF);

    Send("ConfigClearPositionOnLimitF", clearPositionOnLimitF);
}

void SnobotSim::CtreCANifierWrapper::ConfigClearPositionOnLimitR(bool clearPositionOnLimitR)
{
    m_ConfigClearPositionOnLimitR_clearPositionOnLimitR.Set(clearPositionOnLimitR);

    Send("ConfigClearPositionOnLimitR", clearPositionOnLimitR);
}

void SnobotSim::CtreCANifierWrapper::ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx)
{
    m_ConfigClearPositionOnQuadIdx_clearPositionOnQuadIdx.Set(clearPositionOnQuadIdx);

    Send("ConfigClearPositionOnQuadIdx", clearPositionOnQuadIdx);
}

void SnobotSim::CtreCANifierWrapper::SetLastError(int error)
{
    m_LastError_error.Set(error);

    Send("SetLastError", error);
}

void SnobotSim::CtreCANifierWrapper::ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal)
{
    m_ConfigSetParameter_value.Set(value);
    m_ConfigSetParameter_subValue.Set(subValue);
    m_ConfigSetParameter_param.Set(param);
    m_ConfigSetParameter_ordinal.Set(ordinal);

    Send("ConfigSetParameter", param, value, subValue, ordinal);
}

void SnobotSim::CtreCANifierWrapper::ConfigGetParameter(int param, double* value, int ordinal)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);

    *value = m_ConfigGetParameter_value.Get();
    *param = m_ConfigGetParameter_param.Get();
    *ordinal = m_ConfigGetParameter_ordinal.Get();
}

void SnobotSim::CtreCANifierWrapper::ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal)
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

void SnobotSim::CtreCANifierWrapper::ConfigSetCustomParam(int newValue, int paramIndex)
{
    m_ConfigSetCustomParam_paramIndex.Set(paramIndex);
    m_ConfigSetCustomParam_newValue.Set(newValue);

    Send("ConfigSetCustomParam", newValue, paramIndex);
}

void SnobotSim::CtreCANifierWrapper::ConfigGetCustomParam(int* readValue, int paramIndex)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);

    *readValue = m_ConfigGetCustomParam_readValue.Get();
    *paramIndex = m_ConfigGetCustomParam_paramIndex.Get();
}

void SnobotSim::CtreCANifierWrapper::ConfigFactoryDefault()
{

    Send("ConfigFactoryDefault");
}

void SnobotSim::CtreCANifierWrapper::GetFaults(int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_Faults_param.Get();
}

void SnobotSim::CtreCANifierWrapper::GetStickyFaults(int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_StickyFaults_param.Get();
}

void SnobotSim::CtreCANifierWrapper::ClearStickyFaults()
{

    Send("ClearStickyFaults");
}

void SnobotSim::CtreCANifierWrapper::GetFirmwareVersion(int* firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*firmwareVers));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);

    *firmwareVers = m_FirmwareVersion_firmwareVers.Get();
}

void SnobotSim::CtreCANifierWrapper::HasResetOccurred(bool* hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*hasReset));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);

    *hasReset = m_HasResetOccurred_hasReset.Get();
}

void SnobotSim::CtreCANifierWrapper::SetStatusFramePeriod(int frame, uint8_t periodMs)
{
    m_StatusFramePeriod_periodMs.Set(periodMs);
    m_StatusFramePeriod_frame.Set(frame);

    Send("SetStatusFramePeriod", frame, periodMs);
}

void SnobotSim::CtreCANifierWrapper::GetStatusFramePeriod(int frame, int* periodMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);

    *periodMs = m_StatusFramePeriod_periodMs.Get();
    *frame = m_StatusFramePeriod_frame.Get();
}

void SnobotSim::CtreCANifierWrapper::SetControlFramePeriod(int frame, int periodMs)
{
    m_ControlFramePeriod_periodMs.Set(periodMs);
    m_ControlFramePeriod_frame.Set(frame);

    Send("SetControlFramePeriod", frame, periodMs);
}

