#include "ctre/phoenix/CCI/CANifier_CCI.h"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "CtreSimMocks/CtreCanifierWrapper.h"
#include "CtreSimMocks/MockHooks.h"

#define RECEIVE_HELPER(paramName, size)                                         \
    SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle); \
    uint8_t buffer[size]; /* NOLINT */                                          \
    std::memset(&buffer[0], 0, size);                                           \
    wrapper->Receive(paramName, buffer, size);                                  \
    uint32_t buffer_pos = 0;

SnobotSim::CtreCanifierWrapper* ConvertToCanifierWrapper(void* param)
{
    long handle = *static_cast<long*>(param); //NOLINT

    return reinterpret_cast<SnobotSim::CtreCanifierWrapper*>(handle);
}

extern "C"{

void *c_CANifier_Create1(int deviceNumber)
{
	SnobotSim::CtreCanifierWrapper* output = new SnobotSim::CtreCanifierWrapper(deviceNumber);
    return output;
}

ctre::phoenix::ErrorCode c_CANifier_GetDescription(void *handle, char * toFill, int toFillByteSz, int * numBytesFilled)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetLEDOutput(void *handle,  uint32_t  dutyCycle,  uint32_t  ledChannel)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("SetLEDOutput", dutyCycle, ledChannel);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetGeneralOutputs(void *handle,  uint32_t  outputsBits,  uint32_t  isOutputBits)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("SetGeneralOutputs", outputsBits, isOutputBits);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetGeneralOutput(void *handle,  uint32_t  outputPin,  bool  outputValue,  bool  outputEnable)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("SetGeneralOutput", outputPin, outputValue, outputEnable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetPWMOutput(void *handle,  uint32_t  pwmChannel,  uint32_t  dutyCycle)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("SetPWMOutput", pwmChannel, dutyCycle);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_EnablePWMOutput(void *handle, uint32_t pwmChannel, bool bEnable)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("EnablePWMOutput", pwmChannel, bEnable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetGeneralInputs(void *handle, bool allPins[], uint32_t capacity)
{
    RECEIVE_HELPER("GetGeneralInputs", sizeof(bool) * 11);
    for(unsigned int i = 0; i < capacity; ++i)
    {
    	PoplateReceiveResults(buffer, &allPins[i], buffer_pos);
    }
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetGeneralInput(void *handle, uint32_t inputPin, bool * measuredInput)
{
    RECEIVE_HELPER("GetGeneralInput", sizeof(inputPin) + sizeof(*measuredInput));
    PoplateReceiveResults(buffer, &inputPin, buffer_pos);
    PoplateReceiveResults(buffer, measuredInput, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetPWMInput(void *handle,  uint32_t  pwmChannel,  double dutyCycleAndPeriod [2])
{
    RECEIVE_HELPER("GetPWMInput", sizeof(pwmChannel) + sizeof(double) * 2);
    PoplateReceiveResults(buffer, &pwmChannel, buffer_pos);
    PoplateReceiveResults(buffer, &dutyCycleAndPeriod[0], buffer_pos);
    PoplateReceiveResults(buffer, &dutyCycleAndPeriod[1], buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetLastError(void *handle)
{
	int error = 0;
    RECEIVE_HELPER("GetLastError", sizeof(error));
    PoplateReceiveResults(buffer, &error, buffer_pos);
    return (ctre::phoenix::ErrorCode)error;
}

ctre::phoenix::ErrorCode c_CANifier_GetBusVoltage(void *handle, double * batteryVoltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*batteryVoltage));
    PoplateReceiveResults(buffer, batteryVoltage, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}
ctre::phoenix::ErrorCode c_CANifier_GetQuadraturePosition(void *handle, int * pos)
{
    RECEIVE_HELPER("GetQuadraturePosition", sizeof(*pos));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetQuadraturePosition(void *handle, int pos, int timeoutMs)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("SetQuadraturePosition", pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetQuadratureVelocity(void *handle, int * vel)
{
    RECEIVE_HELPER("GetQuadratureVelocity", sizeof(*vel));
    PoplateReceiveResults(buffer, vel, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetQuadratureSensor(void *handle, int * pos, int * vel)
{
    RECEIVE_HELPER("GetQuadratureSensor", sizeof(*pos) + sizeof(*vel));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigVelocityMeasurementPeriod(void *handle, int period, int timeoutMs)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("ConfigVelocityMeasurementPeriod", period);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigVelocityMeasurementWindow(void *handle, int window, int timeoutMs)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("ConfigVelocityMeasurementWindow", window);
    return (ctre::phoenix::ErrorCode)0;
}


void c_CANifier_SetLastError(void *handle, int error)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("SetLastError", error);
}

ctre::phoenix::ErrorCode c_CANifier_ConfigSetParameter(void *handle, int param, double value, int subValue, int ordinal, int timeoutMs)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("ConfigSetParameter", param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigGetParameter(void *handle, int param, double *value, int ordinal, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigSetCustomParam(void *handle, int newValue, int paramIndex, int timeoutMs)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("ConfigSetCustomParam", newValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ConfigGetCustomParam(void *handle, int *readValue, int paramIndex, int timoutMs)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetFaults(void *handle, int * param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetStickyFaults(void *handle, int * param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_ClearStickyFaults(void *handle, int timeoutMs)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("ClearStickyFaults");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetFirmwareVersion(void *handle, int *firmwareVers)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*firmwareVers));
    PoplateReceiveResults(buffer, firmwareVers, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_HasResetOccurred(void *handle, bool * hasReset)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*hasReset));
    PoplateReceiveResults(buffer, hasReset, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetStatusFramePeriod(void *handle, int frame, int periodMs, int timeoutMs)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("SetStatusFramePeriod", frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_GetStatusFramePeriod(void *handle, int frame, int *periodMs, int timeoutMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) +  sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_CANifier_SetControlFramePeriod(void *handle, int frame,	int periodMs)
{
	SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle);
    wrapper->Send("SetControlFramePeriod", frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

}  // extern "C"
