#include "ctre/phoenix/cci/MotController_CCI.h"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "CtreSimMocks/CtreMotControllerWrapper.h"
#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size)        \
    auto* wrapper = ConvertToWrapper(handle);  \
    uint8_t buffer[size]; /* NOLINT */         \
    std::memset(&buffer[0], 0, size);          \
    wrapper->Receive(paramName, buffer, size); \
    uint32_t buffer_pos = 0;

namespace
{
SnobotSim::CtreMotControllerWrapper* ConvertToWrapper(void* param)
{
    return reinterpret_cast<SnobotSim::CtreMotControllerWrapper*>(param);
}
} // namespace

extern "C" {

void* c_MotController_Create1(int baseArbId)
{
    auto* output = new SnobotSim::CtreMotControllerWrapper(baseArbId);
    return output;
}

void* c_MotController_Create2(int deviceID, const char* model)
{
    auto* output = new SnobotSim::CtreMotControllerWrapper(deviceID);
    return output;
}

void c_MotController_DestroyAll(void)
{
}

ctre::phoenix::ErrorCode c_MotController_Destroy(void* handle)
{
    auto* wrapper = ConvertToWrapper(handle);
    delete wrapper;
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetDeviceNumber(void* handle, int* deviceNumber)
{
    auto* wrapper = ConvertToWrapper(handle);
    *deviceNumber = wrapper->mDeviceId;
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetDescription(void* handle, char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    RECEIVE_HELPER("GetDescription", sizeof(*toFill) + sizeof(toFillByteSz) + sizeof(*numBytesFilled));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, &toFillByteSz, buffer_pos);
    PoplateReceiveResults(buffer, numBytesFilled, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetBaseID(void* handle, int* baseArbId)
{
    RECEIVE_HELPER("GetBaseID", sizeof(*baseArbId));
    PoplateReceiveResults(buffer, baseArbId, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetDemand(void* handle, int mode, int demand0, int demand1)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetDemand", mode, demand0, demand1);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Set_4(void* handle, int mode, double demand0, double demand1, int demand1Type)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("Set_4", mode, demand0, demand1, demand1Type);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_SetNeutralMode(void* handle, int neutralMode)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetNeutralMode", neutralMode);
}

void c_MotController_SetSensorPhase(void* handle, bool PhaseSensor)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetSensorPhase", PhaseSensor);
}

void c_MotController_SetInverted(void* handle, bool invert)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetInverted", invert);
}

void c_MotController_SetInverted_2(void* handle, int invertType)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetInverted_2", invertType);
}

ctre::phoenix::ErrorCode c_MotController_ConfigFactoryDefault(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigFactoryDefault");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigOpenLoopRamp(void* handle, double secondsFromNeutralToFull, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigOpenLoopRamp", secondsFromNeutralToFull);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopRamp(void* handle, double secondsFromNeutralToFull, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigClosedLoopRamp", secondsFromNeutralToFull);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakOutputForward(void* handle, double percentOut, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigPeakOutputForward", percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakOutputReverse(void* handle, double percentOut, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigPeakOutputReverse", percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigNominalOutputForward(void* handle, double percentOut, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigNominalOutputForward", percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigNominalOutputReverse(void* handle, double percentOut, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigNominalOutputReverse", percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigNeutralDeadband(void* handle, double percentDeadband, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigNeutralDeadband", percentDeadband);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVoltageCompSaturation(void* handle, double voltage, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigVoltageCompSaturation", voltage);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVoltageMeasurementFilter(void* handle, int filterWindowSamples, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigVoltageMeasurementFilter", filterWindowSamples);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_EnableVoltageCompensation(void* handle, bool enable)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("EnableVoltageCompensation", enable);
}

ctre::phoenix::ErrorCode c_MotController_GetInverted(void* handle, bool* invert)
{
    RECEIVE_HELPER("GetInverted", sizeof(*invert));
    PoplateReceiveResults(buffer, invert, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetBusVoltage(void* handle, double* voltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*voltage));
    PoplateReceiveResults(buffer, voltage, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotorOutputPercent(void* handle, double* percentOutput)
{
    RECEIVE_HELPER("GetMotorOutputPercent", sizeof(*percentOutput));
    PoplateReceiveResults(buffer, percentOutput, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetOutputCurrent(void* handle, double* current)
{
    RECEIVE_HELPER("GetOutputCurrent", sizeof(*current));
    PoplateReceiveResults(buffer, current, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetSupplyCurrent(void* handle, double* current)
{
    RECEIVE_HELPER("GetSupplyCurrent", sizeof(*current));
    PoplateReceiveResults(buffer, current, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetStatorCurrent(void* handle, double* current)
{
    RECEIVE_HELPER("GetStatorCurrent", sizeof(*current));
    PoplateReceiveResults(buffer, current, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetTemperature(void* handle, double* temperature)
{
    RECEIVE_HELPER("GetTemperature", sizeof(*temperature));
    PoplateReceiveResults(buffer, temperature, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSelectedFeedbackSensor(void* handle, int feedbackDevice, int pidIdx, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSelectedFeedbackSensor", feedbackDevice, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSelectedFeedbackCoefficient(void* handle, double coefficient, int pidIdx, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSelectedFeedbackCoefficient", coefficient, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigRemoteFeedbackFilter(void* handle, int deviceID, int remoteSensorSource, int remoteOrdinal, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigRemoteFeedbackFilter", deviceID, remoteSensorSource, remoteOrdinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSensorTerm(void* handle, int sensorTerm, int feedbackDevice, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSensorTerm", sensorTerm, feedbackDevice);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetSelectedSensorPosition(void* handle, int* param, int pidIdx)
{
    RECEIVE_HELPER("GetSelectedSensorPosition", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetSelectedSensorVelocity(void* handle, int* param, int pidIdx)
{
    RECEIVE_HELPER("GetSelectedSensorVelocity", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetSelectedSensorPosition(void* handle, int sensorPos, int pidIdx, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetSelectedSensorPosition", sensorPos, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetControlFramePeriod(void* handle, int frame, int periodMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetControlFramePeriod", frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetStatusFramePeriod(void* handle, int frame, uint8_t periodMs, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetStatusFramePeriod", frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetStatusFramePeriod(void* handle, int frame, int* periodMs, int timeoutMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs) + sizeof(timeoutMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVelocityMeasurementPeriod(void* handle, int period, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigVelocityMeasurementPeriod", period);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVelocityMeasurementWindow(void* handle, int windowSize, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigVelocityMeasurementWindow", windowSize);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigForwardLimitSwitchSource(void* handle, int type, int normalOpenOrClose, int deviceID, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigForwardLimitSwitchSource", type, normalOpenOrClose, deviceID);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigReverseLimitSwitchSource(void* handle, int type, int normalOpenOrClose, int deviceID, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigReverseLimitSwitchSource", type, normalOpenOrClose, deviceID);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_OverrideLimitSwitchesEnable(void* handle, bool enable)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("OverrideLimitSwitchesEnable", enable);
}

ctre::phoenix::ErrorCode c_MotController_ConfigForwardSoftLimitThreshold(void* handle, int forwardSensorLimit, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigForwardSoftLimitThreshold", forwardSensorLimit);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigReverseSoftLimitThreshold(void* handle, int reverseSensorLimit, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigReverseSoftLimitThreshold", reverseSensorLimit);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigForwardSoftLimitEnable(void* handle, bool enable, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigForwardSoftLimitEnable", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigReverseSoftLimitEnable(void* handle, bool enable, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigReverseSoftLimitEnable", enable);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_OverrideSoftLimitsEnable(void* handle, bool enable)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("OverrideSoftLimitsEnable", enable);
}

ctre::phoenix::ErrorCode c_MotController_Config_kP(void* handle, int slotIdx, double value, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("Config_kP", slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_kI(void* handle, int slotIdx, double value, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("Config_kI", slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_kD(void* handle, int slotIdx, double value, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("Config_kD", slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_kF(void* handle, int slotIdx, double value, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("Config_kF", slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_IntegralZone(void* handle, int slotIdx, double izone, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("Config_IntegralZone", slotIdx, izone);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigAllowableClosedloopError(void* handle, int slotIdx, int allowableClosedLoopError, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigAllowableClosedloopError", slotIdx, allowableClosedLoopError);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMaxIntegralAccumulator(void* handle, int slotIdx, double iaccum, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigMaxIntegralAccumulator", slotIdx, iaccum);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopPeakOutput(void* handle, int slotIdx, double percentOut, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigClosedLoopPeakOutput", slotIdx, percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopPeriod(void* handle, int slotIdx, int loopTimeMs, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigClosedLoopPeriod", slotIdx, loopTimeMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetIntegralAccumulator(void* handle, double iaccum, int pidIdx, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetIntegralAccumulator", iaccum, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetClosedLoopError(void* handle, int* closedLoopError, int pidIdx)
{
    RECEIVE_HELPER("GetClosedLoopError", sizeof(*closedLoopError) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, closedLoopError, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetIntegralAccumulator(void* handle, double* iaccum, int pidIdx)
{
    RECEIVE_HELPER("GetIntegralAccumulator", sizeof(*iaccum) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, iaccum, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetErrorDerivative(void* handle, double* derror, int pidIdx)
{
    RECEIVE_HELPER("GetErrorDerivative", sizeof(*derror) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, derror, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SelectProfileSlot(void* handle, int slotIdx, int pidIdx)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SelectProfileSlot", slotIdx, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryPosition(void* handle, int* param)
{
    RECEIVE_HELPER("GetActiveTrajectoryPosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryVelocity(void* handle, int* param)
{
    RECEIVE_HELPER("GetActiveTrajectoryVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryHeading(void* handle, double* param)
{
    RECEIVE_HELPER("GetActiveTrajectoryHeading", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryPosition_3(void* handle, int* param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryPosition_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryVelocity_3(void* handle, int* param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryVelocity_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryArbFeedFwd_3(void* handle, double* param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryArbFeedFwd_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryAll(void* handle, int* vel, int* pos, double* heading)
{
    RECEIVE_HELPER("GetActiveTrajectoryAll", sizeof(*vel) + sizeof(*pos) + sizeof(*heading));
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, heading, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryAll_5(void* handle, int* vel, int* pos, double* arbFeedFwd, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryAll_5", sizeof(*vel) + sizeof(*pos) + sizeof(*arbFeedFwd) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, arbFeedFwd, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionCruiseVelocity(void* handle, int sensorUnitsPer100ms, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigMotionCruiseVelocity", sensorUnitsPer100ms);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionAcceleration(void* handle, int sensorUnitsPer100msPerSec, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigMotionAcceleration", sensorUnitsPer100msPerSec);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionSCurveStrength(void* handle, int curveStrength, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigMotionSCurveStrength", curveStrength);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ClearMotionProfileTrajectories(void* handle)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ClearMotionProfileTrajectories");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotionProfileTopLevelBufferCount(void* handle, int* value)
{
    RECEIVE_HELPER("GetMotionProfileTopLevelBufferCount", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory(void* handle, double position, double velocity, double headingDeg, int profileSlotSelect, bool isLastPoint, bool zeroPos)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("PushMotionProfileTrajectory", position, velocity, headingDeg, profileSlotSelect, isLastPoint, zeroPos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory_2(void* handle, double position, double velocity, double headingDeg, int profileSlotSelect0, int profileSlotSelect1, bool isLastPoint, bool zeroPos, int durationMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    auto* wrapper = ConvertToWrapper(handle);
    //    wrapper->Send("PushMotionProfileTrajectory_2", position, velocity, headingDeg, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos, durationMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory_3(void* handle, double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos0, uint32_t timeDur, bool useAuxPID)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("PushMotionProfileTrajectory_3", position, velocity, arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos0, timeDur, useAuxPID);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_StartMotionProfile(void* handle, void* streamHandle, uint32_t minBufferedPts, ctre::phoenix::motorcontrol::ControlMode controlMode)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("StartMotionProfile", sizeof(*streamHandle) + sizeof(minBufferedPts) + sizeof(controlMode));
    //    PoplateReceiveResults(buffer, streamHandle, buffer_pos);
    //    PoplateReceiveResults(buffer, &minBufferedPts, buffer_pos);
    //    PoplateReceiveResults(buffer, &controlMode, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsMotionProfileFinished(void* handle, bool* value)
{
    RECEIVE_HELPER("IsMotionProfileFinished", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsMotionProfileTopLevelBufferFull(void* handle, bool* value)
{
    RECEIVE_HELPER("IsMotionProfileTopLevelBufferFull", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ProcessMotionProfileBuffer(void* handle)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ProcessMotionProfileBuffer");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotionProfileStatus(void* handle, size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt, bool* hasUnderrun, bool* isUnderrun, bool* activePointValid, bool* isLast, int* profileSlotSelect, int* outputEnable)
{
    RECEIVE_HELPER("GetMotionProfileStatus", sizeof(*topBufferRem) + sizeof(*topBufferCnt) + sizeof(*btmBufferCnt) + sizeof(*hasUnderrun) + sizeof(*isUnderrun) + sizeof(*activePointValid) + sizeof(*isLast) + sizeof(*profileSlotSelect) + sizeof(*outputEnable));
    PoplateReceiveResults(buffer, topBufferRem, buffer_pos);
    PoplateReceiveResults(buffer, topBufferCnt, buffer_pos);
    PoplateReceiveResults(buffer, btmBufferCnt, buffer_pos);
    PoplateReceiveResults(buffer, hasUnderrun, buffer_pos);
    PoplateReceiveResults(buffer, isUnderrun, buffer_pos);
    PoplateReceiveResults(buffer, activePointValid, buffer_pos);
    PoplateReceiveResults(buffer, isLast, buffer_pos);
    PoplateReceiveResults(buffer, profileSlotSelect, buffer_pos);
    PoplateReceiveResults(buffer, outputEnable, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotionProfileStatus_2(void* handle, size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt, bool* hasUnderrun, bool* isUnderrun, bool* activePointValid, bool* isLast, int* profileSlotSelect, int* outputEnable, int* timeDurMs, int* profileSlotSelect1)
{
    RECEIVE_HELPER("GetMotionProfileStatus_2", sizeof(*topBufferRem) + sizeof(*topBufferCnt) + sizeof(*btmBufferCnt) + sizeof(*hasUnderrun) + sizeof(*isUnderrun) + sizeof(*activePointValid) + sizeof(*isLast) + sizeof(*profileSlotSelect) + sizeof(*outputEnable) + sizeof(*timeDurMs) + sizeof(*profileSlotSelect1));
    PoplateReceiveResults(buffer, topBufferRem, buffer_pos);
    PoplateReceiveResults(buffer, topBufferCnt, buffer_pos);
    PoplateReceiveResults(buffer, btmBufferCnt, buffer_pos);
    PoplateReceiveResults(buffer, hasUnderrun, buffer_pos);
    PoplateReceiveResults(buffer, isUnderrun, buffer_pos);
    PoplateReceiveResults(buffer, activePointValid, buffer_pos);
    PoplateReceiveResults(buffer, isLast, buffer_pos);
    PoplateReceiveResults(buffer, profileSlotSelect, buffer_pos);
    PoplateReceiveResults(buffer, outputEnable, buffer_pos);
    PoplateReceiveResults(buffer, timeDurMs, buffer_pos);
    PoplateReceiveResults(buffer, profileSlotSelect1, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ClearMotionProfileHasUnderrun(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ClearMotionProfileHasUnderrun");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ChangeMotionControlFramePeriod(void* handle, int periodMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ChangeMotionControlFramePeriod", periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionProfileTrajectoryPeriod(void* handle, int durationMs, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigMotionProfileTrajectoryPeriod", durationMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionProfileTrajectoryInterpolationEnable(void* handle, bool enable, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigMotionProfileTrajectoryInterpolationEnable", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigFeedbackNotContinuous(void* handle, bool feedbackNotContinuous, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigFeedbackNotContinuous", feedbackNotContinuous);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(void* handle, bool remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigRemoteSensorClosedLoopDisableNeutralOnLOS", remoteSensorClosedLoopDisableNeutralOnLOS);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnLimitF(void* handle, bool clearPositionOnLimitF, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigClearPositionOnLimitF", clearPositionOnLimitF);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnLimitR(void* handle, bool clearPositionOnLimitR, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigClearPositionOnLimitR", clearPositionOnLimitR);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnQuadIdx(void* handle, bool clearPositionOnQuadIdx, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigClearPositionOnQuadIdx", clearPositionOnQuadIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigLimitSwitchDisableNeutralOnLOS(void* handle, bool limitSwitchDisableNeutralOnLOS, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigLimitSwitchDisableNeutralOnLOS", limitSwitchDisableNeutralOnLOS);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSoftLimitDisableNeutralOnLOS(void* handle, bool softLimitDisableNeutralOnLOS, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSoftLimitDisableNeutralOnLOS", softLimitDisableNeutralOnLOS);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPulseWidthPeriod_EdgesPerRot(void* handle, int pulseWidthPeriod_EdgesPerRot, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigPulseWidthPeriod_EdgesPerRot", pulseWidthPeriod_EdgesPerRot);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPulseWidthPeriod_FilterWindowSz(void* handle, int pulseWidthPeriod_FilterWindowSz, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigPulseWidthPeriod_FilterWindowSz", pulseWidthPeriod_FilterWindowSz);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetLastError(void* handle)
{
    int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)lastError;
}

ctre::phoenix::ErrorCode c_MotController_GetFirmwareVersion(void* handle, int* version)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*version));
    PoplateReceiveResults(buffer, version, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_HasResetOccurred(void* handle, bool* output)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*output));
    PoplateReceiveResults(buffer, output, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSetCustomParam(void* handle, int newValue, int paramIndex, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSetCustomParam", newValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetCustomParam(void* handle, int* readValue, int paramIndex, int timoutMs)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex) + sizeof(timoutMs));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);
    PoplateReceiveResults(buffer, &timoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSetParameter(void* handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSetParameter", param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetParameter(void* handle, int param, double* value, int ordinal, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal) + sizeof(timeoutMs));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetParameter_6(void* handle, int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal) + sizeof(timeoutMs));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakCurrentLimit(void* handle, int amps, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigPeakCurrentLimit", amps);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakCurrentDuration(void* handle, int milliseconds, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigPeakCurrentDuration", milliseconds);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigContinuousCurrentLimit(void* handle, int amps, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigContinuousCurrentLimit", amps);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_EnableCurrentLimit(void* handle, bool enable)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("EnableCurrentLimit", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetLastError(void* handle, int error)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetLastError", error);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogIn(void* handle, int* param)
{
    RECEIVE_HELPER("GetAnalogIn", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetAnalogPosition(void* handle, int newPosition, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetAnalogPosition", newPosition);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogInRaw(void* handle, int* param)
{
    RECEIVE_HELPER("GetAnalogInRaw", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogInVel(void* handle, int* param)
{
    RECEIVE_HELPER("GetAnalogInVel", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadraturePosition(void* handle, int* param)
{
    RECEIVE_HELPER("GetQuadraturePosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetQuadraturePosition(void* handle, int newPosition, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetQuadraturePosition", newPosition);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadratureVelocity(void* handle, int* param)
{
    RECEIVE_HELPER("GetQuadratureVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthPosition(void* handle, int* param)
{
    RECEIVE_HELPER("GetPulseWidthPosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetPulseWidthPosition(void* handle, int newPosition, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetPulseWidthPosition", newPosition);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthVelocity(void* handle, int* param)
{
    RECEIVE_HELPER("GetPulseWidthVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthRiseToFallUs(void* handle, int* param)
{
    RECEIVE_HELPER("GetPulseWidthRiseToFallUs", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthRiseToRiseUs(void* handle, int* param)
{
    RECEIVE_HELPER("GetPulseWidthRiseToRiseUs", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadA(void* handle, int* param)
{
    RECEIVE_HELPER("GetPinStateQuadA", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadB(void* handle, int* param)
{
    RECEIVE_HELPER("GetPinStateQuadB", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadIdx(void* handle, int* param)
{
    RECEIVE_HELPER("GetPinStateQuadIdx", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsFwdLimitSwitchClosed(void* handle, int* param)
{
    RECEIVE_HELPER("IsFwdLimitSwitchClosed", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsRevLimitSwitchClosed(void* handle, int* param)
{
    RECEIVE_HELPER("IsRevLimitSwitchClosed", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetFaults(void* handle, int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetStickyFaults(void* handle, int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ClearStickyFaults(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ClearStickyFaults");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SelectDemandType(void* handle, bool enable)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SelectDemandType", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetMPEOutput(void* handle, int MpeOutput)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetMPEOutput", MpeOutput);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_EnableHeadingHold(void* handle, bool enable)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("EnableHeadingHold", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogInAll(void* handle, int* withOv, int* raw, int* vel)
{
    RECEIVE_HELPER("GetAnalogInAll", sizeof(*withOv) + sizeof(*raw) + sizeof(*vel));
    PoplateReceiveResults(buffer, withOv, buffer_pos);
    PoplateReceiveResults(buffer, raw, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadratureSensor(void* handle, int* pos, int* vel)
{
    RECEIVE_HELPER("GetQuadratureSensor", sizeof(*pos) + sizeof(*vel));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthAll(void* handle, int* pos, int* vel, int* riseToRiseUs, int* riseToFallUs)
{
    RECEIVE_HELPER("GetPulseWidthAll", sizeof(*pos) + sizeof(*vel) + sizeof(*riseToRiseUs) + sizeof(*riseToFallUs));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, riseToRiseUs, buffer_pos);
    PoplateReceiveResults(buffer, riseToFallUs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadPinStates(void* handle, int* quadA, int* quadB, int* quadIdx)
{
    RECEIVE_HELPER("GetQuadPinStates", sizeof(*quadA) + sizeof(*quadB) + sizeof(*quadIdx));
    PoplateReceiveResults(buffer, quadA, buffer_pos);
    PoplateReceiveResults(buffer, quadB, buffer_pos);
    PoplateReceiveResults(buffer, quadIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetLimitSwitchState(void* handle, int* isFwdClosed, int* isRevClosed)
{
    RECEIVE_HELPER("GetLimitSwitchState", sizeof(*isFwdClosed) + sizeof(*isRevClosed));
    PoplateReceiveResults(buffer, isFwdClosed, buffer_pos);
    PoplateReceiveResults(buffer, isRevClosed, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetClosedLoopTarget(void* handle, int* value, int pidIdx)
{
    RECEIVE_HELPER("GetClosedLoopTarget", sizeof(*value) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotorCommutation(void* handle, ctre::phoenix::motorcontrol::MotorCommutation motorCommutation, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigMotorCommutation", motorCommutation);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetMotorCommutation(void* handle, ctre::phoenix::motorcontrol::MotorCommutation* motorCommutation, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetMotorCommutation", sizeof(*motorCommutation) + sizeof(timeoutMs));
    PoplateReceiveResults(buffer, motorCommutation, buffer_pos);
    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSupplyCurrentLimit(void* handle, const double* params, int paramCnt, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("ConfigSupplyCurrentLimit", sizeof(*params) + sizeof(paramCnt) + sizeof(timeoutMs));
    //    PoplateReceiveResults(buffer, params, buffer_pos);
    //    PoplateReceiveResults(buffer, &paramCnt, buffer_pos);
    //    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigStatorCurrentLimit(void* handle, const double* params, int paramCnt, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("ConfigStatorCurrentLimit", sizeof(*params) + sizeof(paramCnt) + sizeof(timeoutMs));
    //    PoplateReceiveResults(buffer, params, buffer_pos);
    //    PoplateReceiveResults(buffer, &paramCnt, buffer_pos);
    //    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSupplyCurrentLimitEnable(void* handle, bool enable, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigSupplyCurrentLimitEnable", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigStatorCurrentLimitEnable(void* handle, bool enable, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigStatorCurrentLimitEnable", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetSupplyCurrentLimit(void* handle, double* toFill, int* fillCnt, int fillCapacity, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("ConfigGetSupplyCurrentLimit", sizeof(*toFill) + sizeof(*fillCnt) + sizeof(fillCapacity) + sizeof(timeoutMs));
    //    PoplateReceiveResults(buffer, toFill, buffer_pos);
    //    PoplateReceiveResults(buffer, fillCnt, buffer_pos);
    //    PoplateReceiveResults(buffer, &fillCapacity, buffer_pos);
    //    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetStatorCurrentLimit(void* handle, double* toFill, int* fillCnt, int fillCapacity, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("ConfigGetStatorCurrentLimit", sizeof(*toFill) + sizeof(*fillCnt) + sizeof(fillCapacity) + sizeof(timeoutMs));
    //    PoplateReceiveResults(buffer, toFill, buffer_pos);
    //    PoplateReceiveResults(buffer, fillCnt, buffer_pos);
    //    PoplateReceiveResults(buffer, &fillCapacity, buffer_pos);
    //    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetIntegratedSensorPosition(void* handle, double newpos, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetIntegratedSensorPosition", newpos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetIntegratedSensorPositionToAbsolute(void* handle, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("SetIntegratedSensorPositionToAbsolute");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetIntegratedSensor(void* handle, double* pos, double* absPos, double* vel)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("GetIntegratedSensor", sizeof(*pos) + sizeof(*absPos) + sizeof(*vel));
    //    PoplateReceiveResults(buffer, pos, buffer_pos);
    //    PoplateReceiveResults(buffer, absPos, buffer_pos);
    //    PoplateReceiveResults(buffer, vel, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigIntegratedSensorAbsoluteRange(void* handle, ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigIntegratedSensorAbsoluteRange", absoluteSensorRange);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigIntegratedSensorOffset(void* handle, double offsetDegrees, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigIntegratedSensorOffset", offsetDegrees);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigIntegratedSensorInitializationStrategy(void* handle, ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy, int timeoutMs)
{
    auto* wrapper = ConvertToWrapper(handle);
    wrapper->Send("ConfigIntegratedSensorInitializationStrategy", initializationStrategy);
    return (ctre::phoenix::ErrorCode)0;
}

} // extern "C"
