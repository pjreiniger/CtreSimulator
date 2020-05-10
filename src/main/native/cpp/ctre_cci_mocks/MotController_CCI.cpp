#include "ctre/phoenix/cci/MotController_CCI.h"

#include <cstring>
#include <vector>

#include "CtreSimMocks/CtreMotControllerWrapper.h"
#include "CtreSimUtils/MockHooks.h"

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
    ConvertToWrapper(handle)->GetDescription(toFill, toFillByteSz, numBytesFilled);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetBaseID(void* handle, int* baseArbId)
{
    ConvertToWrapper(handle)->GetBaseID(baseArbId);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetDemand(void* handle, int mode, int demand0, int demand1)
{
    ConvertToWrapper(handle)->SetDemand(mode, demand0, demand1);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Set_4(void* handle, int mode, double demand0, double demand1, int demand1Type)
{
    ConvertToWrapper(handle)->Set_4(mode, demand0, demand1, demand1Type);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_SetNeutralMode(void* handle, int neutralMode)
{
    ConvertToWrapper(handle)->SetNeutralMode(neutralMode);
}

void c_MotController_SetSensorPhase(void* handle, bool PhaseSensor)
{
    ConvertToWrapper(handle)->SetSensorPhase(PhaseSensor);
}

void c_MotController_SetInverted(void* handle, bool invert)
{
    ConvertToWrapper(handle)->SetInverted(invert);
}

void c_MotController_SetInverted_2(void* handle, int invertType)
{
    ConvertToWrapper(handle)->SetInverted_2(invertType);
}

ctre::phoenix::ErrorCode c_MotController_ConfigFactoryDefault(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigFactoryDefault();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigOpenLoopRamp(void* handle, double secondsFromNeutralToFull, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigOpenLoopRamp(secondsFromNeutralToFull);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopRamp(void* handle, double secondsFromNeutralToFull, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigClosedLoopRamp(secondsFromNeutralToFull);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakOutputForward(void* handle, double percentOut, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigPeakOutputForward(percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakOutputReverse(void* handle, double percentOut, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigPeakOutputReverse(percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigNominalOutputForward(void* handle, double percentOut, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigNominalOutputForward(percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigNominalOutputReverse(void* handle, double percentOut, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigNominalOutputReverse(percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigNeutralDeadband(void* handle, double percentDeadband, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigNeutralDeadband(percentDeadband);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVoltageCompSaturation(void* handle, double voltage, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigVoltageCompSaturation(voltage);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVoltageMeasurementFilter(void* handle, int filterWindowSamples, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigVoltageMeasurementFilter(filterWindowSamples);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_EnableVoltageCompensation(void* handle, bool enable)
{
    ConvertToWrapper(handle)->EnableVoltageCompensation(enable);
}

ctre::phoenix::ErrorCode c_MotController_GetInverted(void* handle, bool* invert)
{
    ConvertToWrapper(handle)->GetInverted(invert);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetBusVoltage(void* handle, double* voltage)
{
    ConvertToWrapper(handle)->GetBusVoltage(voltage);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotorOutputPercent(void* handle, double* percentOutput)
{
    ConvertToWrapper(handle)->GetMotorOutputPercent(percentOutput);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetOutputCurrent(void* handle, double* current)
{
    ConvertToWrapper(handle)->GetOutputCurrent(current);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetSupplyCurrent(void* handle, double* current)
{
    ConvertToWrapper(handle)->GetSupplyCurrent(current);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetStatorCurrent(void* handle, double* current)
{
    ConvertToWrapper(handle)->GetStatorCurrent(current);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetTemperature(void* handle, double* temperature)
{
    ConvertToWrapper(handle)->GetTemperature(temperature);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSelectedFeedbackSensor(void* handle, int feedbackDevice, int pidIdx, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSelectedFeedbackSensor(feedbackDevice, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSelectedFeedbackCoefficient(void* handle, double coefficient, int pidIdx, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSelectedFeedbackCoefficient(coefficient, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigRemoteFeedbackFilter(void* handle, int deviceID, int remoteSensorSource, int remoteOrdinal, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigRemoteFeedbackFilter(deviceID, remoteSensorSource, remoteOrdinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSensorTerm(void* handle, int sensorTerm, int feedbackDevice, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSensorTerm(sensorTerm, feedbackDevice);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetSelectedSensorPosition(void* handle, int* param, int pidIdx)
{
    ConvertToWrapper(handle)->GetSelectedSensorPosition(param, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetSelectedSensorVelocity(void* handle, int* param, int pidIdx)
{
    ConvertToWrapper(handle)->GetSelectedSensorVelocity(param, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetSelectedSensorPosition(void* handle, int sensorPos, int pidIdx, int timeoutMs)
{
    ConvertToWrapper(handle)->SetSelectedSensorPosition(sensorPos, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetControlFramePeriod(void* handle, int frame, int periodMs)
{
    ConvertToWrapper(handle)->SetControlFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetStatusFramePeriod(void* handle, int frame, uint8_t periodMs, int timeoutMs)
{
    ConvertToWrapper(handle)->SetStatusFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetStatusFramePeriod(void* handle, int frame, int* periodMs, int timeoutMs)
{
    ConvertToWrapper(handle)->GetStatusFramePeriod(frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVelocityMeasurementPeriod(void* handle, int period, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigVelocityMeasurementPeriod(period);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVelocityMeasurementWindow(void* handle, int windowSize, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigVelocityMeasurementWindow(windowSize);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigForwardLimitSwitchSource(void* handle, int type, int normalOpenOrClose, int deviceID, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigForwardLimitSwitchSource(type, normalOpenOrClose, deviceID);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigReverseLimitSwitchSource(void* handle, int type, int normalOpenOrClose, int deviceID, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigReverseLimitSwitchSource(type, normalOpenOrClose, deviceID);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_OverrideLimitSwitchesEnable(void* handle, bool enable)
{
    ConvertToWrapper(handle)->OverrideLimitSwitchesEnable(enable);
}

ctre::phoenix::ErrorCode c_MotController_ConfigForwardSoftLimitThreshold(void* handle, int forwardSensorLimit, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigForwardSoftLimitThreshold(forwardSensorLimit);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigReverseSoftLimitThreshold(void* handle, int reverseSensorLimit, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigReverseSoftLimitThreshold(reverseSensorLimit);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigForwardSoftLimitEnable(void* handle, bool enable, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigForwardSoftLimitEnable(enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigReverseSoftLimitEnable(void* handle, bool enable, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigReverseSoftLimitEnable(enable);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_OverrideSoftLimitsEnable(void* handle, bool enable)
{
    ConvertToWrapper(handle)->OverrideSoftLimitsEnable(enable);
}

ctre::phoenix::ErrorCode c_MotController_Config_kP(void* handle, int slotIdx, double value, int timeoutMs)
{
    ConvertToWrapper(handle)->Config_kP(slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_kI(void* handle, int slotIdx, double value, int timeoutMs)
{
    ConvertToWrapper(handle)->Config_kI(slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_kD(void* handle, int slotIdx, double value, int timeoutMs)
{
    ConvertToWrapper(handle)->Config_kD(slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_kF(void* handle, int slotIdx, double value, int timeoutMs)
{
    ConvertToWrapper(handle)->Config_kF(slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_IntegralZone(void* handle, int slotIdx, double izone, int timeoutMs)
{
    ConvertToWrapper(handle)->Config_IntegralZone(slotIdx, izone);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigAllowableClosedloopError(void* handle, int slotIdx, int allowableClosedLoopError, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigAllowableClosedloopError(slotIdx, allowableClosedLoopError);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMaxIntegralAccumulator(void* handle, int slotIdx, double iaccum, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigMaxIntegralAccumulator(slotIdx, iaccum);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopPeakOutput(void* handle, int slotIdx, double percentOut, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigClosedLoopPeakOutput(slotIdx, percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopPeriod(void* handle, int slotIdx, int loopTimeMs, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigClosedLoopPeriod(slotIdx, loopTimeMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetIntegralAccumulator(void* handle, double iaccum, int pidIdx, int timeoutMs)
{
    ConvertToWrapper(handle)->SetIntegralAccumulator(iaccum, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetClosedLoopError(void* handle, int* closedLoopError, int pidIdx)
{
    ConvertToWrapper(handle)->GetClosedLoopError(closedLoopError, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetIntegralAccumulator(void* handle, double* iaccum, int pidIdx)
{
    ConvertToWrapper(handle)->GetIntegralAccumulator(iaccum, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetErrorDerivative(void* handle, double* derror, int pidIdx)
{
    ConvertToWrapper(handle)->GetErrorDerivative(derror, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SelectProfileSlot(void* handle, int slotIdx, int pidIdx)
{
    ConvertToWrapper(handle)->SelectProfileSlot(slotIdx, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryPosition(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetActiveTrajectoryPosition(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryVelocity(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetActiveTrajectoryVelocity(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryHeading(void* handle, double* param)
{
    ConvertToWrapper(handle)->GetActiveTrajectoryHeading(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryPosition_3(void* handle, int* param, int pidIdx)
{
    ConvertToWrapper(handle)->GetActiveTrajectoryPosition_3(param, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryVelocity_3(void* handle, int* param, int pidIdx)
{
    ConvertToWrapper(handle)->GetActiveTrajectoryVelocity_3(param, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryArbFeedFwd_3(void* handle, double* param, int pidIdx)
{
    ConvertToWrapper(handle)->GetActiveTrajectoryArbFeedFwd_3(param, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryAll(void* handle, int* vel, int* pos, double* heading)
{
    ConvertToWrapper(handle)->GetActiveTrajectoryAll(vel, pos, heading);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryAll_5(void* handle, int* vel, int* pos, double* arbFeedFwd, int pidIdx)
{
    ConvertToWrapper(handle)->GetActiveTrajectoryAll_5(vel, pos, arbFeedFwd, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionCruiseVelocity(void* handle, int sensorUnitsPer100ms, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigMotionCruiseVelocity(sensorUnitsPer100ms);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionAcceleration(void* handle, int sensorUnitsPer100msPerSec, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigMotionAcceleration(sensorUnitsPer100msPerSec);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionSCurveStrength(void* handle, int curveStrength, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigMotionSCurveStrength(curveStrength);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ClearMotionProfileTrajectories(void* handle)
{
    ConvertToWrapper(handle)->ClearMotionProfileTrajectories();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotionProfileTopLevelBufferCount(void* handle, int* value)
{
    ConvertToWrapper(handle)->GetMotionProfileTopLevelBufferCount(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory(void* handle, double position, double velocity, double headingDeg, int profileSlotSelect, bool isLastPoint, bool zeroPos)
{
    ConvertToWrapper(handle)->PushMotionProfileTrajectory(position, velocity, headingDeg, profileSlotSelect, isLastPoint, zeroPos);
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
    ConvertToWrapper(handle)->PushMotionProfileTrajectory_3(position, velocity, arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos0, timeDur, useAuxPID);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_StartMotionProfile(void* handle, void* streamHandle, uint32_t minBufferedPts, ctre::phoenix::motorcontrol::ControlMode controlMode)
{
    ConvertToWrapper(handle)->StartMotionProfile(streamHandle, minBufferedPts, controlMode);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsMotionProfileFinished(void* handle, bool* value)
{
    ConvertToWrapper(handle)->IsMotionProfileFinished(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsMotionProfileTopLevelBufferFull(void* handle, bool* value)
{
    ConvertToWrapper(handle)->IsMotionProfileTopLevelBufferFull(value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ProcessMotionProfileBuffer(void* handle)
{
    ConvertToWrapper(handle)->ProcessMotionProfileBuffer();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotionProfileStatus(void* handle, size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt, bool* hasUnderrun, bool* isUnderrun, bool* activePointValid, bool* isLast, int* profileSlotSelect, int* outputEnable)
{
    ConvertToWrapper(handle)->GetMotionProfileStatus(topBufferRem, topBufferCnt, btmBufferCnt, hasUnderrun, isUnderrun, activePointValid, isLast, profileSlotSelect, outputEnable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotionProfileStatus_2(void* handle, size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt, bool* hasUnderrun, bool* isUnderrun, bool* activePointValid, bool* isLast, int* profileSlotSelect, int* outputEnable, int* timeDurMs, int* profileSlotSelect1)
{
    ConvertToWrapper(handle)->GetMotionProfileStatus_2(topBufferRem, topBufferCnt, btmBufferCnt, hasUnderrun, isUnderrun, activePointValid, isLast, profileSlotSelect, outputEnable, timeDurMs, profileSlotSelect1);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ClearMotionProfileHasUnderrun(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->ClearMotionProfileHasUnderrun();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ChangeMotionControlFramePeriod(void* handle, int periodMs)
{
    ConvertToWrapper(handle)->ChangeMotionControlFramePeriod(periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionProfileTrajectoryPeriod(void* handle, int durationMs, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigMotionProfileTrajectoryPeriod(durationMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionProfileTrajectoryInterpolationEnable(void* handle, bool enable, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigMotionProfileTrajectoryInterpolationEnable(enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigFeedbackNotContinuous(void* handle, bool feedbackNotContinuous, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigFeedbackNotContinuous(feedbackNotContinuous);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(void* handle, bool remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(remoteSensorClosedLoopDisableNeutralOnLOS);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnLimitF(void* handle, bool clearPositionOnLimitF, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigClearPositionOnLimitF(clearPositionOnLimitF);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnLimitR(void* handle, bool clearPositionOnLimitR, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigClearPositionOnLimitR(clearPositionOnLimitR);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnQuadIdx(void* handle, bool clearPositionOnQuadIdx, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigClearPositionOnQuadIdx(clearPositionOnQuadIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigLimitSwitchDisableNeutralOnLOS(void* handle, bool limitSwitchDisableNeutralOnLOS, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigLimitSwitchDisableNeutralOnLOS(limitSwitchDisableNeutralOnLOS);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSoftLimitDisableNeutralOnLOS(void* handle, bool softLimitDisableNeutralOnLOS, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSoftLimitDisableNeutralOnLOS(softLimitDisableNeutralOnLOS);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPulseWidthPeriod_EdgesPerRot(void* handle, int pulseWidthPeriod_EdgesPerRot, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigPulseWidthPeriod_EdgesPerRot(pulseWidthPeriod_EdgesPerRot);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPulseWidthPeriod_FilterWindowSz(void* handle, int pulseWidthPeriod_FilterWindowSz, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigPulseWidthPeriod_FilterWindowSz(pulseWidthPeriod_FilterWindowSz);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetLastError(void* handle)
{
    return ConvertToWrapper(handle)->GetLastError();
}

ctre::phoenix::ErrorCode c_MotController_GetFirmwareVersion(void* handle, int* version)
{
    ConvertToWrapper(handle)->GetFirmwareVersion(version);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_HasResetOccurred(void* handle, bool* output)
{
    ConvertToWrapper(handle)->HasResetOccurred(output);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSetCustomParam(void* handle, int newValue, int paramIndex, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSetCustomParam(newValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetCustomParam(void* handle, int* readValue, int paramIndex, int timoutMs)
{
    ConvertToWrapper(handle)->ConfigGetCustomParam(readValue, paramIndex, timoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSetParameter(void* handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigSetParameter(param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetParameter(void* handle, int param, double* value, int ordinal, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetParameter(param, value, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetParameter_6(void* handle, int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetParameter_6(param, valueToSend, valueRecieved, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakCurrentLimit(void* handle, int amps, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigPeakCurrentLimit(amps);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakCurrentDuration(void* handle, int milliseconds, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigPeakCurrentDuration(milliseconds);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigContinuousCurrentLimit(void* handle, int amps, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigContinuousCurrentLimit(amps);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_EnableCurrentLimit(void* handle, bool enable)
{
    ConvertToWrapper(handle)->EnableCurrentLimit(enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetLastError(void* handle, int error)
{
    ConvertToWrapper(handle)->SetLastError(error);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogIn(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetAnalogIn(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetAnalogPosition(void* handle, int newPosition, int timeoutMs)
{
    ConvertToWrapper(handle)->SetAnalogPosition(newPosition);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogInRaw(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetAnalogInRaw(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogInVel(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetAnalogInVel(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadraturePosition(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetQuadraturePosition(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetQuadraturePosition(void* handle, int newPosition, int timeoutMs)
{
    ConvertToWrapper(handle)->SetQuadraturePosition(newPosition);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadratureVelocity(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetQuadratureVelocity(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthPosition(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetPulseWidthPosition(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetPulseWidthPosition(void* handle, int newPosition, int timeoutMs)
{
    ConvertToWrapper(handle)->SetPulseWidthPosition(newPosition);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthVelocity(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetPulseWidthVelocity(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthRiseToFallUs(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetPulseWidthRiseToFallUs(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthRiseToRiseUs(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetPulseWidthRiseToRiseUs(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadA(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetPinStateQuadA(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadB(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetPinStateQuadB(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadIdx(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetPinStateQuadIdx(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsFwdLimitSwitchClosed(void* handle, int* param)
{
    ConvertToWrapper(handle)->IsFwdLimitSwitchClosed(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsRevLimitSwitchClosed(void* handle, int* param)
{
    ConvertToWrapper(handle)->IsRevLimitSwitchClosed(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetFaults(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetFaults(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetStickyFaults(void* handle, int* param)
{
    ConvertToWrapper(handle)->GetStickyFaults(param);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ClearStickyFaults(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->ClearStickyFaults();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SelectDemandType(void* handle, bool enable)
{
    ConvertToWrapper(handle)->SelectDemandType(enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetMPEOutput(void* handle, int MpeOutput)
{
    ConvertToWrapper(handle)->SetMPEOutput(MpeOutput);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_EnableHeadingHold(void* handle, bool enable)
{
    ConvertToWrapper(handle)->EnableHeadingHold(enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogInAll(void* handle, int* withOv, int* raw, int* vel)
{
    ConvertToWrapper(handle)->GetAnalogInAll(withOv, raw, vel);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadratureSensor(void* handle, int* pos, int* vel)
{
    ConvertToWrapper(handle)->GetQuadratureSensor(pos, vel);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthAll(void* handle, int* pos, int* vel, int* riseToRiseUs, int* riseToFallUs)
{
    ConvertToWrapper(handle)->GetPulseWidthAll(pos, vel, riseToRiseUs, riseToFallUs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadPinStates(void* handle, int* quadA, int* quadB, int* quadIdx)
{
    ConvertToWrapper(handle)->GetQuadPinStates(quadA, quadB, quadIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetLimitSwitchState(void* handle, int* isFwdClosed, int* isRevClosed)
{
    ConvertToWrapper(handle)->GetLimitSwitchState(isFwdClosed, isRevClosed);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetClosedLoopTarget(void* handle, int* value, int pidIdx)
{
    ConvertToWrapper(handle)->GetClosedLoopTarget(value, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotorCommutation(void* handle, ctre::phoenix::motorcontrol::MotorCommutation motorCommutation, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigMotorCommutation(motorCommutation);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetMotorCommutation(void* handle, ctre::phoenix::motorcontrol::MotorCommutation* motorCommutation, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetMotorCommutation(motorCommutation);
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
    ConvertToWrapper(handle)->ConfigSupplyCurrentLimitEnable(enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigStatorCurrentLimitEnable(void* handle, bool enable, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigStatorCurrentLimitEnable(enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetSupplyCurrentLimit(void* handle, double* toFill, int* fillCnt, int fillCapacity, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetSupplyCurrentLimit(toFill, fillCnt, fillCapacity);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetStatorCurrentLimit(void* handle, double* toFill, int* fillCnt, int fillCapacity, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigGetStatorCurrentLimit(toFill, fillCnt, fillCapacity);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetIntegratedSensorPosition(void* handle, double newpos, int timeoutMs)
{
    ConvertToWrapper(handle)->SetIntegratedSensorPosition(newpos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetIntegratedSensorPositionToAbsolute(void* handle, int timeoutMs)
{
    ConvertToWrapper(handle)->SetIntegratedSensorPositionToAbsolute();
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetIntegratedSensor(void* handle, double* pos, double* absPos, double* vel)
{
    ConvertToWrapper(handle)->GetIntegratedSensor(pos, absPos, vel);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigIntegratedSensorAbsoluteRange(void* handle, ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigIntegratedSensorAbsoluteRange(absoluteSensorRange);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigIntegratedSensorOffset(void* handle, double offsetDegrees, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigIntegratedSensorOffset(offsetDegrees);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigIntegratedSensorInitializationStrategy(void* handle, ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy, int timeoutMs)
{
    ConvertToWrapper(handle)->ConfigIntegratedSensorInitializationStrategy(initializationStrategy);
    return (ctre::phoenix::ErrorCode)0;
}

} // extern "C"
