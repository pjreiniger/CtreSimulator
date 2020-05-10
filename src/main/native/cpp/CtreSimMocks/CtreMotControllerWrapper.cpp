
#include "CtreSimMocks/CtreMotControllerWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size) \
    uint8_t buffer[size]; /* NOLINT */  \
    std::memset(&buffer[0], 0, size);   \
    Receive(paramName, buffer, size);   \
    uint32_t buffer_pos = 0;

std::vector<SnobotSim::CTRE_CallbackFunc> gMotorControllerCallbacks;

void SnobotSim::SetMotControllerCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gMotorControllerCallbacks.clear();
    gMotorControllerCallbacks.push_back(callback);
}

SnobotSim::CtreMotControllerWrapper::CtreMotControllerWrapper(int aDeviceId) :
        mDeviceId(aDeviceId & 0x3F)
{
    Send("Create");
}

void SnobotSim::CtreMotControllerWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gMotorControllerCallbacks.empty())
    {
        gMotorControllerCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreMotControllerWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    if (!gMotorControllerCallbacks.empty())
    {
        gMotorControllerCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreMotControllerWrapper::GetDeviceNumber(int* deviceNumber)
{
    *deviceNumber = mDeviceId;
}

void SnobotSim::CtreMotControllerWrapper::GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    RECEIVE_HELPER("GetDescription", sizeof(*toFill) + sizeof(toFillByteSz) + sizeof(*numBytesFilled));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, &toFillByteSz, buffer_pos);
    PoplateReceiveResults(buffer, numBytesFilled, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetBaseID(int* baseArbId)
{
    RECEIVE_HELPER("GetBaseID", sizeof(*baseArbId));
    PoplateReceiveResults(buffer, baseArbId, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::SetDemand(int mode, int demand0, int demand1)
{
    Send("SetDemand", mode, demand0, demand1);
}

void SnobotSim::CtreMotControllerWrapper::Set_4(int mode, double demand0, double demand1, int demand1Type)
{
    Send("Set_4", mode, demand0, demand1, demand1Type);
}

void SnobotSim::CtreMotControllerWrapper::SetNeutralMode(int neutralMode)
{
    Send("SetNeutralMode", neutralMode);
}

void SnobotSim::CtreMotControllerWrapper::SetSensorPhase(bool PhaseSensor)
{
    Send("SetSensorPhase", PhaseSensor);
}

void SnobotSim::CtreMotControllerWrapper::SetInverted(bool invert)
{
    Send("SetInverted", invert);
}

void SnobotSim::CtreMotControllerWrapper::SetInverted_2(int invertType)
{
    Send("SetInverted_2", invertType);
}

void SnobotSim::CtreMotControllerWrapper::ConfigFactoryDefault(int timeoutMs)
{
    Send("ConfigFactoryDefault");
}

void SnobotSim::CtreMotControllerWrapper::ConfigOpenLoopRamp(double secondsFromNeutralToFull, int timeoutMs)
{
    Send("ConfigOpenLoopRamp", secondsFromNeutralToFull);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClosedLoopRamp(double secondsFromNeutralToFull, int timeoutMs)
{
    Send("ConfigClosedLoopRamp", secondsFromNeutralToFull);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPeakOutputForward(double percentOut, int timeoutMs)
{
    Send("ConfigPeakOutputForward", percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPeakOutputReverse(double percentOut, int timeoutMs)
{
    Send("ConfigPeakOutputReverse", percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigNominalOutputForward(double percentOut, int timeoutMs)
{
    Send("ConfigNominalOutputForward", percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigNominalOutputReverse(double percentOut, int timeoutMs)
{
    Send("ConfigNominalOutputReverse", percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigNeutralDeadband(double percentDeadband, int timeoutMs)
{
    Send("ConfigNeutralDeadband", percentDeadband);
}

void SnobotSim::CtreMotControllerWrapper::ConfigVoltageCompSaturation(double voltage, int timeoutMs)
{
    Send("ConfigVoltageCompSaturation", voltage);
}

void SnobotSim::CtreMotControllerWrapper::ConfigVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs)
{
    Send("ConfigVoltageMeasurementFilter", filterWindowSamples);
}

void SnobotSim::CtreMotControllerWrapper::EnableVoltageCompensation(bool enable)
{
    Send("EnableVoltageCompensation", enable);
}

void SnobotSim::CtreMotControllerWrapper::GetInverted(bool* invert)
{
    RECEIVE_HELPER("GetInverted", sizeof(*invert));
    PoplateReceiveResults(buffer, invert, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetBusVoltage(double* voltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*voltage));
    PoplateReceiveResults(buffer, voltage, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetMotorOutputPercent(double* percentOutput)
{
    RECEIVE_HELPER("GetMotorOutputPercent", sizeof(*percentOutput));
    PoplateReceiveResults(buffer, percentOutput, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetOutputCurrent(double* current)
{
    RECEIVE_HELPER("GetOutputCurrent", sizeof(*current));
    PoplateReceiveResults(buffer, current, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetSupplyCurrent(double* current)
{
    RECEIVE_HELPER("GetSupplyCurrent", sizeof(*current));
    PoplateReceiveResults(buffer, current, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetStatorCurrent(double* current)
{
    RECEIVE_HELPER("GetStatorCurrent", sizeof(*current));
    PoplateReceiveResults(buffer, current, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetTemperature(double* temperature)
{
    RECEIVE_HELPER("GetTemperature", sizeof(*temperature));
    PoplateReceiveResults(buffer, temperature, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSelectedFeedbackSensor(int feedbackDevice, int pidIdx, int timeoutMs)
{
    Send("ConfigSelectedFeedbackSensor", feedbackDevice, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs)
{
    Send("ConfigSelectedFeedbackCoefficient", coefficient, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::ConfigRemoteFeedbackFilter(int deviceID, int remoteSensorSource, int remoteOrdinal, int timeoutMs)
{
    Send("ConfigRemoteFeedbackFilter", deviceID, remoteSensorSource, remoteOrdinal);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSensorTerm(int sensorTerm, int feedbackDevice, int timeoutMs)
{
    Send("ConfigSensorTerm", sensorTerm, feedbackDevice);
}

void SnobotSim::CtreMotControllerWrapper::GetSelectedSensorPosition(int* param, int pidIdx)
{
    RECEIVE_HELPER("GetSelectedSensorPosition", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetSelectedSensorVelocity(int* param, int pidIdx)
{
    RECEIVE_HELPER("GetSelectedSensorVelocity", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::SetSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs)
{
    Send("SetSelectedSensorPosition", sensorPos, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::SetControlFramePeriod(int frame, int periodMs)
{
    Send("SetControlFramePeriod", frame, periodMs);
}

void SnobotSim::CtreMotControllerWrapper::SetStatusFramePeriod(int frame, uint8_t periodMs, int timeoutMs)
{
    Send("SetStatusFramePeriod", frame, periodMs);
}

void SnobotSim::CtreMotControllerWrapper::GetStatusFramePeriod(int frame, int* periodMs, int timeoutMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigVelocityMeasurementPeriod(int period, int timeoutMs)
{
    Send("ConfigVelocityMeasurementPeriod", period);
}

void SnobotSim::CtreMotControllerWrapper::ConfigVelocityMeasurementWindow(int windowSize, int timeoutMs)
{
    Send("ConfigVelocityMeasurementWindow", windowSize);
}

void SnobotSim::CtreMotControllerWrapper::ConfigForwardLimitSwitchSource(int type, int normalOpenOrClose, int deviceID, int timeoutMs)
{
    Send("ConfigForwardLimitSwitchSource", type, normalOpenOrClose, deviceID);
}

void SnobotSim::CtreMotControllerWrapper::ConfigReverseLimitSwitchSource(int type, int normalOpenOrClose, int deviceID, int timeoutMs)
{
    Send("ConfigReverseLimitSwitchSource", type, normalOpenOrClose, deviceID);
}

void SnobotSim::CtreMotControllerWrapper::OverrideLimitSwitchesEnable(bool enable)
{
    Send("OverrideLimitSwitchesEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs)
{
    Send("ConfigForwardSoftLimitThreshold", forwardSensorLimit);
}

void SnobotSim::CtreMotControllerWrapper::ConfigReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs)
{
    Send("ConfigReverseSoftLimitThreshold", reverseSensorLimit);
}

void SnobotSim::CtreMotControllerWrapper::ConfigForwardSoftLimitEnable(bool enable, int timeoutMs)
{
    Send("ConfigForwardSoftLimitEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigReverseSoftLimitEnable(bool enable, int timeoutMs)
{
    Send("ConfigReverseSoftLimitEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::OverrideSoftLimitsEnable(bool enable)
{
    Send("OverrideSoftLimitsEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::Config_kP(int slotIdx, double value, int timeoutMs)
{
    Send("Config_kP", slotIdx, value);
}

void SnobotSim::CtreMotControllerWrapper::Config_kI(int slotIdx, double value, int timeoutMs)
{
    Send("Config_kI", slotIdx, value);
}

void SnobotSim::CtreMotControllerWrapper::Config_kD(int slotIdx, double value, int timeoutMs)
{
    Send("Config_kD", slotIdx, value);
}

void SnobotSim::CtreMotControllerWrapper::Config_kF(int slotIdx, double value, int timeoutMs)
{
    Send("Config_kF", slotIdx, value);
}

void SnobotSim::CtreMotControllerWrapper::Config_IntegralZone(int slotIdx, double izone, int timeoutMs)
{
    Send("Config_IntegralZone", slotIdx, izone);
}

void SnobotSim::CtreMotControllerWrapper::ConfigAllowableClosedloopError(int slotIdx, int allowableClosedLoopError, int timeoutMs)
{
    Send("ConfigAllowableClosedloopError", slotIdx, allowableClosedLoopError);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs)
{
    Send("ConfigMaxIntegralAccumulator", slotIdx, iaccum);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs)
{
    Send("ConfigClosedLoopPeakOutput", slotIdx, percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs)
{
    Send("ConfigClosedLoopPeriod", slotIdx, loopTimeMs);
}

void SnobotSim::CtreMotControllerWrapper::SetIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs)
{
    Send("SetIntegralAccumulator", iaccum, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::GetClosedLoopError(int* closedLoopError, int pidIdx)
{
    RECEIVE_HELPER("GetClosedLoopError", sizeof(*closedLoopError) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, closedLoopError, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetIntegralAccumulator(double* iaccum, int pidIdx)
{
    RECEIVE_HELPER("GetIntegralAccumulator", sizeof(*iaccum) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, iaccum, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetErrorDerivative(double* derror, int pidIdx)
{
    RECEIVE_HELPER("GetErrorDerivative", sizeof(*derror) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, derror, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::SelectProfileSlot(int slotIdx, int pidIdx)
{
    Send("SelectProfileSlot", slotIdx, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryPosition(int* param)
{
    RECEIVE_HELPER("GetActiveTrajectoryPosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryVelocity(int* param)
{
    RECEIVE_HELPER("GetActiveTrajectoryVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryHeading(double* param)
{
    RECEIVE_HELPER("GetActiveTrajectoryHeading", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryPosition_3(int* param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryPosition_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryVelocity_3(int* param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryVelocity_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryArbFeedFwd_3(double* param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryArbFeedFwd_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryAll(int* vel, int* pos, double* heading)
{
    RECEIVE_HELPER("GetActiveTrajectoryAll", sizeof(*vel) + sizeof(*pos) + sizeof(*heading));
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, heading, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryAll_5(int* vel, int* pos, double* arbFeedFwd, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryAll_5", sizeof(*vel) + sizeof(*pos) + sizeof(*arbFeedFwd) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, arbFeedFwd, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs)
{
    Send("ConfigMotionCruiseVelocity", sensorUnitsPer100ms);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs)
{
    Send("ConfigMotionAcceleration", sensorUnitsPer100msPerSec);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionSCurveStrength(int curveStrength, int timeoutMs)
{
    Send("ConfigMotionSCurveStrength", curveStrength);
}

void SnobotSim::CtreMotControllerWrapper::ClearMotionProfileTrajectories()
{
    Send("ClearMotionProfileTrajectories");
}

void SnobotSim::CtreMotControllerWrapper::GetMotionProfileTopLevelBufferCount(int* value)
{
    RECEIVE_HELPER("GetMotionProfileTopLevelBufferCount", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::PushMotionProfileTrajectory(double position, double velocity, double headingDeg, int profileSlotSelect, bool isLastPoint, bool zeroPos)
{
    Send("PushMotionProfileTrajectory", position, velocity, headingDeg, profileSlotSelect, isLastPoint, zeroPos);
}

void SnobotSim::CtreMotControllerWrapper::PushMotionProfileTrajectory_2(double position, double velocity, double headingDeg, int profileSlotSelect0, int profileSlotSelect1, bool isLastPoint, bool zeroPos, int durationMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //
    //    Send("PushMotionProfileTrajectory_2", position, velocity, headingDeg, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos, durationMs);
}

void SnobotSim::CtreMotControllerWrapper::PushMotionProfileTrajectory_3(double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos0, uint32_t timeDur, bool useAuxPID)
{
    Send("PushMotionProfileTrajectory_3", position, velocity, arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos0, timeDur, useAuxPID);
}

void SnobotSim::CtreMotControllerWrapper::StartMotionProfile(void* streamHandle, uint32_t minBufferedPts, ctre::phoenix::motorcontrol::ControlMode controlMode)
{
    Send("StartMotionProfile", streamHandle, minBufferedPts, controlMode);
}

void SnobotSim::CtreMotControllerWrapper::IsMotionProfileFinished(bool* value)
{
    RECEIVE_HELPER("IsMotionProfileFinished", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::IsMotionProfileTopLevelBufferFull(bool* value)
{
    RECEIVE_HELPER("IsMotionProfileTopLevelBufferFull", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ProcessMotionProfileBuffer()
{
    Send("ProcessMotionProfileBuffer");
}

void SnobotSim::CtreMotControllerWrapper::GetMotionProfileStatus(size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt, bool* hasUnderrun, bool* isUnderrun, bool* activePointValid, bool* isLast, int* profileSlotSelect, int* outputEnable)
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
}

void SnobotSim::CtreMotControllerWrapper::GetMotionProfileStatus_2(size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt, bool* hasUnderrun, bool* isUnderrun, bool* activePointValid, bool* isLast, int* profileSlotSelect, int* outputEnable, int* timeDurMs, int* profileSlotSelect1)
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
}

void SnobotSim::CtreMotControllerWrapper::ClearMotionProfileHasUnderrun(int timeoutMs)
{
    Send("ClearMotionProfileHasUnderrun");
}

void SnobotSim::CtreMotControllerWrapper::ChangeMotionControlFramePeriod(int periodMs)
{
    Send("ChangeMotionControlFramePeriod", periodMs);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionProfileTrajectoryPeriod(int durationMs, int timeoutMs)
{
    Send("ConfigMotionProfileTrajectoryPeriod", durationMs);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionProfileTrajectoryInterpolationEnable(bool enable, int timeoutMs)
{
    Send("ConfigMotionProfileTrajectoryInterpolationEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigFeedbackNotContinuous(bool feedbackNotContinuous, int timeoutMs)
{
    Send("ConfigFeedbackNotContinuous", feedbackNotContinuous);
}

void SnobotSim::CtreMotControllerWrapper::ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(bool remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs)
{
    Send("ConfigRemoteSensorClosedLoopDisableNeutralOnLOS", remoteSensorClosedLoopDisableNeutralOnLOS);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClearPositionOnLimitF(bool clearPositionOnLimitF, int timeoutMs)
{
    Send("ConfigClearPositionOnLimitF", clearPositionOnLimitF);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClearPositionOnLimitR(bool clearPositionOnLimitR, int timeoutMs)
{
    Send("ConfigClearPositionOnLimitR", clearPositionOnLimitR);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx, int timeoutMs)
{
    Send("ConfigClearPositionOnQuadIdx", clearPositionOnQuadIdx);
}

void SnobotSim::CtreMotControllerWrapper::ConfigLimitSwitchDisableNeutralOnLOS(bool limitSwitchDisableNeutralOnLOS, int timeoutMs)
{
    Send("ConfigLimitSwitchDisableNeutralOnLOS", limitSwitchDisableNeutralOnLOS);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSoftLimitDisableNeutralOnLOS(bool softLimitDisableNeutralOnLOS, int timeoutMs)
{
    Send("ConfigSoftLimitDisableNeutralOnLOS", softLimitDisableNeutralOnLOS);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPulseWidthPeriod_EdgesPerRot(int pulseWidthPeriod_EdgesPerRot, int timeoutMs)
{
    Send("ConfigPulseWidthPeriod_EdgesPerRot", pulseWidthPeriod_EdgesPerRot);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPulseWidthPeriod_FilterWindowSz(int pulseWidthPeriod_FilterWindowSz, int timeoutMs)
{
    Send("ConfigPulseWidthPeriod_FilterWindowSz", pulseWidthPeriod_FilterWindowSz);
}

ctre::phoenix::ErrorCode SnobotSim::CtreMotControllerWrapper::GetLastError()
{
    int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)lastError;
}

void SnobotSim::CtreMotControllerWrapper::GetFirmwareVersion(int* version)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*version));
    PoplateReceiveResults(buffer, version, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::HasResetOccurred(bool* output)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*output));
    PoplateReceiveResults(buffer, output, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSetCustomParam(int newValue, int paramIndex, int timeoutMs)
{
    Send("ConfigSetCustomParam", newValue, paramIndex);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetCustomParam(int* readValue, int paramIndex, int timoutMs)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex) + sizeof(timoutMs));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);
    PoplateReceiveResults(buffer, &timoutMs, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    Send("ConfigSetParameter", param, value, subValue, ordinal);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetParameter(int param, double* value, int ordinal, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPeakCurrentLimit(int amps, int timeoutMs)
{
    Send("ConfigPeakCurrentLimit", amps);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPeakCurrentDuration(int milliseconds, int timeoutMs)
{
    Send("ConfigPeakCurrentDuration", milliseconds);
}

void SnobotSim::CtreMotControllerWrapper::ConfigContinuousCurrentLimit(int amps, int timeoutMs)
{
    Send("ConfigContinuousCurrentLimit", amps);
}

void SnobotSim::CtreMotControllerWrapper::EnableCurrentLimit(bool enable)
{
    Send("EnableCurrentLimit", enable);
}

void SnobotSim::CtreMotControllerWrapper::SetLastError(int error)
{
    Send("SetLastError", error);
}

void SnobotSim::CtreMotControllerWrapper::GetAnalogIn(int* param)
{
    RECEIVE_HELPER("GetAnalogIn", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::SetAnalogPosition(int newPosition, int timeoutMs)
{
    Send("SetAnalogPosition", newPosition);
}

void SnobotSim::CtreMotControllerWrapper::GetAnalogInRaw(int* param)
{
    RECEIVE_HELPER("GetAnalogInRaw", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetAnalogInVel(int* param)
{
    RECEIVE_HELPER("GetAnalogInVel", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetQuadraturePosition(int* param)
{
    RECEIVE_HELPER("GetQuadraturePosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::SetQuadraturePosition(int newPosition, int timeoutMs)
{
    Send("SetQuadraturePosition", newPosition);
}

void SnobotSim::CtreMotControllerWrapper::GetQuadratureVelocity(int* param)
{
    RECEIVE_HELPER("GetQuadratureVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthPosition(int* param)
{
    RECEIVE_HELPER("GetPulseWidthPosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::SetPulseWidthPosition(int newPosition, int timeoutMs)
{
    Send("SetPulseWidthPosition", newPosition);
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthVelocity(int* param)
{
    RECEIVE_HELPER("GetPulseWidthVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthRiseToFallUs(int* param)
{
    RECEIVE_HELPER("GetPulseWidthRiseToFallUs", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthRiseToRiseUs(int* param)
{
    RECEIVE_HELPER("GetPulseWidthRiseToRiseUs", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetPinStateQuadA(int* param)
{
    RECEIVE_HELPER("GetPinStateQuadA", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetPinStateQuadB(int* param)
{
    RECEIVE_HELPER("GetPinStateQuadB", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetPinStateQuadIdx(int* param)
{
    RECEIVE_HELPER("GetPinStateQuadIdx", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::IsFwdLimitSwitchClosed(int* param)
{
    RECEIVE_HELPER("IsFwdLimitSwitchClosed", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::IsRevLimitSwitchClosed(int* param)
{
    RECEIVE_HELPER("IsRevLimitSwitchClosed", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetFaults(int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetStickyFaults(int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ClearStickyFaults(int timeoutMs)
{
    Send("ClearStickyFaults");
}

void SnobotSim::CtreMotControllerWrapper::SelectDemandType(bool enable)
{
    Send("SelectDemandType", enable);
}

void SnobotSim::CtreMotControllerWrapper::SetMPEOutput(int MpeOutput)
{
    Send("SetMPEOutput", MpeOutput);
}

void SnobotSim::CtreMotControllerWrapper::EnableHeadingHold(bool enable)
{
    Send("EnableHeadingHold", enable);
}

void SnobotSim::CtreMotControllerWrapper::GetAnalogInAll(int* withOv, int* raw, int* vel)
{
    RECEIVE_HELPER("GetAnalogInAll", sizeof(*withOv) + sizeof(*raw) + sizeof(*vel));
    PoplateReceiveResults(buffer, withOv, buffer_pos);
    PoplateReceiveResults(buffer, raw, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetQuadratureSensor(int* pos, int* vel)
{
    RECEIVE_HELPER("GetQuadratureSensor", sizeof(*pos) + sizeof(*vel));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthAll(int* pos, int* vel, int* riseToRiseUs, int* riseToFallUs)
{
    RECEIVE_HELPER("GetPulseWidthAll", sizeof(*pos) + sizeof(*vel) + sizeof(*riseToRiseUs) + sizeof(*riseToFallUs));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, riseToRiseUs, buffer_pos);
    PoplateReceiveResults(buffer, riseToFallUs, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetQuadPinStates(int* quadA, int* quadB, int* quadIdx)
{
    RECEIVE_HELPER("GetQuadPinStates", sizeof(*quadA) + sizeof(*quadB) + sizeof(*quadIdx));
    PoplateReceiveResults(buffer, quadA, buffer_pos);
    PoplateReceiveResults(buffer, quadB, buffer_pos);
    PoplateReceiveResults(buffer, quadIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetLimitSwitchState(int* isFwdClosed, int* isRevClosed)
{
    RECEIVE_HELPER("GetLimitSwitchState", sizeof(*isFwdClosed) + sizeof(*isRevClosed));
    PoplateReceiveResults(buffer, isFwdClosed, buffer_pos);
    PoplateReceiveResults(buffer, isRevClosed, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::GetClosedLoopTarget(int* value, int pidIdx)
{
    RECEIVE_HELPER("GetClosedLoopTarget", sizeof(*value) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotorCommutation(ctre::phoenix::motorcontrol::MotorCommutation motorCommutation, int timeoutMs)
{
    Send("ConfigMotorCommutation", motorCommutation);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetMotorCommutation(ctre::phoenix::motorcontrol::MotorCommutation* motorCommutation, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetMotorCommutation", sizeof(*motorCommutation));
    PoplateReceiveResults(buffer, motorCommutation, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSupplyCurrentLimit(const double* params, int paramCnt, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("ConfigSupplyCurrentLimit", sizeof(*params) + sizeof(paramCnt) + sizeof(timeoutMs));
    //    PoplateReceiveResults(buffer, params, buffer_pos);
    //    PoplateReceiveResults(buffer, &paramCnt, buffer_pos);
    //    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigStatorCurrentLimit(const double* params, int paramCnt, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("ConfigStatorCurrentLimit", sizeof(*params) + sizeof(paramCnt) + sizeof(timeoutMs));
    //    PoplateReceiveResults(buffer, params, buffer_pos);
    //    PoplateReceiveResults(buffer, &paramCnt, buffer_pos);
    //    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSupplyCurrentLimitEnable(bool enable, int timeoutMs)
{
    Send("ConfigSupplyCurrentLimitEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigStatorCurrentLimitEnable(bool enable, int timeoutMs)
{
    Send("ConfigStatorCurrentLimitEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetSupplyCurrentLimit(double* toFill, int* fillCnt, int fillCapacity, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetSupplyCurrentLimit", sizeof(*toFill) + sizeof(*fillCnt) + sizeof(fillCapacity));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, fillCnt, buffer_pos);
    PoplateReceiveResults(buffer, &fillCapacity, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetStatorCurrentLimit(double* toFill, int* fillCnt, int fillCapacity, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetStatorCurrentLimit", sizeof(*toFill) + sizeof(*fillCnt) + sizeof(fillCapacity));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, fillCnt, buffer_pos);
    PoplateReceiveResults(buffer, &fillCapacity, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::SetIntegratedSensorPosition(double newpos, int timeoutMs)
{
    Send("SetIntegratedSensorPosition", newpos);
}

void SnobotSim::CtreMotControllerWrapper::SetIntegratedSensorPositionToAbsolute(int timeoutMs)
{
    Send("SetIntegratedSensorPositionToAbsolute");
}

void SnobotSim::CtreMotControllerWrapper::GetIntegratedSensor(double* pos, double* absPos, double* vel)
{
    RECEIVE_HELPER("GetIntegratedSensor", sizeof(*pos) + sizeof(*absPos) + sizeof(*vel));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, absPos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigIntegratedSensorAbsoluteRange(ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange, int timeoutMs)
{
    Send("ConfigIntegratedSensorAbsoluteRange", absoluteSensorRange);
}

void SnobotSim::CtreMotControllerWrapper::ConfigIntegratedSensorOffset(double offsetDegrees, int timeoutMs)
{
    Send("ConfigIntegratedSensorOffset", offsetDegrees);
}

void SnobotSim::CtreMotControllerWrapper::ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy, int timeoutMs)
{
    Send("ConfigIntegratedSensorInitializationStrategy", initializationStrategy);
}
