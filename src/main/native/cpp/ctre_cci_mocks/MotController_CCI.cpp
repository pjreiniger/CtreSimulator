#include "ctre/phoenix/cci/MotController_CCI.h"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "CtreSimMocks/CtreMotorControllerWrapper.h"
#include "CtreSimMocks/MockHooks.h"

typedef SnobotSim::CtreMotorControllerWrapper MotorControllerWrapper;

#define RECEIVE_HELPER(paramName, size)                                        \
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle); \
    uint8_t buffer[size]; /* NOLINT */                                         \
    std::memset(&buffer[0], 0, size);                                          \
    wrapper->Receive(paramName, buffer, size);                                 \
    uint32_t buffer_pos = 0;


MotorControllerWrapper* ConvertToMotorControllerWrapper(void* param)
{
    return reinterpret_cast<MotorControllerWrapper*>(param);
}

extern "C"{

void* c_MotController_Create1(int baseArbId)
{
	SnobotSim::CtreMotorControllerWrapper* output = new SnobotSim::CtreMotorControllerWrapper(baseArbId);
    return output;
}

void* c_MotController_Create2(int deviceID, const char * model)
{
	SnobotSim::CtreMotorControllerWrapper* output = new SnobotSim::CtreMotorControllerWrapper(deviceID);
    return output;
}

void c_MotController_DestroyAll()
{

}

ctre::phoenix::ErrorCode c_MotController_Destroy(void *handle)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    delete wrapper;
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetDeviceNumber(void *handle, int *deviceNumber)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    *deviceNumber = wrapper->mDeviceId;
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetDescription(void *handle, char * toFill, int toFillByteSz, size_t * numBytesFilled)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetBaseID(void* handle, int* baseArbId)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    *baseArbId = wrapper->mDeviceId;
    return (ctre::phoenix::ErrorCode)0;
}


ctre::phoenix::ErrorCode c_MotController_SetDemand(void *handle, int mode, int demand0, int demand1)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetDemand", mode, demand0, demand1);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Set_4(void *handle, int mode, double demand0, double demand1, int demand1Type)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("Set_4", mode, demand0, demand1, demand1Type);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_SetNeutralMode(void *handle, int neutralMode)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetNeutralMode", neutralMode);
}

void c_MotController_SetSensorPhase(void *handle, bool PhaseSensor)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetSensorPhase", PhaseSensor);
}

void c_MotController_SetInverted(void *handle, bool invert)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetInverted", invert);
}

void c_MotController_SetInverted_2(void *handle, int invertType)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetInverted_2", invertType);
}

ctre::phoenix::ErrorCode c_MotController_ConfigFactoryDefault(void *handle, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigFactoryDefault");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigOpenLoopRamp(void *handle, double secondsFromNeutralToFull, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigOpenLoopRamp", secondsFromNeutralToFull);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopRamp(void *handle, double secondsFromNeutralToFull, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigClosedLoopRamp", secondsFromNeutralToFull);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakOutputForward(void *handle, double percentOut, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigPeakOutputForward", percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakOutputReverse(void *handle, double percentOut, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigPeakOutputReverse", percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigNominalOutputForward(void *handle, double percentOut, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigNominalOutputForward", percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigNominalOutputReverse(void *handle, double percentOut, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigNominalOutputReverse", percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigNeutralDeadband(void *handle, double percentDeadband, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigNeutralDeadband", percentDeadband);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVoltageCompSaturation(void *handle, double voltage, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigVoltageCompSaturation", voltage);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVoltageMeasurementFilter(void *handle, int filterWindowSamples, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigVoltageMeasurementFilter", filterWindowSamples);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_EnableVoltageCompensation(void *handle, bool enable)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("EnableVoltageCompensation", enable);
}

ctre::phoenix::ErrorCode c_MotController_GetInverted(void *handle, bool *invert)
{
    RECEIVE_HELPER("GetInverted", sizeof(*invert));
    PoplateReceiveResults(buffer, invert, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetBusVoltage(void *handle, double *voltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*voltage));
    PoplateReceiveResults(buffer, voltage, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotorOutputPercent(void *handle, double *percentOutput)
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
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetStatorCurrent(void* handle, double* current)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}


ctre::phoenix::ErrorCode c_MotController_GetTemperature(void *handle, double *temperature)
{
    RECEIVE_HELPER("GetTemperature", sizeof(*temperature));
    PoplateReceiveResults(buffer, temperature, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSelectedFeedbackSensor(void *handle, int feedbackDevice, int pidIdx, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigSelectedFeedbackSensor", feedbackDevice, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSelectedFeedbackCoefficient(void *handle, double coefficient, int pidIdx, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigSelectedFeedbackCoefficient", coefficient, pidIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigRemoteFeedbackFilter(void *handle, int deviceID, int remoteSensorSource, int remoteOrdinal, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigRemoteFeedbackFilter", deviceID, remoteSensorSource, remoteOrdinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSensorTerm(void *handle, int sensorTerm, int feedbackDevice, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigSensorTerm", sensorTerm, feedbackDevice, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetSelectedSensorPosition(void *handle, int *param, int pidIdx)
{
    RECEIVE_HELPER("GetSelectedSensorPosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetSelectedSensorVelocity(void *handle, int *param, int pidIdx)
{
    RECEIVE_HELPER("GetSelectedSensorVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetSelectedSensorPosition(void *handle, int sensorPos, int pidIdx,int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetSelectedSensorPosition", sensorPos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetControlFramePeriod(void *handle, int frame, int periodMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetControlFramePeriod", frame);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetStatusFramePeriod(void *handle, int frame, uint8_t periodMs, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetStatusFramePeriod", frame, periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetStatusFramePeriod(void *handle, int frame, int *periodMs, int timeoutMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(*periodMs));
    PoplateReceiveResults(buffer, periodMs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVelocityMeasurementPeriod(void *handle, int period, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigVelocityMeasurementPeriod", period);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigVelocityMeasurementWindow(void *handle, int windowSize, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigVelocityMeasurementWindow", windowSize);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigForwardLimitSwitchSource(void *handle, int type, int normalOpenOrClose, int deviceID, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigForwardLimitSwitchSource", type, normalOpenOrClose, deviceID);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigReverseLimitSwitchSource(void *handle, int type, int normalOpenOrClose, int deviceID, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigReverseLimitSwitchSource", type, normalOpenOrClose, deviceID);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_OverrideLimitSwitchesEnable(void *handle, bool enable)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("OverrideLimitSwitchesEnable", enable);
}

ctre::phoenix::ErrorCode c_MotController_ConfigForwardSoftLimitThreshold(void *handle, int forwardSensorLimit, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigForwardSoftLimitThreshold", forwardSensorLimit, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigReverseSoftLimitThreshold(void *handle, int reverseSensorLimit, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigReverseSoftLimitThreshold", reverseSensorLimit, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigForwardSoftLimitEnable(void *handle, bool enable, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigForwardSoftLimitEnable", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigReverseSoftLimitEnable(void *handle, bool enable, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigReverseSoftLimitEnable", enable);
    return (ctre::phoenix::ErrorCode)0;
}

void c_MotController_OverrideSoftLimitsEnable(void *handle, bool enable)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("OverrideSoftLimitsEnable", enable);
}

ctre::phoenix::ErrorCode c_MotController_Config_kP(void *handle, int slotIdx, double value, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("Config_kP", slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_kI(void *handle, int slotIdx, double value, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("Config_kI", slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_kD(void *handle, int slotIdx, double value, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("Config_kD", slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_kF(void *handle, int slotIdx, double value, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("Config_kF", slotIdx, value);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_Config_IntegralZone(void *handle, int slotIdx, double izone, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("Config_IntegralZone", slotIdx, izone);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigAllowableClosedloopError(void *handle, int slotIdx, int allowableClosedLoopError, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigAllowableClosedloopError", slotIdx, allowableClosedLoopError);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMaxIntegralAccumulator(void *handle, int slotIdx, double iaccum, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigMaxIntegralAccumulator", slotIdx, iaccum);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopPeakOutput(void *handle, int slotIdx, double percentOut, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigClosedLoopPeakOutput", slotIdx, percentOut);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopPeriod(void *handle, int slotIdx, int loopTimeMs, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigClosedLoopPeriod", slotIdx, loopTimeMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetIntegralAccumulator(void *handle, double iaccum, int pidIdx, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetIntegralAccumulator", iaccum);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetClosedLoopError(void *handle, int *closedLoopError, int pidIdx)
{
    RECEIVE_HELPER("GetClosedLoopError", sizeof(*closedLoopError));
    PoplateReceiveResults(buffer, closedLoopError, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetIntegralAccumulator(void *handle, double *iaccum, int pidIdx)
{
    RECEIVE_HELPER("GetIntegralAccumulator", sizeof(*iaccum));
    PoplateReceiveResults(buffer, iaccum, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetErrorDerivative(void *handle, double *derror, int pidIdx)
{
    RECEIVE_HELPER("GetErrorDerivative", sizeof(*derror));
    PoplateReceiveResults(buffer, derror, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SelectProfileSlot(void *handle, int slotIdx, int pidIdx)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SelectProfileSlot", slotIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryPosition(void *handle, int *param)
{
    RECEIVE_HELPER("GetActiveTrajectoryPosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryVelocity(void *handle, int *param)
{
    RECEIVE_HELPER("GetActiveTrajectoryVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryHeading(void *handle, double *param)
{
    RECEIVE_HELPER("GetActiveTrajectoryHeading", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode) 0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryPosition_3(void *handle, int *param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryPosition_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryVelocity_3(void *handle, int *param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryVelocity_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryArbFeedFwd_3(void *handle, double *param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryArbFeedFwd_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryAll(void *handle, int * vel, int * pos, double *heading)
{
    RECEIVE_HELPER("GetActiveTrajectoryAll", sizeof(*vel) + sizeof(*pos) + sizeof(*heading));
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, heading, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionCruiseVelocity(void *handle, int sensorUnitsPer100ms, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigMotionCruiseVelocity", sensorUnitsPer100ms);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionAcceleration(void *handle, int sensorUnitsPer100msPerSec, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigMotionAcceleration", sensorUnitsPer100msPerSec);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionSCurveStrength(void *handle, int curveStrength, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigMotionSCurveStrength", curveStrength);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ClearMotionProfileTrajectories(void *handle)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ClearMotionProfileTrajectories");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotionProfileTopLevelBufferCount(void *handle, int * value)
{
    RECEIVE_HELPER("GetMotionProfileTopLevelBufferCount", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory(void *handle, double position,
		double velocity, double headingDeg, int profileSlotSelect, bool isLastPoint, bool zeroPos)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("PushMotionProfileTrajectory", position, velocity, headingDeg, profileSlotSelect, isLastPoint, zeroPos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory_2(
		void *handle, double position, double velocity, double headingDeg,
		int profileSlotSelect0, int profileSlotSelect1, bool isLastPoint, bool zeroPos, int durationMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("PushMotionProfileTrajectory_2", position, velocity, headingDeg,
            profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory_3(
        void *handle, double position, double velocity,
        double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd,
        uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos0, uint32_t timeDur, bool useAuxPID)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("PushMotionProfileTrajectory_3", position, velocity,
            arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd,
            profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos0, timeDur, useAuxPID);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_StartMotionProfile(void *handle, void * streamHandle, uint32_t minBufferedPts, ctre::phoenix::motorcontrol::ControlMode controlMode)
{
    int castControlMode = (int) controlMode;
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("StartMotionProfile", minBufferedPts, castControlMode);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsMotionProfileTopLevelBufferFull(void *handle, bool * value)
{
    RECEIVE_HELPER("IsMotionProfileTopLevelBufferFull", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ProcessMotionProfileBuffer(void *handle)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ProcessMotionProfileBuffer");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetMotionProfileStatus(void *handle,
        size_t *topBufferRem, size_t *topBufferCnt, int *btmBufferCnt,
		bool *hasUnderrun, bool *isUnderrun, bool *activePointValid,
		bool *isLast, int *profileSlotSelect, int *outputEnable)
{
    RECEIVE_HELPER("GetMotionProfileStatus", sizeof(int) * 5 + sizeof(bool) * 4);
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

ctre::phoenix::ErrorCode c_MotController_GetMotionProfileStatus_2(void *handle,
        size_t *topBufferRem, size_t *topBufferCnt, int *btmBufferCnt,
		bool *hasUnderrun, bool *isUnderrun, bool *activePointValid,
		bool *isLast, int *profileSlotSelect, int *outputEnable, int *timeDurMs,
		int *profileSlotSelect1)
{
    RECEIVE_HELPER("GetMotionProfileStatus2", sizeof(int) * 7 + sizeof(bool) * 4);
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

ctre::phoenix::ErrorCode c_MotController_ClearMotionProfileHasUnderrun(void *handle,
		int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ClearMotionProfileHasUnderrun", timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ChangeMotionControlFramePeriod(void *handle,
		int periodMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ChangeMotionControlFramePeriod", periodMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionProfileTrajectoryPeriod(
		void *handle, int durationMs, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigMotionProfileTrajectoryPeriod", durationMs, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigMotionProfileTrajectoryInterpolationEnable(void *handle, bool enable, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigMotionProfileTrajectoryInterpolationEnable", enable, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigFeedbackNotContinuous(void *handle,
            bool feedbackNotContinuous, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigFeedbackNotContinuous", feedbackNotContinuous);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(void *handle,
        bool remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigRemoteSensorClosedLoopDisableNeutralOnLOS", remoteSensorClosedLoopDisableNeutralOnLOS);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnLimitF(void *handle,
        bool clearPositionOnLimitF, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigClearPositionOnLimitF", clearPositionOnLimitF);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnLimitR(void *handle,
        bool clearPositionOnLimitR, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigClearPositionOnLimitR", clearPositionOnLimitR);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnQuadIdx(void *handle,
        bool clearPositionOnQuadIdx, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigClearPositionOnQuadIdx", clearPositionOnQuadIdx);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigLimitSwitchDisableNeutralOnLOS(void *handle,
        bool limitSwitchDisableNeutralOnLOS, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigLimitSwitchDisableNeutralOnLOS", limitSwitchDisableNeutralOnLOS);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSoftLimitDisableNeutralOnLOS(void *handle,
        bool softLimitDisableNeutralOnLOS, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigSoftLimitDisableNeutralOnLOS", softLimitDisableNeutralOnLOS);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPulseWidthPeriod_EdgesPerRot(void *handle,
        int pulseWidthPeriod_EdgesPerRot, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigPulseWidthPeriod_EdgesPerRot", pulseWidthPeriod_EdgesPerRot);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPulseWidthPeriod_FilterWindowSz(void *handle,
        int pulseWidthPeriod_FilterWindowSz, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigPulseWidthPeriod_FilterWindowSz", pulseWidthPeriod_FilterWindowSz);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetLastError(void *handle)
{
	int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode) lastError;
}

ctre::phoenix::ErrorCode c_MotController_GetFirmwareVersion(void *handle, int * version)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*version));
    PoplateReceiveResults(buffer, version, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_HasResetOccurred(void *handle,bool * output)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*output));
    PoplateReceiveResults(buffer, output, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSetCustomParam(void *handle, int newValue, int paramIndex, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigSetCustomParam", newValue, paramIndex);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetCustomParam(void *handle, int *readValue, int paramIndex, int timoutMs)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSetParameter(void *handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigSetParameter", param, value, subValue, ordinal);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetParameter(void *handle, int param, double *value, int ordinal, int timeoutMs)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetParameter_6(void *handle, int32_t param, int32_t valueToSend, int32_t * valueRecieved, uint8_t * subValue, int32_t ordinal, int32_t timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakCurrentLimit(void *handle, int amps, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigPeakCurrentLimit", amps);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigPeakCurrentDuration(void *handle, int milliseconds, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigPeakCurrentDuration", milliseconds);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigContinuousCurrentLimit(void *handle, int amps, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ConfigContinuousCurrentLimit", amps);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_EnableCurrentLimit(void *handle, bool enable)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("EnableCurrentLimit", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetLastError(void *handle, int error)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetLastError", error);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogIn(void *handle, int * param)
{
    RECEIVE_HELPER("GetAnalogIn", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetAnalogPosition(void *handle,int newPosition, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetAnalogPosition", newPosition, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogInRaw(void *handle, int * param)
{
    RECEIVE_HELPER("GetAnalogInRaw", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogInVel(void *handle, int * param)
{
    RECEIVE_HELPER("GetAnalogInVel", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadraturePosition(void *handle, int * param)
{
    RECEIVE_HELPER("GetQuadraturePosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetQuadraturePosition(void *handle,int newPosition, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetQuadraturePosition", newPosition, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadratureVelocity(void *handle, int * param)
{
    RECEIVE_HELPER("GetQuadratureVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthPosition(void *handle, int * param)
{
    RECEIVE_HELPER("GetPulseWidthPosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetPulseWidthPosition(void *handle,int newPosition, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetPulseWidthPosition", newPosition, timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthVelocity(void *handle, int * param)
{
    RECEIVE_HELPER("GetPulseWidthVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthRiseToFallUs(void *handle, int * param)
{
    RECEIVE_HELPER("GetPulseWidthRiseToFallUs", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthRiseToRiseUs(void *handle, int * param)
{
    RECEIVE_HELPER("GetPulseWidthRiseToRiseUs", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadA(void *handle, int * param)
{
    RECEIVE_HELPER("GetPinStateQuadA", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadB(void *handle, int * param)
{
    RECEIVE_HELPER("GetPinStateQuadB", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadIdx(void *handle, int * param)
{
    RECEIVE_HELPER("GetPinStateQuadIdx", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsFwdLimitSwitchClosed(void *handle, int * param)
{
    RECEIVE_HELPER("IsFwdLimitSwitchClosed", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_IsRevLimitSwitchClosed(void *handle, int * param)
{
    RECEIVE_HELPER("IsRevLimitSwitchClosed", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetFaults(void *handle, int * param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetStickyFaults(void *handle, int * param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ClearStickyFaults(void *handle, int timeoutMs)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("ClearStickyFaults", timeoutMs);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SelectDemandType(void *handle, bool enable)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SelectDemandType", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetMPEOutput(void *handle, int MpeOutput)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("SetMPEOutput", MpeOutput);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_EnableHeadingHold(void *handle, bool enable)
{
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle);
    wrapper->Send("EnableHeadingHold", enable);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetAnalogInAll(void *handle, int * withOv, int * raw, int * vel)
{
    RECEIVE_HELPER("GetAnalogInAll", sizeof(*withOv) + sizeof(*raw) + sizeof(*vel));
    PoplateReceiveResults(buffer, withOv, buffer_pos);
    PoplateReceiveResults(buffer, raw, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadratureSensor(void *handle, int * pos, int * vel)
{
    RECEIVE_HELPER("GetQuadratureSensor", sizeof(*pos) + sizeof(*vel));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetPulseWidthAll(void *handle, int * pos, int * vel, int * riseToRiseUs, int * riseToFallUs)
{
    RECEIVE_HELPER("GetPulseWidthAll", sizeof(*pos) + sizeof(*vel) + sizeof(*riseToRiseUs) + sizeof(*riseToFallUs));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, riseToRiseUs, buffer_pos);
    PoplateReceiveResults(buffer, riseToFallUs, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetQuadPinStates(void *handle, int * quadA, int * quadB, int * quadIdx)
{
    RECEIVE_HELPER("GetQuadPinStates", sizeof(*quadA) + sizeof(*quadB) + sizeof(*quadIdx));
    PoplateReceiveResults(buffer, quadA, buffer_pos);
    PoplateReceiveResults(buffer, quadB, buffer_pos);
    PoplateReceiveResults(buffer, quadIdx, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetLimitSwitchState(void *handle, int * isFwdClosed, int * isRevClosed)
{
    RECEIVE_HELPER("GetLimitSwitchState", sizeof(*isFwdClosed) + sizeof(*isRevClosed));
    PoplateReceiveResults(buffer, isFwdClosed, buffer_pos);
    PoplateReceiveResults(buffer, isRevClosed, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetClosedLoopTarget(void *handle, int * value, int pidIdx)
{
    RECEIVE_HELPER("GetClosedLoopTarget", sizeof(*value) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    return (ctre::phoenix::ErrorCode)0;
}
ctre::phoenix::ErrorCode c_MotController_ConfigMotorCommutation(void* handle, ctre::phoenix::motorcontrol::MotorCommutation motorCommutation, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetMotorCommutation(void* handle, ctre::phoenix::motorcontrol::MotorCommutation *motorCommutation, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSupplyCurrentLimit(void* handle, const double* params, int paramCnt, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigStatorCurrentLimit(void* handle, const double* params, int paramCnt, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigSupplyCurrentLimitEnable(void* handle, bool enable, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigStatorCurrentLimitEnable(void* handle, bool enable, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetSupplyCurrentLimit(void* handle, double* toFill, int * fillCnt, int fillCapacity, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigGetStatorCurrentLimit(void* handle, double* toFill, int* fillCnt, int fillCapacity, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetIntegratedSensorPosition(void* handle, double newpos, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_SetIntegratedSensorPositionToAbsolute(void* handle, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_GetIntegratedSensor(void* handle, double* pos, double * absPos, double * vel)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigIntegratedSensorAbsoluteRange(void* handle, ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigIntegratedSensorOffset(void* handle, double offsetDegrees, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}

ctre::phoenix::ErrorCode c_MotController_ConfigIntegratedSensorInitializationStrategy(void* handle, ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy, int timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (ctre::phoenix::ErrorCode)0;
}


}  // extern "C"
