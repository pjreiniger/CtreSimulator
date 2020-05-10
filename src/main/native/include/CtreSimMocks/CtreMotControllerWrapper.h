
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/MotController_CCI.h"

namespace SnobotSim
{

class CtreMotControllerWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    explicit CtreMotControllerWrapper(int aDeviceId);
    const int mDeviceId;

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);

    /////////////////////////////////////////////////////////////////////
    void GetDeviceNumber(int* deviceNumber);
    void GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled);
    void GetBaseID(int* baseArbId);
    void SetDemand(int mode, int demand0, int demand1);
    void Set_4(int mode, double demand0, double demand1, int demand1Type);
    void SetNeutralMode(int neutralMode);
    void SetSensorPhase(bool PhaseSensor);
    void SetInverted(bool invert);
    void SetInverted_2(int invertType);
    void ConfigFactoryDefault(int timeoutMs);
    void ConfigOpenLoopRamp(double secondsFromNeutralToFull, int timeoutMs);
    void ConfigClosedLoopRamp(double secondsFromNeutralToFull, int timeoutMs);
    void ConfigPeakOutputForward(double percentOut, int timeoutMs);
    void ConfigPeakOutputReverse(double percentOut, int timeoutMs);
    void ConfigNominalOutputForward(double percentOut, int timeoutMs);
    void ConfigNominalOutputReverse(double percentOut, int timeoutMs);
    void ConfigNeutralDeadband(double percentDeadband, int timeoutMs);
    void ConfigVoltageCompSaturation(double voltage, int timeoutMs);
    void ConfigVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs);
    void EnableVoltageCompensation(bool enable);
    void GetInverted(bool* invert);
    void GetBusVoltage(double* voltage);
    void GetMotorOutputPercent(double* percentOutput);
    void GetOutputCurrent(double* current);
    void GetSupplyCurrent(double* current);
    void GetStatorCurrent(double* current);
    void GetTemperature(double* temperature);
    void ConfigSelectedFeedbackSensor(int feedbackDevice, int pidIdx, int timeoutMs);
    void ConfigSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs);
    void ConfigRemoteFeedbackFilter(int deviceID, int remoteSensorSource, int remoteOrdinal, int timeoutMs);
    void ConfigSensorTerm(int sensorTerm, int feedbackDevice, int timeoutMs);
    void GetSelectedSensorPosition(int* param, int pidIdx);
    void GetSelectedSensorVelocity(int* param, int pidIdx);
    void SetSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs);
    void SetControlFramePeriod(int frame, int periodMs);
    void SetStatusFramePeriod(int frame, uint8_t periodMs, int timeoutMs);
    void GetStatusFramePeriod(int frame, int* periodMs, int timeoutMs);
    void ConfigVelocityMeasurementPeriod(int period, int timeoutMs);
    void ConfigVelocityMeasurementWindow(int windowSize, int timeoutMs);
    void ConfigForwardLimitSwitchSource(int type, int normalOpenOrClose, int deviceID, int timeoutMs);
    void ConfigReverseLimitSwitchSource(int type, int normalOpenOrClose, int deviceID, int timeoutMs);
    void OverrideLimitSwitchesEnable(bool enable);
    void ConfigForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs);
    void ConfigReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs);
    void ConfigForwardSoftLimitEnable(bool enable, int timeoutMs);
    void ConfigReverseSoftLimitEnable(bool enable, int timeoutMs);
    void OverrideSoftLimitsEnable(bool enable);
    void Config_kP(int slotIdx, double value, int timeoutMs);
    void Config_kI(int slotIdx, double value, int timeoutMs);
    void Config_kD(int slotIdx, double value, int timeoutMs);
    void Config_kF(int slotIdx, double value, int timeoutMs);
    void Config_IntegralZone(int slotIdx, double izone, int timeoutMs);
    void ConfigAllowableClosedloopError(int slotIdx, int allowableClosedLoopError, int timeoutMs);
    void ConfigMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs);
    void ConfigClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs);
    void ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs);
    void SetIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs);
    void GetClosedLoopError(int* closedLoopError, int pidIdx);
    void GetIntegralAccumulator(double* iaccum, int pidIdx);
    void GetErrorDerivative(double* derror, int pidIdx);
    void SelectProfileSlot(int slotIdx, int pidIdx);
    void GetActiveTrajectoryPosition(int* param);
    void GetActiveTrajectoryVelocity(int* param);
    void GetActiveTrajectoryHeading(double* param);
    void GetActiveTrajectoryPosition_3(int* param, int pidIdx);
    void GetActiveTrajectoryVelocity_3(int* param, int pidIdx);
    void GetActiveTrajectoryArbFeedFwd_3(double* param, int pidIdx);
    void GetActiveTrajectoryAll(int* vel, int* pos, double* heading);
    void GetActiveTrajectoryAll_5(int* vel, int* pos, double* arbFeedFwd, int pidIdx);
    void ConfigMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs);
    void ConfigMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs);
    void ConfigMotionSCurveStrength(int curveStrength, int timeoutMs);
    void ClearMotionProfileTrajectories();
    void GetMotionProfileTopLevelBufferCount(int* value);
    void PushMotionProfileTrajectory(double position, double velocity, double headingDeg, int profileSlotSelect, bool isLastPoint, bool zeroPos);
    void PushMotionProfileTrajectory_2(
            double position, double velocity, double headingDeg,
            int profileSlotSelect0, int profileSlotSelect1, bool isLastPoint, bool zeroPos, int durationMs);
    void PushMotionProfileTrajectory_3(double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos0, uint32_t timeDur, bool useAuxPID);
    void StartMotionProfile(void* streamHandle, uint32_t minBufferedPts, ctre::phoenix::motorcontrol::ControlMode controlMode);
    void IsMotionProfileFinished(bool* value);
    void IsMotionProfileTopLevelBufferFull(bool* value);
    void ProcessMotionProfileBuffer();
    void GetMotionProfileStatus(
            size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt,
            bool* hasUnderrun, bool* isUnderrun, bool* activePointValid,
            bool* isLast, int* profileSlotSelect, int* outputEnable);
    void GetMotionProfileStatus_2(
            size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt,
            bool* hasUnderrun, bool* isUnderrun, bool* activePointValid,
            bool* isLast, int* profileSlotSelect, int* outputEnable, int* timeDurMs,
            int* profileSlotSelect1);
    void ClearMotionProfileHasUnderrun(int timeoutMs);
    void ChangeMotionControlFramePeriod(int periodMs);
    void ConfigMotionProfileTrajectoryPeriod(int durationMs, int timeoutMs);
    void ConfigMotionProfileTrajectoryInterpolationEnable(bool enable, int timeoutMs);
    void ConfigFeedbackNotContinuous(bool feedbackNotContinuous, int timeoutMs);
    void ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(bool remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs);
    void ConfigClearPositionOnLimitF(bool clearPositionOnLimitF, int timeoutMs);
    void ConfigClearPositionOnLimitR(bool clearPositionOnLimitR, int timeoutMs);
    void ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx, int timeoutMs);
    void ConfigLimitSwitchDisableNeutralOnLOS(bool limitSwitchDisableNeutralOnLOS, int timeoutMs);
    void ConfigSoftLimitDisableNeutralOnLOS(bool softLimitDisableNeutralOnLOS, int timeoutMs);
    void ConfigPulseWidthPeriod_EdgesPerRot(int pulseWidthPeriod_EdgesPerRot, int timeoutMs);
    void ConfigPulseWidthPeriod_FilterWindowSz(int pulseWidthPeriod_FilterWindowSz, int timeoutMs);
    ctre::phoenix::ErrorCode GetLastError();
    void GetFirmwareVersion(int*);
    void HasResetOccurred(bool*);
    void ConfigSetCustomParam(int newValue, int paramIndex, int timeoutMs);
    void ConfigGetCustomParam(int* readValue, int paramIndex, int timoutMs);
    void ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal, int timeoutMs);
    void ConfigGetParameter(int param, double* value, int ordinal, int timeoutMs);
    void ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal, int32_t timeoutMs);
    void ConfigPeakCurrentLimit(int amps, int timeoutMs);
    void ConfigPeakCurrentDuration(int milliseconds, int timeoutMs);
    void ConfigContinuousCurrentLimit(int amps, int timeoutMs);
    void EnableCurrentLimit(bool enable);
    void SetLastError(int error);
    void GetAnalogIn(int* param);
    void SetAnalogPosition(int newPosition, int timeoutMs);
    void GetAnalogInRaw(int* param);
    void GetAnalogInVel(int* param);
    void GetQuadraturePosition(int* param);
    void SetQuadraturePosition(int newPosition, int timeoutMs);
    void GetQuadratureVelocity(int* param);
    void GetPulseWidthPosition(int* param);
    void SetPulseWidthPosition(int newPosition, int timeoutMs);
    void GetPulseWidthVelocity(int* param);
    void GetPulseWidthRiseToFallUs(int* param);
    void GetPulseWidthRiseToRiseUs(int* param);
    void GetPinStateQuadA(int* param);
    void GetPinStateQuadB(int* param);
    void GetPinStateQuadIdx(int* param);
    void IsFwdLimitSwitchClosed(int* param);
    void IsRevLimitSwitchClosed(int* param);
    void GetFaults(int* param);
    void GetStickyFaults(int* param);
    void ClearStickyFaults(int timeoutMs);
    void SelectDemandType(bool enable);
    void SetMPEOutput(int MpeOutput);
    void EnableHeadingHold(bool enable);
    void GetAnalogInAll(int* withOv, int* raw, int* vel);
    void GetQuadratureSensor(int* pos, int* vel);
    void GetPulseWidthAll(int* pos, int* vel, int* riseToRiseUs, int* riseToFallUs);
    void GetQuadPinStates(int* quadA, int* quadB, int* quadIdx);
    void GetLimitSwitchState(int* isFwdClosed, int* isRevClosed);
    void GetClosedLoopTarget(int* value, int pidIdx);
    void ConfigMotorCommutation(ctre::phoenix::motorcontrol::MotorCommutation motorCommutation, int timeoutMs);
    void ConfigGetMotorCommutation(ctre::phoenix::motorcontrol::MotorCommutation* motorCommutation, int timeoutMs);
    void ConfigSupplyCurrentLimit(const double* params, int paramCnt, int timeoutMs);
    void ConfigStatorCurrentLimit(const double* params, int paramCnt, int timeoutMs);
    void ConfigSupplyCurrentLimitEnable(bool enable, int timeoutMs);
    void ConfigStatorCurrentLimitEnable(bool enable, int timeoutMs);
    void ConfigGetSupplyCurrentLimit(double* toFill, int* fillCnt, int fillCapacity, int timeoutMs);
    void ConfigGetStatorCurrentLimit(double* toFill, int* fillCnt, int fillCapacity, int timeoutMs);
    void SetIntegratedSensorPosition(double newpos, int timeoutMs);
    void SetIntegratedSensorPositionToAbsolute(int timeoutMs);
    void GetIntegratedSensor(double* pos, double* absPos, double* vel);
    void ConfigIntegratedSensorAbsoluteRange(ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange, int timeoutMs);
    void ConfigIntegratedSensorOffset(double offsetDegrees, int timeoutMs);
    void ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy, int timeoutMs);
};

} // namespace SnobotSim
