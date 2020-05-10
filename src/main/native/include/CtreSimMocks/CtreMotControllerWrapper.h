
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
    void ConfigFactoryDefault();
    void ConfigOpenLoopRamp(double secondsFromNeutralToFull);
    void ConfigClosedLoopRamp(double secondsFromNeutralToFull);
    void ConfigPeakOutputForward(double percentOut);
    void ConfigPeakOutputReverse(double percentOut);
    void ConfigNominalOutputForward(double percentOut);
    void ConfigNominalOutputReverse(double percentOut);
    void ConfigNeutralDeadband(double percentDeadband);
    void ConfigVoltageCompSaturation(double voltage);
    void ConfigVoltageMeasurementFilter(int filterWindowSamples);
    void EnableVoltageCompensation(bool enable);
    void GetInverted(bool* invert);
    void GetBusVoltage(double* voltage);
    void GetMotorOutputPercent(double* percentOutput);
    void GetOutputCurrent(double* current);
    void GetSupplyCurrent(double* current);
    void GetStatorCurrent(double* current);
    void GetTemperature(double* temperature);
    void ConfigSelectedFeedbackSensor(int feedbackDevice, int pidIdx);
    void ConfigSelectedFeedbackCoefficient(double coefficient, int pidIdx);
    void ConfigRemoteFeedbackFilter(int deviceID, int remoteSensorSource, int remoteOrdinal);
    void ConfigSensorTerm(int sensorTerm, int feedbackDevice);
    void GetSelectedSensorPosition(int* param, int pidIdx);
    void GetSelectedSensorVelocity(int* param, int pidIdx);
    void SetSelectedSensorPosition(int sensorPos, int pidIdx);
    void SetControlFramePeriod(int frame, int periodMs);
    void SetStatusFramePeriod(int frame, uint8_t periodMs);
    void GetStatusFramePeriod(int frame, int* periodMs);
    void ConfigVelocityMeasurementPeriod(int period);
    void ConfigVelocityMeasurementWindow(int windowSize);
    void ConfigForwardLimitSwitchSource(int type, int normalOpenOrClose, int deviceID);
    void ConfigReverseLimitSwitchSource(int type, int normalOpenOrClose, int deviceID);
    void OverrideLimitSwitchesEnable(bool enable);
    void ConfigForwardSoftLimitThreshold(int forwardSensorLimit);
    void ConfigReverseSoftLimitThreshold(int reverseSensorLimit);
    void ConfigForwardSoftLimitEnable(bool enable);
    void ConfigReverseSoftLimitEnable(bool enable);
    void OverrideSoftLimitsEnable(bool enable);
    void Config_kP(int slotIdx, double value);
    void Config_kI(int slotIdx, double value);
    void Config_kD(int slotIdx, double value);
    void Config_kF(int slotIdx, double value);
    void Config_IntegralZone(int slotIdx, double izone);
    void ConfigAllowableClosedloopError(int slotIdx, int allowableClosedLoopError);
    void ConfigMaxIntegralAccumulator(int slotIdx, double iaccum);
    void ConfigClosedLoopPeakOutput(int slotIdx, double percentOut);
    void ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs);
    void SetIntegralAccumulator(double iaccum, int pidIdx);
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
    void ConfigMotionCruiseVelocity(int sensorUnitsPer100ms);
    void ConfigMotionAcceleration(int sensorUnitsPer100msPerSec);
    void ConfigMotionSCurveStrength(int curveStrength);
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
    void ClearMotionProfileHasUnderrun();
    void ChangeMotionControlFramePeriod(int periodMs);
    void ConfigMotionProfileTrajectoryPeriod(int durationMs);
    void ConfigMotionProfileTrajectoryInterpolationEnable(bool enable);
    void ConfigFeedbackNotContinuous(bool feedbackNotContinuous);
    void ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(bool remoteSensorClosedLoopDisableNeutralOnLOS);
    void ConfigClearPositionOnLimitF(bool clearPositionOnLimitF);
    void ConfigClearPositionOnLimitR(bool clearPositionOnLimitR);
    void ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx);
    void ConfigLimitSwitchDisableNeutralOnLOS(bool limitSwitchDisableNeutralOnLOS);
    void ConfigSoftLimitDisableNeutralOnLOS(bool softLimitDisableNeutralOnLOS);
    void ConfigPulseWidthPeriod_EdgesPerRot(int pulseWidthPeriod_EdgesPerRot);
    void ConfigPulseWidthPeriod_FilterWindowSz(int pulseWidthPeriod_FilterWindowSz);
    ctre::phoenix::ErrorCode GetLastError();
    void GetFirmwareVersion(int*);
    void HasResetOccurred(bool*);
    void ConfigSetCustomParam(int newValue, int paramIndex);
    void ConfigGetCustomParam(int* readValue, int paramIndex, int timoutMs);
    void ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal);
    void ConfigGetParameter(int param, double* value, int ordinal);
    void ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal);
    void ConfigPeakCurrentLimit(int amps);
    void ConfigPeakCurrentDuration(int milliseconds);
    void ConfigContinuousCurrentLimit(int amps);
    void EnableCurrentLimit(bool enable);
    void SetLastError(int error);
    void GetAnalogIn(int* param);
    void SetAnalogPosition(int newPosition);
    void GetAnalogInRaw(int* param);
    void GetAnalogInVel(int* param);
    void GetQuadraturePosition(int* param);
    void SetQuadraturePosition(int newPosition);
    void GetQuadratureVelocity(int* param);
    void GetPulseWidthPosition(int* param);
    void SetPulseWidthPosition(int newPosition);
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
    void ClearStickyFaults();
    void SelectDemandType(bool enable);
    void SetMPEOutput(int MpeOutput);
    void EnableHeadingHold(bool enable);
    void GetAnalogInAll(int* withOv, int* raw, int* vel);
    void GetQuadratureSensor(int* pos, int* vel);
    void GetPulseWidthAll(int* pos, int* vel, int* riseToRiseUs, int* riseToFallUs);
    void GetQuadPinStates(int* quadA, int* quadB, int* quadIdx);
    void GetLimitSwitchState(int* isFwdClosed, int* isRevClosed);
    void GetClosedLoopTarget(int* value, int pidIdx);
    void ConfigMotorCommutation(ctre::phoenix::motorcontrol::MotorCommutation motorCommutation);
    void ConfigGetMotorCommutation(ctre::phoenix::motorcontrol::MotorCommutation* motorCommutation);
    void ConfigSupplyCurrentLimit(const double* params, int paramCnt);
    void ConfigStatorCurrentLimit(const double* params, int paramCnt);
    void ConfigSupplyCurrentLimitEnable(bool enable);
    void ConfigStatorCurrentLimitEnable(bool enable);
    void ConfigGetSupplyCurrentLimit(double* toFill, int* fillCnt, int fillCapacity);
    void ConfigGetStatorCurrentLimit(double* toFill, int* fillCnt, int fillCapacity);
    void SetIntegratedSensorPosition(double newpos);
    void SetIntegratedSensorPositionToAbsolute();
    void GetIntegratedSensor(double* pos, double* absPos, double* vel);
    void ConfigIntegratedSensorAbsoluteRange(ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange);
    void ConfigIntegratedSensorOffset(double offsetDegrees);
    void ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy);
};

} // namespace SnobotSim
