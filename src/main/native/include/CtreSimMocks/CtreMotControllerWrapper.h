
#pragma once

#include <string>

#include "CtreSimUtils/BaseCtreWrapper.h"
#include "ctre/phoenix/cci/MotController_CCI.h"
#include "simulation/SimDeviceSim.h"

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

    ctre::phoenix::ErrorCode GetLastError();

    //////////////////////////////////////////
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
    void PushMotionProfileTrajectory_2(double position, double velocity, double headingDeg, int profileSlotSelect0, int profileSlotSelect1, bool isLastPoint, bool zeroPos, int durationMs);
    void PushMotionProfileTrajectory_3(double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos0, uint32_t timeDur, bool useAuxPID);
    void StartMotionProfile(void* streamHandle, uint32_t minBufferedPts, ctre::phoenix::motorcontrol::ControlMode controlMode);
    void IsMotionProfileFinished(bool* value);
    void IsMotionProfileTopLevelBufferFull(bool* value);
    void ProcessMotionProfileBuffer();
    void GetMotionProfileStatus(size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt, bool* hasUnderrun, bool* isUnderrun, bool* activePointValid, bool* isLast, int* profileSlotSelect, int* outputEnable);
    void GetMotionProfileStatus_2(size_t* topBufferRem, size_t* topBufferCnt, int* btmBufferCnt, bool* hasUnderrun, bool* isUnderrun, bool* activePointValid, bool* isLast, int* profileSlotSelect, int* outputEnable, int* timeDurMs, int* profileSlotSelect1);
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
    void GetFirmwareVersion(int* version);
    void HasResetOccurred(bool* output);
    void ConfigSetCustomParam(int newValue, int paramIndex);
    void ConfigGetCustomParam(int* readValue, int paramIndex);
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

protected:
    hal::SimDevice m_simDevice;

    hal::SimDouble m_ActiveTrajectoryAll_heading;
    hal::SimDouble m_ActiveTrajectoryAll_pos;
    hal::SimDouble m_ActiveTrajectoryAll_vel;
    hal::SimDouble m_ActiveTrajectoryHeading_param;
    hal::SimDouble m_ActiveTrajectoryPosition_param;
    hal::SimDouble m_ActiveTrajectoryVelocity_param;
    hal::SimDouble m_AnalogInAll_raw;
    hal::SimDouble m_AnalogInAll_vel;
    hal::SimDouble m_AnalogInAll_withOv;
    hal::SimDouble m_AnalogInRaw_param;
    hal::SimDouble m_AnalogInVel_param;
    hal::SimDouble m_AnalogIn_param;
    hal::SimDouble m_AnalogPosition_newPosition;
    hal::SimDouble m_BaseID_baseArbId;
    hal::SimDouble m_BusVoltage_voltage;
    hal::SimDouble m_ChangeMotionControlFramePeriod_periodMs;
    hal::SimDouble m_ConfigClearPositionOnLimitF_clearPositionOnLimitF;
    hal::SimDouble m_ConfigClearPositionOnLimitR_clearPositionOnLimitR;
    hal::SimDouble m_ConfigClearPositionOnQuadIdx_clearPositionOnQuadIdx;
    hal::SimDouble m_ConfigClosedLoopRamp_secondsFromNeutralToFull;
    hal::SimDouble m_ConfigContinuousCurrentLimit_amps;
    hal::SimDouble m_ConfigFeedbackNotContinuous_feedbackNotContinuous;
    hal::SimDouble m_ConfigForwardLimitSwitchSource_deviceID;
    hal::SimDouble m_ConfigForwardLimitSwitchSource_normalOpenOrClose;
    hal::SimDouble m_ConfigForwardLimitSwitchSource_type;
    hal::SimDouble m_ConfigForwardSoftLimitEnable_enable;
    hal::SimDouble m_ConfigForwardSoftLimitThreshold_forwardSensorLimit;
    hal::SimDouble m_ConfigGetCustomParam_paramIndex;
    hal::SimDouble m_ConfigGetCustomParam_readValue;
    hal::SimDouble m_ConfigGetMotorCommutation_motorCommutation;
    hal::SimDouble m_ConfigGetParameter_6_ordinal;
    hal::SimDouble m_ConfigGetParameter_6_param;
    hal::SimDouble m_ConfigGetParameter_6_subValue;
    hal::SimDouble m_ConfigGetParameter_6_valueRecieved;
    hal::SimDouble m_ConfigGetParameter_6_valueToSend;
    hal::SimDouble m_ConfigGetParameter_ordinal;
    hal::SimDouble m_ConfigGetParameter_param;
    hal::SimDouble m_ConfigGetParameter_value;
    hal::SimDouble m_ConfigGetStatorCurrentLimit_fillCapacity;
    hal::SimDouble m_ConfigGetStatorCurrentLimit_fillCnt;
    hal::SimDouble m_ConfigGetStatorCurrentLimit_toFill;
    hal::SimDouble m_ConfigGetSupplyCurrentLimit_fillCapacity;
    hal::SimDouble m_ConfigGetSupplyCurrentLimit_fillCnt;
    hal::SimDouble m_ConfigGetSupplyCurrentLimit_toFill;
    hal::SimDouble m_ConfigIntegratedSensorAbsoluteRange_absoluteSensorRange;
    hal::SimDouble m_ConfigIntegratedSensorInitializationStrategy_initializationStrategy;
    hal::SimDouble m_ConfigIntegratedSensorOffset_offsetDegrees;
    hal::SimDouble m_ConfigLimitSwitchDisableNeutralOnLOS_limitSwitchDisableNeutralOnLOS;
    hal::SimDouble m_ConfigMotionAcceleration_sensorUnitsPer100msPerSec;
    hal::SimDouble m_ConfigMotionCruiseVelocity_sensorUnitsPer100ms;
    hal::SimDouble m_ConfigMotionProfileTrajectoryInterpolationEnable_enable;
    hal::SimDouble m_ConfigMotionProfileTrajectoryPeriod_durationMs;
    hal::SimDouble m_ConfigMotionSCurveStrength_curveStrength;
    hal::SimDouble m_ConfigMotorCommutation_motorCommutation;
    hal::SimDouble m_ConfigNeutralDeadband_percentDeadband;
    hal::SimDouble m_ConfigNominalOutputForward_percentOut;
    hal::SimDouble m_ConfigNominalOutputReverse_percentOut;
    hal::SimDouble m_ConfigOpenLoopRamp_secondsFromNeutralToFull;
    hal::SimDouble m_ConfigPeakCurrentDuration_milliseconds;
    hal::SimDouble m_ConfigPeakCurrentLimit_amps;
    hal::SimDouble m_ConfigPeakOutputForward_percentOut;
    hal::SimDouble m_ConfigPeakOutputReverse_percentOut;
    hal::SimDouble m_ConfigPulseWidthPeriod_EdgesPerRot_pulseWidthPeriod_EdgesPerRot;
    hal::SimDouble m_ConfigPulseWidthPeriod_FilterWindowSz_pulseWidthPeriod_FilterWindowSz;
    hal::SimDouble m_ConfigRemoteFeedbackFilter_deviceID;
    hal::SimDouble m_ConfigRemoteFeedbackFilter_remoteOrdinal;
    hal::SimDouble m_ConfigRemoteFeedbackFilter_remoteSensorSource;
    hal::SimDouble m_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS_remoteSensorClosedLoopDisableNeutralOnLOS;
    hal::SimDouble m_ConfigReverseLimitSwitchSource_deviceID;
    hal::SimDouble m_ConfigReverseLimitSwitchSource_normalOpenOrClose;
    hal::SimDouble m_ConfigReverseLimitSwitchSource_type;
    hal::SimDouble m_ConfigReverseSoftLimitEnable_enable;
    hal::SimDouble m_ConfigReverseSoftLimitThreshold_reverseSensorLimit;
    hal::SimDouble m_ConfigSensorTerm_feedbackDevice;
    hal::SimDouble m_ConfigSensorTerm_sensorTerm;
    hal::SimDouble m_ConfigSetCustomParam_newValue;
    hal::SimDouble m_ConfigSetCustomParam_paramIndex;
    hal::SimDouble m_ConfigSetParameter_ordinal;
    hal::SimDouble m_ConfigSetParameter_param;
    hal::SimDouble m_ConfigSetParameter_subValue;
    hal::SimDouble m_ConfigSetParameter_value;
    hal::SimDouble m_ConfigSoftLimitDisableNeutralOnLOS_softLimitDisableNeutralOnLOS;
    hal::SimDouble m_ConfigStatorCurrentLimitEnable_enable;
    hal::SimDouble m_ConfigStatorCurrentLimit_paramCnt;
    hal::SimDouble m_ConfigStatorCurrentLimit_params;
    hal::SimDouble m_ConfigSupplyCurrentLimitEnable_enable;
    hal::SimDouble m_ConfigSupplyCurrentLimit_paramCnt;
    hal::SimDouble m_ConfigSupplyCurrentLimit_params;
    hal::SimDouble m_ConfigVelocityMeasurementPeriod_period;
    hal::SimDouble m_ConfigVelocityMeasurementWindow_windowSize;
    hal::SimDouble m_ConfigVoltageCompSaturation_voltage;
    hal::SimDouble m_ConfigVoltageMeasurementFilter_filterWindowSamples;
    hal::SimDouble m_ControlFramePeriod_frame;
    hal::SimDouble m_ControlFramePeriod_periodMs;
    hal::SimDouble m_Create1_baseArbId;
    hal::SimDouble m_Create2_deviceID;
    hal::SimDouble m_Create2_model;
    hal::SimDouble m_Demand_demand0;
    hal::SimDouble m_Demand_demand1;
    hal::SimDouble m_Demand_mode;
    hal::SimDouble m_Description_numBytesFilled;
    hal::SimDouble m_Description_toFill;
    hal::SimDouble m_Description_toFillByteSz;
    hal::SimDouble m_DeviceNumber_deviceNumber;
    hal::SimDouble m_EnableCurrentLimit_enable;
    hal::SimDouble m_EnableHeadingHold_enable;
    hal::SimDouble m_EnableVoltageCompensation_enable;
    hal::SimDouble m_Faults_param;
    hal::SimDouble m_FirmwareVersion_version;
    hal::SimDouble m_FwdLimitSwitchClosed_param;
    hal::SimDouble m_HasResetOccurred_output;
    hal::SimDouble m_IntegratedSensorPosition_newpos;
    hal::SimDouble m_IntegratedSensor_absPos;
    hal::SimDouble m_IntegratedSensor_pos;
    hal::SimDouble m_IntegratedSensor_vel;
    hal::SimDouble m_Inverted_2_invertType;
    hal::SimDouble m_Inverted_invert;
    hal::SimDouble m_LastError_error;
    hal::SimDouble m_LimitSwitchState_isFwdClosed;
    hal::SimDouble m_LimitSwitchState_isRevClosed;
    hal::SimDouble m_MPEOutput_MpeOutput;
    hal::SimDouble m_MotionProfileFinished_value;
    hal::SimDouble m_MotionProfileStatus_2_activePointValid;
    hal::SimDouble m_MotionProfileStatus_2_btmBufferCnt;
    hal::SimDouble m_MotionProfileStatus_2_hasUnderrun;
    hal::SimDouble m_MotionProfileStatus_2_isLast;
    hal::SimDouble m_MotionProfileStatus_2_isUnderrun;
    hal::SimDouble m_MotionProfileStatus_2_outputEnable;
    hal::SimDouble m_MotionProfileStatus_2_profileSlotSelect;
    hal::SimDouble m_MotionProfileStatus_2_profileSlotSelect1;
    hal::SimDouble m_MotionProfileStatus_2_timeDurMs;
    hal::SimDouble m_MotionProfileStatus_2_topBufferCnt;
    hal::SimDouble m_MotionProfileStatus_2_topBufferRem;
    hal::SimDouble m_MotionProfileStatus_activePointValid;
    hal::SimDouble m_MotionProfileStatus_btmBufferCnt;
    hal::SimDouble m_MotionProfileStatus_hasUnderrun;
    hal::SimDouble m_MotionProfileStatus_isLast;
    hal::SimDouble m_MotionProfileStatus_isUnderrun;
    hal::SimDouble m_MotionProfileStatus_outputEnable;
    hal::SimDouble m_MotionProfileStatus_profileSlotSelect;
    hal::SimDouble m_MotionProfileStatus_topBufferCnt;
    hal::SimDouble m_MotionProfileStatus_topBufferRem;
    hal::SimDouble m_MotionProfileTopLevelBufferCount_value;
    hal::SimDouble m_MotionProfileTopLevelBufferFull_value;
    hal::SimDouble m_MotorOutputPercent_percentOutput;
    hal::SimDouble m_NeutralMode_neutralMode;
    hal::SimDouble m_OutputCurrent_current;
    hal::SimDouble m_OverrideLimitSwitchesEnable_enable;
    hal::SimDouble m_OverrideSoftLimitsEnable_enable;
    hal::SimDouble m_PinStateQuadA_param;
    hal::SimDouble m_PinStateQuadB_param;
    hal::SimDouble m_PinStateQuadIdx_param;
    hal::SimDouble m_PulseWidthAll_pos;
    hal::SimDouble m_PulseWidthAll_riseToFallUs;
    hal::SimDouble m_PulseWidthAll_riseToRiseUs;
    hal::SimDouble m_PulseWidthAll_vel;
    hal::SimDouble m_PulseWidthPosition_newPosition;
    hal::SimDouble m_PulseWidthPosition_param;
    hal::SimDouble m_PulseWidthRiseToFallUs_param;
    hal::SimDouble m_PulseWidthRiseToRiseUs_param;
    hal::SimDouble m_PulseWidthVelocity_param;
    hal::SimDouble m_PushMotionProfileTrajectory_2_durationMs;
    hal::SimDouble m_PushMotionProfileTrajectory_2_headingDeg;
    hal::SimDouble m_PushMotionProfileTrajectory_2_isLastPoint;
    hal::SimDouble m_PushMotionProfileTrajectory_2_position;
    hal::SimDouble m_PushMotionProfileTrajectory_2_profileSlotSelect0;
    hal::SimDouble m_PushMotionProfileTrajectory_2_profileSlotSelect1;
    hal::SimDouble m_PushMotionProfileTrajectory_2_velocity;
    hal::SimDouble m_PushMotionProfileTrajectory_2_zeroPos;
    hal::SimDouble m_PushMotionProfileTrajectory_3_arbFeedFwd;
    hal::SimDouble m_PushMotionProfileTrajectory_3_auxiliaryArbFeedFwd;
    hal::SimDouble m_PushMotionProfileTrajectory_3_auxiliaryPos;
    hal::SimDouble m_PushMotionProfileTrajectory_3_auxiliaryVel;
    hal::SimDouble m_PushMotionProfileTrajectory_3_isLastPoint;
    hal::SimDouble m_PushMotionProfileTrajectory_3_position;
    hal::SimDouble m_PushMotionProfileTrajectory_3_profileSlotSelect0;
    hal::SimDouble m_PushMotionProfileTrajectory_3_profileSlotSelect1;
    hal::SimDouble m_PushMotionProfileTrajectory_3_timeDur;
    hal::SimDouble m_PushMotionProfileTrajectory_3_useAuxPID;
    hal::SimDouble m_PushMotionProfileTrajectory_3_velocity;
    hal::SimDouble m_PushMotionProfileTrajectory_3_zeroPos0;
    hal::SimDouble m_PushMotionProfileTrajectory_headingDeg;
    hal::SimDouble m_PushMotionProfileTrajectory_isLastPoint;
    hal::SimDouble m_PushMotionProfileTrajectory_position;
    hal::SimDouble m_PushMotionProfileTrajectory_profileSlotSelect;
    hal::SimDouble m_PushMotionProfileTrajectory_velocity;
    hal::SimDouble m_PushMotionProfileTrajectory_zeroPos;
    hal::SimDouble m_QuadPinStates_quadA;
    hal::SimDouble m_QuadPinStates_quadB;
    hal::SimDouble m_QuadPinStates_quadIdx;
    hal::SimDouble m_QuadraturePosition_newPosition;
    hal::SimDouble m_QuadraturePosition_param;
    hal::SimDouble m_QuadratureSensor_pos;
    hal::SimDouble m_QuadratureSensor_vel;
    hal::SimDouble m_QuadratureVelocity_param;
    hal::SimDouble m_RevLimitSwitchClosed_param;
    hal::SimDouble m_SelectDemandType_enable;
    hal::SimDouble m_SensorPhase_PhaseSensor;
    hal::SimDouble m_StartMotionProfile_controlMode;
    hal::SimDouble m_StartMotionProfile_minBufferedPts;
    hal::SimDouble m_StartMotionProfile_streamHandle;
    hal::SimDouble m_StatorCurrent_current;
    hal::SimDouble m_StatusFramePeriod_frame;
    hal::SimDouble m_StatusFramePeriod_periodMs;
    hal::SimDouble m_StickyFaults_param;
    hal::SimDouble m_SupplyCurrent_current;
    hal::SimDouble m_Temperature_temperature;
    hal::SimDouble m__4_demand0;
    hal::SimDouble m__4_demand1;
    hal::SimDouble m__4_demand1Type;
    hal::SimDouble m__4_mode;

    struct SlottedVariables
    {

        hal::SimDouble m_ActiveTrajectoryAll_5_arbFeedFwd;
        hal::SimDouble m_ActiveTrajectoryAll_5_pos;
        hal::SimDouble m_ActiveTrajectoryAll_5_vel;
        hal::SimDouble m_ActiveTrajectoryArbFeedFwd_3_param;
        hal::SimDouble m_ActiveTrajectoryPosition_3_param;
        hal::SimDouble m_ActiveTrajectoryVelocity_3_param;
        hal::SimDouble m_ClosedLoopError_closedLoopError;
        hal::SimDouble m_ClosedLoopTarget_value;
        hal::SimDouble m_ConfigAllowableClosedloopError_allowableClosedLoopError;
        hal::SimDouble m_ConfigClosedLoopPeakOutput_percentOut;
        hal::SimDouble m_ConfigClosedLoopPeriod_loopTimeMs;
        hal::SimDouble m_ConfigMaxIntegralAccumulator_iaccum;
        hal::SimDouble m_ConfigSelectedFeedbackCoefficient_coefficient;
        hal::SimDouble m_ConfigSelectedFeedbackSensor_feedbackDevice;
        hal::SimDouble m_Config_IntegralZone_izone;
        hal::SimDouble m_Config_kD_value;
        hal::SimDouble m_Config_kF_value;
        hal::SimDouble m_Config_kI_value;
        hal::SimDouble m_Config_kP_value;
        hal::SimDouble m_ErrorDerivative_derror;
        hal::SimDouble m_IntegralAccumulator_iaccum;
        hal::SimDouble m_SelectedSensorPosition_param;
        hal::SimDouble m_SelectedSensorPosition_sensorPos;
        hal::SimDouble m_SelectedSensorVelocity_param;
    };

    SlottedVariables m_slotted_variables[6];
};

} // namespace SnobotSim
