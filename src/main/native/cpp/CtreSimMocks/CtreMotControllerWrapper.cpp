

#include "CtreSimMocks/CtreMotControllerWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size) \
    uint8_t buffer[size]; /* NOLINT */  \
    std::memset(&buffer[0], 0, size);   \
    Receive(paramName, buffer, size);   \
    uint32_t buffer_pos = 0;

std::vector<SnobotSim::CTRE_CallbackFunc> gMotControllerCallbacks;

void SnobotSim::SetMotControllerCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gMotControllerCallbacks.clear();
    gMotControllerCallbacks.push_back(callback);
}

SnobotSim::CtreMotControllerWrapper::CtreMotControllerWrapper(int aDeviceId) :
        mDeviceId(aDeviceId & 0x3F),
                m_simDevice(std::string("CtreMotControllerWrapper " + std::to_string(aDeviceId)).c_str(), aDeviceId)
{

    m_ActiveTrajectoryAll_heading = m_simDevice.CreateDouble("ActiveTrajectoryAll_heading", false, 0);
    m_ActiveTrajectoryAll_pos = m_simDevice.CreateDouble("ActiveTrajectoryAll_pos", false, 0);
    m_ActiveTrajectoryAll_vel = m_simDevice.CreateDouble("ActiveTrajectoryAll_vel", false, 0);
    m_ActiveTrajectoryHeading_param = m_simDevice.CreateDouble("ActiveTrajectoryHeading_param", false, 0);
    m_ActiveTrajectoryPosition_param = m_simDevice.CreateDouble("ActiveTrajectoryPosition_param", false, 0);
    m_ActiveTrajectoryVelocity_param = m_simDevice.CreateDouble("ActiveTrajectoryVelocity_param", false, 0);
    m_AnalogInAll_raw = m_simDevice.CreateDouble("AnalogInAll_raw", false, 0);
    m_AnalogInAll_vel = m_simDevice.CreateDouble("AnalogInAll_vel", false, 0);
    m_AnalogInAll_withOv = m_simDevice.CreateDouble("AnalogInAll_withOv", false, 0);
    m_AnalogInRaw_param = m_simDevice.CreateDouble("AnalogInRaw_param", false, 0);
    m_AnalogInVel_param = m_simDevice.CreateDouble("AnalogInVel_param", false, 0);
    m_AnalogIn_param = m_simDevice.CreateDouble("AnalogIn_param", false, 0);
    m_AnalogPosition_newPosition = m_simDevice.CreateDouble("AnalogPosition_newPosition", false, 0);
    m_BaseID_baseArbId = m_simDevice.CreateDouble("BaseID_baseArbId", false, 0);
    m_BusVoltage_voltage = m_simDevice.CreateDouble("BusVoltage_voltage", false, 0);
    m_ChangeMotionControlFramePeriod_periodMs = m_simDevice.CreateDouble("ChangeMotionControlFramePeriod_periodMs", false, 0);
    m_ConfigClearPositionOnLimitF_clearPositionOnLimitF = m_simDevice.CreateDouble("ConfigClearPositionOnLimitF_clearPositionOnLimitF", false, 0);
    m_ConfigClearPositionOnLimitR_clearPositionOnLimitR = m_simDevice.CreateDouble("ConfigClearPositionOnLimitR_clearPositionOnLimitR", false, 0);
    m_ConfigClearPositionOnQuadIdx_clearPositionOnQuadIdx = m_simDevice.CreateDouble("ConfigClearPositionOnQuadIdx_clearPositionOnQuadIdx", false, 0);
    m_ConfigClosedLoopRamp_secondsFromNeutralToFull = m_simDevice.CreateDouble("ConfigClosedLoopRamp_secondsFromNeutralToFull", false, 0);
    m_ConfigContinuousCurrentLimit_amps = m_simDevice.CreateDouble("ConfigContinuousCurrentLimit_amps", false, 0);
    m_ConfigFeedbackNotContinuous_feedbackNotContinuous = m_simDevice.CreateDouble("ConfigFeedbackNotContinuous_feedbackNotContinuous", false, 0);
    m_ConfigForwardLimitSwitchSource_deviceID = m_simDevice.CreateDouble("ConfigForwardLimitSwitchSource_deviceID", false, 0);
    m_ConfigForwardLimitSwitchSource_normalOpenOrClose = m_simDevice.CreateDouble("ConfigForwardLimitSwitchSource_normalOpenOrClose", false, 0);
    m_ConfigForwardLimitSwitchSource_type = m_simDevice.CreateDouble("ConfigForwardLimitSwitchSource_type", false, 0);
    m_ConfigForwardSoftLimitEnable_enable = m_simDevice.CreateDouble("ConfigForwardSoftLimitEnable_enable", false, 0);
    m_ConfigForwardSoftLimitThreshold_forwardSensorLimit = m_simDevice.CreateDouble("ConfigForwardSoftLimitThreshold_forwardSensorLimit", false, 0);
    m_ConfigGetCustomParam_paramIndex = m_simDevice.CreateDouble("ConfigGetCustomParam_paramIndex", false, 0);
    m_ConfigGetCustomParam_readValue = m_simDevice.CreateDouble("ConfigGetCustomParam_readValue", false, 0);
    m_ConfigGetMotorCommutation_motorCommutation = m_simDevice.CreateDouble("ConfigGetMotorCommutation_motorCommutation", false, 0);
    m_ConfigGetParameter_6_ordinal = m_simDevice.CreateDouble("ConfigGetParameter_6_ordinal", false, 0);
    m_ConfigGetParameter_6_param = m_simDevice.CreateDouble("ConfigGetParameter_6_param", false, 0);
    m_ConfigGetParameter_6_subValue = m_simDevice.CreateDouble("ConfigGetParameter_6_subValue", false, 0);
    m_ConfigGetParameter_6_valueRecieved = m_simDevice.CreateDouble("ConfigGetParameter_6_valueRecieved", false, 0);
    m_ConfigGetParameter_6_valueToSend = m_simDevice.CreateDouble("ConfigGetParameter_6_valueToSend", false, 0);
    m_ConfigGetParameter_ordinal = m_simDevice.CreateDouble("ConfigGetParameter_ordinal", false, 0);
    m_ConfigGetParameter_param = m_simDevice.CreateDouble("ConfigGetParameter_param", false, 0);
    m_ConfigGetParameter_value = m_simDevice.CreateDouble("ConfigGetParameter_value", false, 0);
    m_ConfigGetStatorCurrentLimit_fillCapacity = m_simDevice.CreateDouble("ConfigGetStatorCurrentLimit_fillCapacity", false, 0);
    m_ConfigGetStatorCurrentLimit_fillCnt = m_simDevice.CreateDouble("ConfigGetStatorCurrentLimit_fillCnt", false, 0);
    m_ConfigGetStatorCurrentLimit_toFill = m_simDevice.CreateDouble("ConfigGetStatorCurrentLimit_toFill", false, 0);
    m_ConfigGetSupplyCurrentLimit_fillCapacity = m_simDevice.CreateDouble("ConfigGetSupplyCurrentLimit_fillCapacity", false, 0);
    m_ConfigGetSupplyCurrentLimit_fillCnt = m_simDevice.CreateDouble("ConfigGetSupplyCurrentLimit_fillCnt", false, 0);
    m_ConfigGetSupplyCurrentLimit_toFill = m_simDevice.CreateDouble("ConfigGetSupplyCurrentLimit_toFill", false, 0);
    m_ConfigIntegratedSensorAbsoluteRange_absoluteSensorRange = m_simDevice.CreateDouble("ConfigIntegratedSensorAbsoluteRange_absoluteSensorRange", false, 0);
    m_ConfigIntegratedSensorInitializationStrategy_initializationStrategy = m_simDevice.CreateDouble("ConfigIntegratedSensorInitializationStrategy_initializationStrategy", false, 0);
    m_ConfigIntegratedSensorOffset_offsetDegrees = m_simDevice.CreateDouble("ConfigIntegratedSensorOffset_offsetDegrees", false, 0);
    m_ConfigLimitSwitchDisableNeutralOnLOS_limitSwitchDisableNeutralOnLOS = m_simDevice.CreateDouble("ConfigLimitSwitchDisableNeutralOnLOS_limitSwitchDisableNeutralOnLOS", false, 0);
    m_ConfigMotionAcceleration_sensorUnitsPer100msPerSec = m_simDevice.CreateDouble("ConfigMotionAcceleration_sensorUnitsPer100msPerSec", false, 0);
    m_ConfigMotionCruiseVelocity_sensorUnitsPer100ms = m_simDevice.CreateDouble("ConfigMotionCruiseVelocity_sensorUnitsPer100ms", false, 0);
    m_ConfigMotionProfileTrajectoryInterpolationEnable_enable = m_simDevice.CreateDouble("ConfigMotionProfileTrajectoryInterpolationEnable_enable", false, 0);
    m_ConfigMotionProfileTrajectoryPeriod_durationMs = m_simDevice.CreateDouble("ConfigMotionProfileTrajectoryPeriod_durationMs", false, 0);
    m_ConfigMotionSCurveStrength_curveStrength = m_simDevice.CreateDouble("ConfigMotionSCurveStrength_curveStrength", false, 0);
    m_ConfigMotorCommutation_motorCommutation = m_simDevice.CreateDouble("ConfigMotorCommutation_motorCommutation", false, 0);
    m_ConfigNeutralDeadband_percentDeadband = m_simDevice.CreateDouble("ConfigNeutralDeadband_percentDeadband", false, 0);
    m_ConfigNominalOutputForward_percentOut = m_simDevice.CreateDouble("ConfigNominalOutputForward_percentOut", false, 0);
    m_ConfigNominalOutputReverse_percentOut = m_simDevice.CreateDouble("ConfigNominalOutputReverse_percentOut", false, 0);
    m_ConfigOpenLoopRamp_secondsFromNeutralToFull = m_simDevice.CreateDouble("ConfigOpenLoopRamp_secondsFromNeutralToFull", false, 0);
    m_ConfigPeakCurrentDuration_milliseconds = m_simDevice.CreateDouble("ConfigPeakCurrentDuration_milliseconds", false, 0);
    m_ConfigPeakCurrentLimit_amps = m_simDevice.CreateDouble("ConfigPeakCurrentLimit_amps", false, 0);
    m_ConfigPeakOutputForward_percentOut = m_simDevice.CreateDouble("ConfigPeakOutputForward_percentOut", false, 0);
    m_ConfigPeakOutputReverse_percentOut = m_simDevice.CreateDouble("ConfigPeakOutputReverse_percentOut", false, 0);
    m_ConfigPulseWidthPeriod_EdgesPerRot_pulseWidthPeriod_EdgesPerRot = m_simDevice.CreateDouble("ConfigPulseWidthPeriod_EdgesPerRot_pulseWidthPeriod_EdgesPerRot", false, 0);
    m_ConfigPulseWidthPeriod_FilterWindowSz_pulseWidthPeriod_FilterWindowSz = m_simDevice.CreateDouble("ConfigPulseWidthPeriod_FilterWindowSz_pulseWidthPeriod_FilterWindowSz", false, 0);
    m_ConfigRemoteFeedbackFilter_deviceID = m_simDevice.CreateDouble("ConfigRemoteFeedbackFilter_deviceID", false, 0);
    m_ConfigRemoteFeedbackFilter_remoteOrdinal = m_simDevice.CreateDouble("ConfigRemoteFeedbackFilter_remoteOrdinal", false, 0);
    m_ConfigRemoteFeedbackFilter_remoteSensorSource = m_simDevice.CreateDouble("ConfigRemoteFeedbackFilter_remoteSensorSource", false, 0);
    m_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS_remoteSensorClosedLoopDisableNeutralOnLOS = m_simDevice.CreateDouble("ConfigRemoteSensorClosedLoopDisableNeutralOnLOS_remoteSensorClosedLoopDisableNeutralOnLOS", false, 0);
    m_ConfigReverseLimitSwitchSource_deviceID = m_simDevice.CreateDouble("ConfigReverseLimitSwitchSource_deviceID", false, 0);
    m_ConfigReverseLimitSwitchSource_normalOpenOrClose = m_simDevice.CreateDouble("ConfigReverseLimitSwitchSource_normalOpenOrClose", false, 0);
    m_ConfigReverseLimitSwitchSource_type = m_simDevice.CreateDouble("ConfigReverseLimitSwitchSource_type", false, 0);
    m_ConfigReverseSoftLimitEnable_enable = m_simDevice.CreateDouble("ConfigReverseSoftLimitEnable_enable", false, 0);
    m_ConfigReverseSoftLimitThreshold_reverseSensorLimit = m_simDevice.CreateDouble("ConfigReverseSoftLimitThreshold_reverseSensorLimit", false, 0);
    m_ConfigSensorTerm_feedbackDevice = m_simDevice.CreateDouble("ConfigSensorTerm_feedbackDevice", false, 0);
    m_ConfigSensorTerm_sensorTerm = m_simDevice.CreateDouble("ConfigSensorTerm_sensorTerm", false, 0);
    m_ConfigSetCustomParam_newValue = m_simDevice.CreateDouble("ConfigSetCustomParam_newValue", false, 0);
    m_ConfigSetCustomParam_paramIndex = m_simDevice.CreateDouble("ConfigSetCustomParam_paramIndex", false, 0);
    m_ConfigSetParameter_ordinal = m_simDevice.CreateDouble("ConfigSetParameter_ordinal", false, 0);
    m_ConfigSetParameter_param = m_simDevice.CreateDouble("ConfigSetParameter_param", false, 0);
    m_ConfigSetParameter_subValue = m_simDevice.CreateDouble("ConfigSetParameter_subValue", false, 0);
    m_ConfigSetParameter_value = m_simDevice.CreateDouble("ConfigSetParameter_value", false, 0);
    m_ConfigSoftLimitDisableNeutralOnLOS_softLimitDisableNeutralOnLOS = m_simDevice.CreateDouble("ConfigSoftLimitDisableNeutralOnLOS_softLimitDisableNeutralOnLOS", false, 0);
    m_ConfigStatorCurrentLimitEnable_enable = m_simDevice.CreateDouble("ConfigStatorCurrentLimitEnable_enable", false, 0);
    m_ConfigStatorCurrentLimit_paramCnt = m_simDevice.CreateDouble("ConfigStatorCurrentLimit_paramCnt", false, 0);
    m_ConfigStatorCurrentLimit_params = m_simDevice.CreateDouble("ConfigStatorCurrentLimit_params", false, 0);
    m_ConfigSupplyCurrentLimitEnable_enable = m_simDevice.CreateDouble("ConfigSupplyCurrentLimitEnable_enable", false, 0);
    m_ConfigSupplyCurrentLimit_paramCnt = m_simDevice.CreateDouble("ConfigSupplyCurrentLimit_paramCnt", false, 0);
    m_ConfigSupplyCurrentLimit_params = m_simDevice.CreateDouble("ConfigSupplyCurrentLimit_params", false, 0);
    m_ConfigVelocityMeasurementPeriod_period = m_simDevice.CreateDouble("ConfigVelocityMeasurementPeriod_period", false, 0);
    m_ConfigVelocityMeasurementWindow_windowSize = m_simDevice.CreateDouble("ConfigVelocityMeasurementWindow_windowSize", false, 0);
    m_ConfigVoltageCompSaturation_voltage = m_simDevice.CreateDouble("ConfigVoltageCompSaturation_voltage", false, 0);
    m_ConfigVoltageMeasurementFilter_filterWindowSamples = m_simDevice.CreateDouble("ConfigVoltageMeasurementFilter_filterWindowSamples", false, 0);
    m_ControlFramePeriod_frame = m_simDevice.CreateDouble("ControlFramePeriod_frame", false, 0);
    m_ControlFramePeriod_periodMs = m_simDevice.CreateDouble("ControlFramePeriod_periodMs", false, 0);
    m_Create1_baseArbId = m_simDevice.CreateDouble("Create1_baseArbId", false, 0);
    m_Create2_deviceID = m_simDevice.CreateDouble("Create2_deviceID", false, 0);
    m_Create2_model = m_simDevice.CreateDouble("Create2_model", false, 0);
    m_Demand_demand0 = m_simDevice.CreateDouble("Demand_demand0", false, 0);
    m_Demand_demand1 = m_simDevice.CreateDouble("Demand_demand1", false, 0);
    m_Demand_mode = m_simDevice.CreateDouble("Demand_mode", false, 0);
    m_Description_numBytesFilled = m_simDevice.CreateDouble("Description_numBytesFilled", false, 0);
    m_Description_toFill = m_simDevice.CreateDouble("Description_toFill", false, 0);
    m_Description_toFillByteSz = m_simDevice.CreateDouble("Description_toFillByteSz", false, 0);
    m_DeviceNumber_deviceNumber = m_simDevice.CreateDouble("DeviceNumber_deviceNumber", false, 0);
    m_EnableCurrentLimit_enable = m_simDevice.CreateDouble("EnableCurrentLimit_enable", false, 0);
    m_EnableHeadingHold_enable = m_simDevice.CreateDouble("EnableHeadingHold_enable", false, 0);
    m_EnableVoltageCompensation_enable = m_simDevice.CreateDouble("EnableVoltageCompensation_enable", false, 0);
    m_Faults_param = m_simDevice.CreateDouble("Faults_param", false, 0);
    m_FirmwareVersion_version = m_simDevice.CreateDouble("FirmwareVersion_version", false, 0);
    m_FwdLimitSwitchClosed_param = m_simDevice.CreateDouble("FwdLimitSwitchClosed_param", false, 0);
    m_HasResetOccurred_output = m_simDevice.CreateDouble("HasResetOccurred_output", false, 0);
    m_IntegratedSensorPosition_newpos = m_simDevice.CreateDouble("IntegratedSensorPosition_newpos", false, 0);
    m_IntegratedSensor_absPos = m_simDevice.CreateDouble("IntegratedSensor_absPos", false, 0);
    m_IntegratedSensor_pos = m_simDevice.CreateDouble("IntegratedSensor_pos", false, 0);
    m_IntegratedSensor_vel = m_simDevice.CreateDouble("IntegratedSensor_vel", false, 0);
    m_Inverted_2_invertType = m_simDevice.CreateDouble("Inverted_2_invertType", false, 0);
    m_Inverted_invert = m_simDevice.CreateDouble("Inverted_invert", false, 0);
    m_LastError_error = m_simDevice.CreateDouble("LastError_error", false, 0);
    m_LimitSwitchState_isFwdClosed = m_simDevice.CreateDouble("LimitSwitchState_isFwdClosed", false, 0);
    m_LimitSwitchState_isRevClosed = m_simDevice.CreateDouble("LimitSwitchState_isRevClosed", false, 0);
    m_MPEOutput_MpeOutput = m_simDevice.CreateDouble("MPEOutput_MpeOutput", false, 0);
    m_MotionProfileFinished_value = m_simDevice.CreateDouble("MotionProfileFinished_value", false, 0);
    m_MotionProfileStatus_2_activePointValid = m_simDevice.CreateDouble("MotionProfileStatus_2_activePointValid", false, 0);
    m_MotionProfileStatus_2_btmBufferCnt = m_simDevice.CreateDouble("MotionProfileStatus_2_btmBufferCnt", false, 0);
    m_MotionProfileStatus_2_hasUnderrun = m_simDevice.CreateDouble("MotionProfileStatus_2_hasUnderrun", false, 0);
    m_MotionProfileStatus_2_isLast = m_simDevice.CreateDouble("MotionProfileStatus_2_isLast", false, 0);
    m_MotionProfileStatus_2_isUnderrun = m_simDevice.CreateDouble("MotionProfileStatus_2_isUnderrun", false, 0);
    m_MotionProfileStatus_2_outputEnable = m_simDevice.CreateDouble("MotionProfileStatus_2_outputEnable", false, 0);
    m_MotionProfileStatus_2_profileSlotSelect = m_simDevice.CreateDouble("MotionProfileStatus_2_profileSlotSelect", false, 0);
    m_MotionProfileStatus_2_profileSlotSelect1 = m_simDevice.CreateDouble("MotionProfileStatus_2_profileSlotSelect1", false, 0);
    m_MotionProfileStatus_2_timeDurMs = m_simDevice.CreateDouble("MotionProfileStatus_2_timeDurMs", false, 0);
    m_MotionProfileStatus_2_topBufferCnt = m_simDevice.CreateDouble("MotionProfileStatus_2_topBufferCnt", false, 0);
    m_MotionProfileStatus_2_topBufferRem = m_simDevice.CreateDouble("MotionProfileStatus_2_topBufferRem", false, 0);
    m_MotionProfileStatus_activePointValid = m_simDevice.CreateDouble("MotionProfileStatus_activePointValid", false, 0);
    m_MotionProfileStatus_btmBufferCnt = m_simDevice.CreateDouble("MotionProfileStatus_btmBufferCnt", false, 0);
    m_MotionProfileStatus_hasUnderrun = m_simDevice.CreateDouble("MotionProfileStatus_hasUnderrun", false, 0);
    m_MotionProfileStatus_isLast = m_simDevice.CreateDouble("MotionProfileStatus_isLast", false, 0);
    m_MotionProfileStatus_isUnderrun = m_simDevice.CreateDouble("MotionProfileStatus_isUnderrun", false, 0);
    m_MotionProfileStatus_outputEnable = m_simDevice.CreateDouble("MotionProfileStatus_outputEnable", false, 0);
    m_MotionProfileStatus_profileSlotSelect = m_simDevice.CreateDouble("MotionProfileStatus_profileSlotSelect", false, 0);
    m_MotionProfileStatus_topBufferCnt = m_simDevice.CreateDouble("MotionProfileStatus_topBufferCnt", false, 0);
    m_MotionProfileStatus_topBufferRem = m_simDevice.CreateDouble("MotionProfileStatus_topBufferRem", false, 0);
    m_MotionProfileTopLevelBufferCount_value = m_simDevice.CreateDouble("MotionProfileTopLevelBufferCount_value", false, 0);
    m_MotionProfileTopLevelBufferFull_value = m_simDevice.CreateDouble("MotionProfileTopLevelBufferFull_value", false, 0);
    m_MotorOutputPercent_percentOutput = m_simDevice.CreateDouble("MotorOutputPercent_percentOutput", false, 0);
    m_NeutralMode_neutralMode = m_simDevice.CreateDouble("NeutralMode_neutralMode", false, 0);
    m_OutputCurrent_current = m_simDevice.CreateDouble("OutputCurrent_current", false, 0);
    m_OverrideLimitSwitchesEnable_enable = m_simDevice.CreateDouble("OverrideLimitSwitchesEnable_enable", false, 0);
    m_OverrideSoftLimitsEnable_enable = m_simDevice.CreateDouble("OverrideSoftLimitsEnable_enable", false, 0);
    m_PinStateQuadA_param = m_simDevice.CreateDouble("PinStateQuadA_param", false, 0);
    m_PinStateQuadB_param = m_simDevice.CreateDouble("PinStateQuadB_param", false, 0);
    m_PinStateQuadIdx_param = m_simDevice.CreateDouble("PinStateQuadIdx_param", false, 0);
    m_PulseWidthAll_pos = m_simDevice.CreateDouble("PulseWidthAll_pos", false, 0);
    m_PulseWidthAll_riseToFallUs = m_simDevice.CreateDouble("PulseWidthAll_riseToFallUs", false, 0);
    m_PulseWidthAll_riseToRiseUs = m_simDevice.CreateDouble("PulseWidthAll_riseToRiseUs", false, 0);
    m_PulseWidthAll_vel = m_simDevice.CreateDouble("PulseWidthAll_vel", false, 0);
    m_PulseWidthPosition_newPosition = m_simDevice.CreateDouble("PulseWidthPosition_newPosition", false, 0);
    m_PulseWidthPosition_param = m_simDevice.CreateDouble("PulseWidthPosition_param", false, 0);
    m_PulseWidthRiseToFallUs_param = m_simDevice.CreateDouble("PulseWidthRiseToFallUs_param", false, 0);
    m_PulseWidthRiseToRiseUs_param = m_simDevice.CreateDouble("PulseWidthRiseToRiseUs_param", false, 0);
    m_PulseWidthVelocity_param = m_simDevice.CreateDouble("PulseWidthVelocity_param", false, 0);
    m_PushMotionProfileTrajectory_2_durationMs = m_simDevice.CreateDouble("PushMotionProfileTrajectory_2_durationMs", false, 0);
    m_PushMotionProfileTrajectory_2_headingDeg = m_simDevice.CreateDouble("PushMotionProfileTrajectory_2_headingDeg", false, 0);
    m_PushMotionProfileTrajectory_2_isLastPoint = m_simDevice.CreateDouble("PushMotionProfileTrajectory_2_isLastPoint", false, 0);
    m_PushMotionProfileTrajectory_2_position = m_simDevice.CreateDouble("PushMotionProfileTrajectory_2_position", false, 0);
    m_PushMotionProfileTrajectory_2_profileSlotSelect0 = m_simDevice.CreateDouble("PushMotionProfileTrajectory_2_profileSlotSelect0", false, 0);
    m_PushMotionProfileTrajectory_2_profileSlotSelect1 = m_simDevice.CreateDouble("PushMotionProfileTrajectory_2_profileSlotSelect1", false, 0);
    m_PushMotionProfileTrajectory_2_velocity = m_simDevice.CreateDouble("PushMotionProfileTrajectory_2_velocity", false, 0);
    m_PushMotionProfileTrajectory_2_zeroPos = m_simDevice.CreateDouble("PushMotionProfileTrajectory_2_zeroPos", false, 0);
    m_PushMotionProfileTrajectory_3_arbFeedFwd = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_arbFeedFwd", false, 0);
    m_PushMotionProfileTrajectory_3_auxiliaryArbFeedFwd = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_auxiliaryArbFeedFwd", false, 0);
    m_PushMotionProfileTrajectory_3_auxiliaryPos = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_auxiliaryPos", false, 0);
    m_PushMotionProfileTrajectory_3_auxiliaryVel = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_auxiliaryVel", false, 0);
    m_PushMotionProfileTrajectory_3_isLastPoint = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_isLastPoint", false, 0);
    m_PushMotionProfileTrajectory_3_position = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_position", false, 0);
    m_PushMotionProfileTrajectory_3_profileSlotSelect0 = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_profileSlotSelect0", false, 0);
    m_PushMotionProfileTrajectory_3_profileSlotSelect1 = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_profileSlotSelect1", false, 0);
    m_PushMotionProfileTrajectory_3_timeDur = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_timeDur", false, 0);
    m_PushMotionProfileTrajectory_3_useAuxPID = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_useAuxPID", false, 0);
    m_PushMotionProfileTrajectory_3_velocity = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_velocity", false, 0);
    m_PushMotionProfileTrajectory_3_zeroPos0 = m_simDevice.CreateDouble("PushMotionProfileTrajectory_3_zeroPos0", false, 0);
    m_PushMotionProfileTrajectory_headingDeg = m_simDevice.CreateDouble("PushMotionProfileTrajectory_headingDeg", false, 0);
    m_PushMotionProfileTrajectory_isLastPoint = m_simDevice.CreateDouble("PushMotionProfileTrajectory_isLastPoint", false, 0);
    m_PushMotionProfileTrajectory_position = m_simDevice.CreateDouble("PushMotionProfileTrajectory_position", false, 0);
    m_PushMotionProfileTrajectory_profileSlotSelect = m_simDevice.CreateDouble("PushMotionProfileTrajectory_profileSlotSelect", false, 0);
    m_PushMotionProfileTrajectory_velocity = m_simDevice.CreateDouble("PushMotionProfileTrajectory_velocity", false, 0);
    m_PushMotionProfileTrajectory_zeroPos = m_simDevice.CreateDouble("PushMotionProfileTrajectory_zeroPos", false, 0);
    m_QuadPinStates_quadA = m_simDevice.CreateDouble("QuadPinStates_quadA", false, 0);
    m_QuadPinStates_quadB = m_simDevice.CreateDouble("QuadPinStates_quadB", false, 0);
    m_QuadPinStates_quadIdx = m_simDevice.CreateDouble("QuadPinStates_quadIdx", false, 0);
    m_QuadraturePosition_newPosition = m_simDevice.CreateDouble("QuadraturePosition_newPosition", false, 0);
    m_QuadraturePosition_param = m_simDevice.CreateDouble("QuadraturePosition_param", false, 0);
    m_QuadratureSensor_pos = m_simDevice.CreateDouble("QuadratureSensor_pos", false, 0);
    m_QuadratureSensor_vel = m_simDevice.CreateDouble("QuadratureSensor_vel", false, 0);
    m_QuadratureVelocity_param = m_simDevice.CreateDouble("QuadratureVelocity_param", false, 0);
    m_RevLimitSwitchClosed_param = m_simDevice.CreateDouble("RevLimitSwitchClosed_param", false, 0);
    m_SelectDemandType_enable = m_simDevice.CreateDouble("SelectDemandType_enable", false, 0);
    m_SensorPhase_PhaseSensor = m_simDevice.CreateDouble("SensorPhase_PhaseSensor", false, 0);
    m_StartMotionProfile_controlMode = m_simDevice.CreateDouble("StartMotionProfile_controlMode", false, 0);
    m_StartMotionProfile_minBufferedPts = m_simDevice.CreateDouble("StartMotionProfile_minBufferedPts", false, 0);
    m_StartMotionProfile_streamHandle = m_simDevice.CreateDouble("StartMotionProfile_streamHandle", false, 0);
    m_StatorCurrent_current = m_simDevice.CreateDouble("StatorCurrent_current", false, 0);
    m_StatusFramePeriod_frame = m_simDevice.CreateDouble("StatusFramePeriod_frame", false, 0);
    m_StatusFramePeriod_periodMs = m_simDevice.CreateDouble("StatusFramePeriod_periodMs", false, 0);
    m_StickyFaults_param = m_simDevice.CreateDouble("StickyFaults_param", false, 0);
    m_SupplyCurrent_current = m_simDevice.CreateDouble("SupplyCurrent_current", false, 0);
    m_Temperature_temperature = m_simDevice.CreateDouble("Temperature_temperature", false, 0);
    m__4_demand0 = m_simDevice.CreateDouble("_4_demand0", false, 0);
    m__4_demand1 = m_simDevice.CreateDouble("_4_demand1", false, 0);
    m__4_demand1Type = m_simDevice.CreateDouble("_4_demand1Type", false, 0);
    m__4_mode = m_simDevice.CreateDouble("_4_mode", false, 0);

    for (int slotId = 0; slotId < 6; ++slotId)
    {

        m_slotted_variables[slotId].m_ActiveTrajectoryAll_5_arbFeedFwd = m_simDevice.CreateDouble(std::string("ActiveTrajectoryAll_5_arbFeedFwd[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ActiveTrajectoryAll_5_pos = m_simDevice.CreateDouble(std::string("ActiveTrajectoryAll_5_pos[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ActiveTrajectoryAll_5_vel = m_simDevice.CreateDouble(std::string("ActiveTrajectoryAll_5_vel[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ActiveTrajectoryArbFeedFwd_3_param = m_simDevice.CreateDouble(std::string("ActiveTrajectoryArbFeedFwd_3_param[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ActiveTrajectoryPosition_3_param = m_simDevice.CreateDouble(std::string("ActiveTrajectoryPosition_3_param[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ActiveTrajectoryVelocity_3_param = m_simDevice.CreateDouble(std::string("ActiveTrajectoryVelocity_3_param[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ClosedLoopError_closedLoopError = m_simDevice.CreateDouble(std::string("ClosedLoopError_closedLoopError[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ClosedLoopTarget_value = m_simDevice.CreateDouble(std::string("ClosedLoopTarget_value[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ConfigAllowableClosedloopError_allowableClosedLoopError = m_simDevice.CreateDouble(std::string("ConfigAllowableClosedloopError_allowableClosedLoopError[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ConfigClosedLoopPeakOutput_percentOut = m_simDevice.CreateDouble(std::string("ConfigClosedLoopPeakOutput_percentOut[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ConfigClosedLoopPeriod_loopTimeMs = m_simDevice.CreateDouble(std::string("ConfigClosedLoopPeriod_loopTimeMs[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ConfigMaxIntegralAccumulator_iaccum = m_simDevice.CreateDouble(std::string("ConfigMaxIntegralAccumulator_iaccum[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ConfigSelectedFeedbackCoefficient_coefficient = m_simDevice.CreateDouble(std::string("ConfigSelectedFeedbackCoefficient_coefficient[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ConfigSelectedFeedbackSensor_feedbackDevice = m_simDevice.CreateDouble(std::string("ConfigSelectedFeedbackSensor_feedbackDevice[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_Config_IntegralZone_izone = m_simDevice.CreateDouble(std::string("Config_IntegralZone_izone[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_Config_kD_value = m_simDevice.CreateDouble(std::string("Config_kD_value[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_Config_kF_value = m_simDevice.CreateDouble(std::string("Config_kF_value[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_Config_kI_value = m_simDevice.CreateDouble(std::string("Config_kI_value[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_Config_kP_value = m_simDevice.CreateDouble(std::string("Config_kP_value[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_ErrorDerivative_derror = m_simDevice.CreateDouble(std::string("ErrorDerivative_derror[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_IntegralAccumulator_iaccum = m_simDevice.CreateDouble(std::string("IntegralAccumulator_iaccum[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_SelectedSensorPosition_param = m_simDevice.CreateDouble(std::string("SelectedSensorPosition_param[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_SelectedSensorPosition_sensorPos = m_simDevice.CreateDouble(std::string("SelectedSensorPosition_sensorPos[" + std::to_string(slotId) + "]").c_str(), false, 0);
        m_slotted_variables[slotId].m_SelectedSensorVelocity_param = m_simDevice.CreateDouble(std::string("SelectedSensorVelocity_param[" + std::to_string(slotId) + "]").c_str(), false, 0);
    }

    Send("Create");
}

void SnobotSim::CtreMotControllerWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gMotControllerCallbacks.empty())
    {
        gMotControllerCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
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
    if (!gMotControllerCallbacks.empty())
    {
        gMotControllerCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreMotControllerWrapper::GetDeviceNumber(int* deviceNumber)
{
    RECEIVE_HELPER("GetDeviceNumber", sizeof(*deviceNumber));
    PoplateReceiveResults(buffer, deviceNumber, buffer_pos);

    *deviceNumber = m_DeviceNumber_deviceNumber.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetDescription(char* toFill, int toFillByteSz, size_t* numBytesFilled)
{
    RECEIVE_HELPER("GetDescription", sizeof(*toFill) + sizeof(toFillByteSz) + sizeof(*numBytesFilled));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, &toFillByteSz, buffer_pos);
    PoplateReceiveResults(buffer, numBytesFilled, buffer_pos);

//    *toFillByteSz = m_Description_toFillByteSz.Get();
    *toFill = m_Description_toFill.Get();
    *numBytesFilled = m_Description_numBytesFilled.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetBaseID(int* baseArbId)
{
    RECEIVE_HELPER("GetBaseID", sizeof(*baseArbId));
    PoplateReceiveResults(buffer, baseArbId, buffer_pos);

    *baseArbId = m_BaseID_baseArbId.Get();
}

void SnobotSim::CtreMotControllerWrapper::SetDemand(int mode, int demand0, int demand1)
{
    m_Demand_mode.Set(mode);
    m_Demand_demand1.Set(demand1);
    m_Demand_demand0.Set(demand0);

    Send("SetDemand", mode, demand0, demand1);
}

void SnobotSim::CtreMotControllerWrapper::Set_4(int mode, double demand0, double demand1, int demand1Type)
{
    m__4_mode.Set(mode);
    m__4_demand1Type.Set(demand1Type);
    m__4_demand1.Set(demand1);
    m__4_demand0.Set(demand0);

    Send("Set_4", mode, demand0, demand1, demand1Type);
}

void SnobotSim::CtreMotControllerWrapper::SetNeutralMode(int neutralMode)
{
    m_NeutralMode_neutralMode.Set(neutralMode);

    Send("SetNeutralMode", neutralMode);
}

void SnobotSim::CtreMotControllerWrapper::SetSensorPhase(bool PhaseSensor)
{
    m_SensorPhase_PhaseSensor.Set(PhaseSensor);

    Send("SetSensorPhase", PhaseSensor);
}

void SnobotSim::CtreMotControllerWrapper::SetInverted(bool invert)
{
    m_Inverted_invert.Set(invert);

    Send("SetInverted", invert);
}

void SnobotSim::CtreMotControllerWrapper::SetInverted_2(int invertType)
{
    m_Inverted_2_invertType.Set(invertType);

    Send("SetInverted_2", invertType);
}

void SnobotSim::CtreMotControllerWrapper::ConfigFactoryDefault()
{

    Send("ConfigFactoryDefault");
}

void SnobotSim::CtreMotControllerWrapper::ConfigOpenLoopRamp(double secondsFromNeutralToFull)
{
    m_ConfigOpenLoopRamp_secondsFromNeutralToFull.Set(secondsFromNeutralToFull);

    Send("ConfigOpenLoopRamp", secondsFromNeutralToFull);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClosedLoopRamp(double secondsFromNeutralToFull)
{
    m_ConfigClosedLoopRamp_secondsFromNeutralToFull.Set(secondsFromNeutralToFull);

    Send("ConfigClosedLoopRamp", secondsFromNeutralToFull);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPeakOutputForward(double percentOut)
{
    m_ConfigPeakOutputForward_percentOut.Set(percentOut);

    Send("ConfigPeakOutputForward", percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPeakOutputReverse(double percentOut)
{
    m_ConfigPeakOutputReverse_percentOut.Set(percentOut);

    Send("ConfigPeakOutputReverse", percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigNominalOutputForward(double percentOut)
{
    m_ConfigNominalOutputForward_percentOut.Set(percentOut);

    Send("ConfigNominalOutputForward", percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigNominalOutputReverse(double percentOut)
{
    m_ConfigNominalOutputReverse_percentOut.Set(percentOut);

    Send("ConfigNominalOutputReverse", percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigNeutralDeadband(double percentDeadband)
{
    m_ConfigNeutralDeadband_percentDeadband.Set(percentDeadband);

    Send("ConfigNeutralDeadband", percentDeadband);
}

void SnobotSim::CtreMotControllerWrapper::ConfigVoltageCompSaturation(double voltage)
{
    m_ConfigVoltageCompSaturation_voltage.Set(voltage);

    Send("ConfigVoltageCompSaturation", voltage);
}

void SnobotSim::CtreMotControllerWrapper::ConfigVoltageMeasurementFilter(int filterWindowSamples)
{
    m_ConfigVoltageMeasurementFilter_filterWindowSamples.Set(filterWindowSamples);

    Send("ConfigVoltageMeasurementFilter", filterWindowSamples);
}

void SnobotSim::CtreMotControllerWrapper::EnableVoltageCompensation(bool enable)
{
    m_EnableVoltageCompensation_enable.Set(enable);

    Send("EnableVoltageCompensation", enable);
}

void SnobotSim::CtreMotControllerWrapper::GetInverted(bool* invert)
{
    RECEIVE_HELPER("GetInverted", sizeof(*invert));
    PoplateReceiveResults(buffer, invert, buffer_pos);

    *invert = m_Inverted_invert.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetBusVoltage(double* voltage)
{
    RECEIVE_HELPER("GetBusVoltage", sizeof(*voltage));
    PoplateReceiveResults(buffer, voltage, buffer_pos);

    *voltage = m_BusVoltage_voltage.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetMotorOutputPercent(double* percentOutput)
{
    RECEIVE_HELPER("GetMotorOutputPercent", sizeof(*percentOutput));
    PoplateReceiveResults(buffer, percentOutput, buffer_pos);

    *percentOutput = m_MotorOutputPercent_percentOutput.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetOutputCurrent(double* current)
{
    RECEIVE_HELPER("GetOutputCurrent", sizeof(*current));
    PoplateReceiveResults(buffer, current, buffer_pos);

    *current = m_OutputCurrent_current.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetSupplyCurrent(double* current)
{
    RECEIVE_HELPER("GetSupplyCurrent", sizeof(*current));
    PoplateReceiveResults(buffer, current, buffer_pos);

    *current = m_SupplyCurrent_current.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetStatorCurrent(double* current)
{
    RECEIVE_HELPER("GetStatorCurrent", sizeof(*current));
    PoplateReceiveResults(buffer, current, buffer_pos);

    *current = m_StatorCurrent_current.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetTemperature(double* temperature)
{
    RECEIVE_HELPER("GetTemperature", sizeof(*temperature));
    PoplateReceiveResults(buffer, temperature, buffer_pos);

    *temperature = m_Temperature_temperature.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigSelectedFeedbackSensor(int feedbackDevice, int pidIdx)
{
    m_slotted_variables[pidIdx].m_ConfigSelectedFeedbackSensor_feedbackDevice.Set(feedbackDevice);

    Send("ConfigSelectedFeedbackSensor", feedbackDevice, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSelectedFeedbackCoefficient(double coefficient, int pidIdx)
{
    m_slotted_variables[pidIdx].m_ConfigSelectedFeedbackCoefficient_coefficient.Set(coefficient);

    Send("ConfigSelectedFeedbackCoefficient", coefficient, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::ConfigRemoteFeedbackFilter(int deviceID, int remoteSensorSource, int remoteOrdinal)
{
    m_ConfigRemoteFeedbackFilter_remoteSensorSource.Set(remoteSensorSource);
    m_ConfigRemoteFeedbackFilter_remoteOrdinal.Set(remoteOrdinal);
    m_ConfigRemoteFeedbackFilter_deviceID.Set(deviceID);

    Send("ConfigRemoteFeedbackFilter", deviceID, remoteSensorSource, remoteOrdinal);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSensorTerm(int sensorTerm, int feedbackDevice)
{
    m_ConfigSensorTerm_sensorTerm.Set(sensorTerm);
    m_ConfigSensorTerm_feedbackDevice.Set(feedbackDevice);

    Send("ConfigSensorTerm", sensorTerm, feedbackDevice);
}

void SnobotSim::CtreMotControllerWrapper::GetSelectedSensorPosition(int* param, int pidIdx)
{
    RECEIVE_HELPER("GetSelectedSensorPosition", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *param = m_slotted_variables[pidIdx].m_SelectedSensorPosition_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetSelectedSensorVelocity(int* param, int pidIdx)
{
    RECEIVE_HELPER("GetSelectedSensorVelocity", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *param = m_slotted_variables[pidIdx].m_SelectedSensorVelocity_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::SetSelectedSensorPosition(int sensorPos, int pidIdx)
{
    m_slotted_variables[pidIdx].m_SelectedSensorPosition_sensorPos.Set(sensorPos);

    Send("SetSelectedSensorPosition", sensorPos, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::SetControlFramePeriod(int frame, int periodMs)
{
    m_ControlFramePeriod_periodMs.Set(periodMs);
    m_ControlFramePeriod_frame.Set(frame);

    Send("SetControlFramePeriod", frame, periodMs);
}

void SnobotSim::CtreMotControllerWrapper::SetStatusFramePeriod(int frame, uint8_t periodMs)
{
    m_StatusFramePeriod_periodMs.Set(periodMs);
    m_StatusFramePeriod_frame.Set(frame);

    Send("SetStatusFramePeriod", frame, periodMs);
}

void SnobotSim::CtreMotControllerWrapper::GetStatusFramePeriod(int frame, int* periodMs)
{
    RECEIVE_HELPER("GetStatusFramePeriod", sizeof(frame) + sizeof(*periodMs));
    PoplateReceiveResults(buffer, &frame, buffer_pos);
    PoplateReceiveResults(buffer, periodMs, buffer_pos);

    *periodMs = m_StatusFramePeriod_periodMs.Get();
//    *frame = m_StatusFramePeriod_frame.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigVelocityMeasurementPeriod(int period)
{
    m_ConfigVelocityMeasurementPeriod_period.Set(period);

    Send("ConfigVelocityMeasurementPeriod", period);
}

void SnobotSim::CtreMotControllerWrapper::ConfigVelocityMeasurementWindow(int windowSize)
{
    m_ConfigVelocityMeasurementWindow_windowSize.Set(windowSize);

    Send("ConfigVelocityMeasurementWindow", windowSize);
}

void SnobotSim::CtreMotControllerWrapper::ConfigForwardLimitSwitchSource(int type, int normalOpenOrClose, int deviceID)
{
    m_ConfigForwardLimitSwitchSource_type.Set(type);
    m_ConfigForwardLimitSwitchSource_normalOpenOrClose.Set(normalOpenOrClose);
    m_ConfigForwardLimitSwitchSource_deviceID.Set(deviceID);

    Send("ConfigForwardLimitSwitchSource", type, normalOpenOrClose, deviceID);
}

void SnobotSim::CtreMotControllerWrapper::ConfigReverseLimitSwitchSource(int type, int normalOpenOrClose, int deviceID)
{
    m_ConfigReverseLimitSwitchSource_type.Set(type);
    m_ConfigReverseLimitSwitchSource_normalOpenOrClose.Set(normalOpenOrClose);
    m_ConfigReverseLimitSwitchSource_deviceID.Set(deviceID);

    Send("ConfigReverseLimitSwitchSource", type, normalOpenOrClose, deviceID);
}

void SnobotSim::CtreMotControllerWrapper::OverrideLimitSwitchesEnable(bool enable)
{
    m_OverrideLimitSwitchesEnable_enable.Set(enable);

    Send("OverrideLimitSwitchesEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigForwardSoftLimitThreshold(int forwardSensorLimit)
{
    m_ConfigForwardSoftLimitThreshold_forwardSensorLimit.Set(forwardSensorLimit);

    Send("ConfigForwardSoftLimitThreshold", forwardSensorLimit);
}

void SnobotSim::CtreMotControllerWrapper::ConfigReverseSoftLimitThreshold(int reverseSensorLimit)
{
    m_ConfigReverseSoftLimitThreshold_reverseSensorLimit.Set(reverseSensorLimit);

    Send("ConfigReverseSoftLimitThreshold", reverseSensorLimit);
}

void SnobotSim::CtreMotControllerWrapper::ConfigForwardSoftLimitEnable(bool enable)
{
    m_ConfigForwardSoftLimitEnable_enable.Set(enable);

    Send("ConfigForwardSoftLimitEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigReverseSoftLimitEnable(bool enable)
{
    m_ConfigReverseSoftLimitEnable_enable.Set(enable);

    Send("ConfigReverseSoftLimitEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::OverrideSoftLimitsEnable(bool enable)
{
    m_OverrideSoftLimitsEnable_enable.Set(enable);

    Send("OverrideSoftLimitsEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::Config_kP(int slotIdx, double value)
{
    m_slotted_variables[slotIdx].m_Config_kP_value.Set(value);

    Send("Config_kP", slotIdx, value);
}

void SnobotSim::CtreMotControllerWrapper::Config_kI(int slotIdx, double value)
{
    m_slotted_variables[slotIdx].m_Config_kI_value.Set(value);

    Send("Config_kI", slotIdx, value);
}

void SnobotSim::CtreMotControllerWrapper::Config_kD(int slotIdx, double value)
{
    m_slotted_variables[slotIdx].m_Config_kD_value.Set(value);

    Send("Config_kD", slotIdx, value);
}

void SnobotSim::CtreMotControllerWrapper::Config_kF(int slotIdx, double value)
{
    m_slotted_variables[slotIdx].m_Config_kF_value.Set(value);

    Send("Config_kF", slotIdx, value);
}

void SnobotSim::CtreMotControllerWrapper::Config_IntegralZone(int slotIdx, double izone)
{
    m_slotted_variables[slotIdx].m_Config_IntegralZone_izone.Set(izone);

    Send("Config_IntegralZone", slotIdx, izone);
}

void SnobotSim::CtreMotControllerWrapper::ConfigAllowableClosedloopError(int slotIdx, int allowableClosedLoopError)
{
    m_slotted_variables[slotIdx].m_ConfigAllowableClosedloopError_allowableClosedLoopError.Set(allowableClosedLoopError);

    Send("ConfigAllowableClosedloopError", slotIdx, allowableClosedLoopError);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMaxIntegralAccumulator(int slotIdx, double iaccum)
{
    m_slotted_variables[slotIdx].m_ConfigMaxIntegralAccumulator_iaccum.Set(iaccum);

    Send("ConfigMaxIntegralAccumulator", slotIdx, iaccum);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClosedLoopPeakOutput(int slotIdx, double percentOut)
{
    m_slotted_variables[slotIdx].m_ConfigClosedLoopPeakOutput_percentOut.Set(percentOut);

    Send("ConfigClosedLoopPeakOutput", slotIdx, percentOut);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs)
{
    m_slotted_variables[slotIdx].m_ConfigClosedLoopPeriod_loopTimeMs.Set(loopTimeMs);

    Send("ConfigClosedLoopPeriod", slotIdx, loopTimeMs);
}

void SnobotSim::CtreMotControllerWrapper::SetIntegralAccumulator(double iaccum, int pidIdx)
{
    m_slotted_variables[pidIdx].m_IntegralAccumulator_iaccum.Set(iaccum);

    Send("SetIntegralAccumulator", iaccum, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::GetClosedLoopError(int* closedLoopError, int pidIdx)
{
    RECEIVE_HELPER("GetClosedLoopError", sizeof(*closedLoopError) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, closedLoopError, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *closedLoopError = m_slotted_variables[pidIdx].m_ClosedLoopError_closedLoopError.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetIntegralAccumulator(double* iaccum, int pidIdx)
{
    RECEIVE_HELPER("GetIntegralAccumulator", sizeof(*iaccum) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, iaccum, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *iaccum = m_slotted_variables[pidIdx].m_IntegralAccumulator_iaccum.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetErrorDerivative(double* derror, int pidIdx)
{
    RECEIVE_HELPER("GetErrorDerivative", sizeof(*derror) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, derror, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *derror = m_slotted_variables[pidIdx].m_ErrorDerivative_derror.Get();
}

void SnobotSim::CtreMotControllerWrapper::SelectProfileSlot(int slotIdx, int pidIdx)
{

    Send("SelectProfileSlot", slotIdx, pidIdx);
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryPosition(int* param)
{
    RECEIVE_HELPER("GetActiveTrajectoryPosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_ActiveTrajectoryPosition_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryVelocity(int* param)
{
    RECEIVE_HELPER("GetActiveTrajectoryVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_ActiveTrajectoryVelocity_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryHeading(double* param)
{
    RECEIVE_HELPER("GetActiveTrajectoryHeading", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_ActiveTrajectoryHeading_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryPosition_3(int* param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryPosition_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *param = m_slotted_variables[pidIdx].m_ActiveTrajectoryPosition_3_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryVelocity_3(int* param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryVelocity_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *param = m_slotted_variables[pidIdx].m_ActiveTrajectoryVelocity_3_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryArbFeedFwd_3(double* param, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryArbFeedFwd_3", sizeof(*param) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, param, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *param = m_slotted_variables[pidIdx].m_ActiveTrajectoryArbFeedFwd_3_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryAll(int* vel, int* pos, double* heading)
{
    RECEIVE_HELPER("GetActiveTrajectoryAll", sizeof(*vel) + sizeof(*pos) + sizeof(*heading));
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, heading, buffer_pos);

    *vel = m_ActiveTrajectoryAll_vel.Get();
    *pos = m_ActiveTrajectoryAll_pos.Get();
    *heading = m_ActiveTrajectoryAll_heading.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetActiveTrajectoryAll_5(int* vel, int* pos, double* arbFeedFwd, int pidIdx)
{
    RECEIVE_HELPER("GetActiveTrajectoryAll_5", sizeof(*vel) + sizeof(*pos) + sizeof(*arbFeedFwd) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, arbFeedFwd, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *vel = m_slotted_variables[pidIdx].m_ActiveTrajectoryAll_5_vel.Get();
    *pos = m_slotted_variables[pidIdx].m_ActiveTrajectoryAll_5_pos.Get();
    *arbFeedFwd = m_slotted_variables[pidIdx].m_ActiveTrajectoryAll_5_arbFeedFwd.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionCruiseVelocity(int sensorUnitsPer100ms)
{
    m_ConfigMotionCruiseVelocity_sensorUnitsPer100ms.Set(sensorUnitsPer100ms);

    Send("ConfigMotionCruiseVelocity", sensorUnitsPer100ms);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionAcceleration(int sensorUnitsPer100msPerSec)
{
    m_ConfigMotionAcceleration_sensorUnitsPer100msPerSec.Set(sensorUnitsPer100msPerSec);

    Send("ConfigMotionAcceleration", sensorUnitsPer100msPerSec);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionSCurveStrength(int curveStrength)
{
    m_ConfigMotionSCurveStrength_curveStrength.Set(curveStrength);

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

    *value = m_MotionProfileTopLevelBufferCount_value.Get();
}

void SnobotSim::CtreMotControllerWrapper::PushMotionProfileTrajectory(double position, double velocity, double headingDeg, int profileSlotSelect, bool isLastPoint, bool zeroPos)
{
    m_PushMotionProfileTrajectory_zeroPos.Set(zeroPos);
    m_PushMotionProfileTrajectory_velocity.Set(velocity);
    m_PushMotionProfileTrajectory_profileSlotSelect.Set(profileSlotSelect);
    m_PushMotionProfileTrajectory_position.Set(position);
    m_PushMotionProfileTrajectory_isLastPoint.Set(isLastPoint);
    m_PushMotionProfileTrajectory_headingDeg.Set(headingDeg);

    Send("PushMotionProfileTrajectory", position, velocity, headingDeg, profileSlotSelect, isLastPoint, zeroPos);
}

void SnobotSim::CtreMotControllerWrapper::PushMotionProfileTrajectory_2(double position, double velocity, double headingDeg, int profileSlotSelect0, int profileSlotSelect1, bool isLastPoint, bool zeroPos, int durationMs)
{
    m_PushMotionProfileTrajectory_2_zeroPos.Set(zeroPos);
    m_PushMotionProfileTrajectory_2_velocity.Set(velocity);
    m_PushMotionProfileTrajectory_2_profileSlotSelect1.Set(profileSlotSelect1);
    m_PushMotionProfileTrajectory_2_profileSlotSelect0.Set(profileSlotSelect0);
    m_PushMotionProfileTrajectory_2_position.Set(position);
    m_PushMotionProfileTrajectory_2_isLastPoint.Set(isLastPoint);
    m_PushMotionProfileTrajectory_2_headingDeg.Set(headingDeg);
    m_PushMotionProfileTrajectory_2_durationMs.Set(durationMs);

    Send("PushMotionProfileTrajectory_2", position, velocity, headingDeg, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos, durationMs);
}

void SnobotSim::CtreMotControllerWrapper::PushMotionProfileTrajectory_3(double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos0, uint32_t timeDur, bool useAuxPID)
{
    m_PushMotionProfileTrajectory_3_zeroPos0.Set(zeroPos0);
    m_PushMotionProfileTrajectory_3_velocity.Set(velocity);
    m_PushMotionProfileTrajectory_3_useAuxPID.Set(useAuxPID);
    m_PushMotionProfileTrajectory_3_timeDur.Set(timeDur);
    m_PushMotionProfileTrajectory_3_profileSlotSelect1.Set(profileSlotSelect1);
    m_PushMotionProfileTrajectory_3_profileSlotSelect0.Set(profileSlotSelect0);
    m_PushMotionProfileTrajectory_3_position.Set(position);
    m_PushMotionProfileTrajectory_3_isLastPoint.Set(isLastPoint);
    m_PushMotionProfileTrajectory_3_auxiliaryVel.Set(auxiliaryVel);
    m_PushMotionProfileTrajectory_3_auxiliaryPos.Set(auxiliaryPos);
    m_PushMotionProfileTrajectory_3_auxiliaryArbFeedFwd.Set(auxiliaryArbFeedFwd);
    m_PushMotionProfileTrajectory_3_arbFeedFwd.Set(arbFeedFwd);

    Send("PushMotionProfileTrajectory_3", position, velocity, arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos0, timeDur, useAuxPID);
}

void SnobotSim::CtreMotControllerWrapper::StartMotionProfile(void* streamHandle, uint32_t minBufferedPts, ctre::phoenix::motorcontrol::ControlMode controlMode)
{
//    m_StartMotionProfile_streamHandle.Set(streamHandle);
    m_StartMotionProfile_minBufferedPts.Set(minBufferedPts);
//    m_StartMotionProfile_controlMode.Set(controlMode);

    Send("StartMotionProfile", streamHandle, minBufferedPts, controlMode);
}

void SnobotSim::CtreMotControllerWrapper::IsMotionProfileFinished(bool* value)
{
    RECEIVE_HELPER("IsMotionProfileFinished", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_MotionProfileFinished_value.Get();
}

void SnobotSim::CtreMotControllerWrapper::IsMotionProfileTopLevelBufferFull(bool* value)
{
    RECEIVE_HELPER("IsMotionProfileTopLevelBufferFull", sizeof(*value));
    PoplateReceiveResults(buffer, value, buffer_pos);

    *value = m_MotionProfileTopLevelBufferFull_value.Get();
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

    *topBufferRem = m_MotionProfileStatus_topBufferRem.Get();
    *topBufferCnt = m_MotionProfileStatus_topBufferCnt.Get();
    *profileSlotSelect = m_MotionProfileStatus_profileSlotSelect.Get();
    *outputEnable = m_MotionProfileStatus_outputEnable.Get();
    *isUnderrun = m_MotionProfileStatus_isUnderrun.Get();
    *isLast = m_MotionProfileStatus_isLast.Get();
    *hasUnderrun = m_MotionProfileStatus_hasUnderrun.Get();
    *btmBufferCnt = m_MotionProfileStatus_btmBufferCnt.Get();
    *activePointValid = m_MotionProfileStatus_activePointValid.Get();
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

    *topBufferRem = m_MotionProfileStatus_2_topBufferRem.Get();
    *topBufferCnt = m_MotionProfileStatus_2_topBufferCnt.Get();
    *timeDurMs = m_MotionProfileStatus_2_timeDurMs.Get();
    *profileSlotSelect1 = m_MotionProfileStatus_2_profileSlotSelect1.Get();
    *profileSlotSelect = m_MotionProfileStatus_2_profileSlotSelect.Get();
    *outputEnable = m_MotionProfileStatus_2_outputEnable.Get();
    *isUnderrun = m_MotionProfileStatus_2_isUnderrun.Get();
    *isLast = m_MotionProfileStatus_2_isLast.Get();
    *hasUnderrun = m_MotionProfileStatus_2_hasUnderrun.Get();
    *btmBufferCnt = m_MotionProfileStatus_2_btmBufferCnt.Get();
    *activePointValid = m_MotionProfileStatus_2_activePointValid.Get();
}

void SnobotSim::CtreMotControllerWrapper::ClearMotionProfileHasUnderrun()
{

    Send("ClearMotionProfileHasUnderrun");
}

void SnobotSim::CtreMotControllerWrapper::ChangeMotionControlFramePeriod(int periodMs)
{
    m_ChangeMotionControlFramePeriod_periodMs.Set(periodMs);

    Send("ChangeMotionControlFramePeriod", periodMs);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionProfileTrajectoryPeriod(int durationMs)
{
    m_ConfigMotionProfileTrajectoryPeriod_durationMs.Set(durationMs);

    Send("ConfigMotionProfileTrajectoryPeriod", durationMs);
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotionProfileTrajectoryInterpolationEnable(bool enable)
{
    m_ConfigMotionProfileTrajectoryInterpolationEnable_enable.Set(enable);

    Send("ConfigMotionProfileTrajectoryInterpolationEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigFeedbackNotContinuous(bool feedbackNotContinuous)
{
    m_ConfigFeedbackNotContinuous_feedbackNotContinuous.Set(feedbackNotContinuous);

    Send("ConfigFeedbackNotContinuous", feedbackNotContinuous);
}

void SnobotSim::CtreMotControllerWrapper::ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(bool remoteSensorClosedLoopDisableNeutralOnLOS)
{
    m_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS_remoteSensorClosedLoopDisableNeutralOnLOS.Set(remoteSensorClosedLoopDisableNeutralOnLOS);

    Send("ConfigRemoteSensorClosedLoopDisableNeutralOnLOS", remoteSensorClosedLoopDisableNeutralOnLOS);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClearPositionOnLimitF(bool clearPositionOnLimitF)
{
    m_ConfigClearPositionOnLimitF_clearPositionOnLimitF.Set(clearPositionOnLimitF);

    Send("ConfigClearPositionOnLimitF", clearPositionOnLimitF);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClearPositionOnLimitR(bool clearPositionOnLimitR)
{
    m_ConfigClearPositionOnLimitR_clearPositionOnLimitR.Set(clearPositionOnLimitR);

    Send("ConfigClearPositionOnLimitR", clearPositionOnLimitR);
}

void SnobotSim::CtreMotControllerWrapper::ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx)
{
    m_ConfigClearPositionOnQuadIdx_clearPositionOnQuadIdx.Set(clearPositionOnQuadIdx);

    Send("ConfigClearPositionOnQuadIdx", clearPositionOnQuadIdx);
}

void SnobotSim::CtreMotControllerWrapper::ConfigLimitSwitchDisableNeutralOnLOS(bool limitSwitchDisableNeutralOnLOS)
{
    m_ConfigLimitSwitchDisableNeutralOnLOS_limitSwitchDisableNeutralOnLOS.Set(limitSwitchDisableNeutralOnLOS);

    Send("ConfigLimitSwitchDisableNeutralOnLOS", limitSwitchDisableNeutralOnLOS);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSoftLimitDisableNeutralOnLOS(bool softLimitDisableNeutralOnLOS)
{
    m_ConfigSoftLimitDisableNeutralOnLOS_softLimitDisableNeutralOnLOS.Set(softLimitDisableNeutralOnLOS);

    Send("ConfigSoftLimitDisableNeutralOnLOS", softLimitDisableNeutralOnLOS);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPulseWidthPeriod_EdgesPerRot(int pulseWidthPeriod_EdgesPerRot)
{
    m_ConfigPulseWidthPeriod_EdgesPerRot_pulseWidthPeriod_EdgesPerRot.Set(pulseWidthPeriod_EdgesPerRot);

    Send("ConfigPulseWidthPeriod_EdgesPerRot", pulseWidthPeriod_EdgesPerRot);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPulseWidthPeriod_FilterWindowSz(int pulseWidthPeriod_FilterWindowSz)
{
    m_ConfigPulseWidthPeriod_FilterWindowSz_pulseWidthPeriod_FilterWindowSz.Set(pulseWidthPeriod_FilterWindowSz);

    Send("ConfigPulseWidthPeriod_FilterWindowSz", pulseWidthPeriod_FilterWindowSz);
}

void SnobotSim::CtreMotControllerWrapper::GetFirmwareVersion(int* version)
{
    RECEIVE_HELPER("GetFirmwareVersion", sizeof(*version));
    PoplateReceiveResults(buffer, version, buffer_pos);

    *version = m_FirmwareVersion_version.Get();
}

void SnobotSim::CtreMotControllerWrapper::HasResetOccurred(bool* output)
{
    RECEIVE_HELPER("HasResetOccurred", sizeof(*output));
    PoplateReceiveResults(buffer, output, buffer_pos);

    *output = m_HasResetOccurred_output.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigSetCustomParam(int newValue, int paramIndex)
{
    m_ConfigSetCustomParam_paramIndex.Set(paramIndex);
    m_ConfigSetCustomParam_newValue.Set(newValue);

    Send("ConfigSetCustomParam", newValue, paramIndex);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetCustomParam(int* readValue, int paramIndex)
{
    RECEIVE_HELPER("ConfigGetCustomParam", sizeof(*readValue) + sizeof(paramIndex));
    PoplateReceiveResults(buffer, readValue, buffer_pos);
    PoplateReceiveResults(buffer, &paramIndex, buffer_pos);

    *readValue = m_ConfigGetCustomParam_readValue.Get();
//    *paramIndex = m_ConfigGetCustomParam_paramIndex.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigSetParameter(int param, double value, uint8_t subValue, int ordinal)
{
    m_ConfigSetParameter_value.Set(value);
    m_ConfigSetParameter_subValue.Set(subValue);
    m_ConfigSetParameter_param.Set(param);
    m_ConfigSetParameter_ordinal.Set(ordinal);

    Send("ConfigSetParameter", param, value, subValue, ordinal);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetParameter(int param, double* value, int ordinal)
{
    RECEIVE_HELPER("ConfigGetParameter", sizeof(param) + sizeof(*value) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);

    *value = m_ConfigGetParameter_value.Get();
//    *param = m_ConfigGetParameter_param.Get();
//    *ordinal = m_ConfigGetParameter_ordinal.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetParameter_6(int32_t param, int32_t valueToSend, int32_t* valueRecieved, uint8_t* subValue, int32_t ordinal)
{
    RECEIVE_HELPER("ConfigGetParameter_6", sizeof(param) + sizeof(valueToSend) + sizeof(*valueRecieved) + sizeof(*subValue) + sizeof(ordinal));
    PoplateReceiveResults(buffer, &param, buffer_pos);
    PoplateReceiveResults(buffer, &valueToSend, buffer_pos);
    PoplateReceiveResults(buffer, valueRecieved, buffer_pos);
    PoplateReceiveResults(buffer, subValue, buffer_pos);
    PoplateReceiveResults(buffer, &ordinal, buffer_pos);

//    *valueToSend = m_ConfigGetParameter_6_valueToSend.Get();
    *valueRecieved = m_ConfigGetParameter_6_valueRecieved.Get();
    *subValue = m_ConfigGetParameter_6_subValue.Get();
//    *param = m_ConfigGetParameter_6_param.Get();
//    *ordinal = m_ConfigGetParameter_6_ordinal.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigPeakCurrentLimit(int amps)
{
    m_ConfigPeakCurrentLimit_amps.Set(amps);

    Send("ConfigPeakCurrentLimit", amps);
}

void SnobotSim::CtreMotControllerWrapper::ConfigPeakCurrentDuration(int milliseconds)
{
    m_ConfigPeakCurrentDuration_milliseconds.Set(milliseconds);

    Send("ConfigPeakCurrentDuration", milliseconds);
}

void SnobotSim::CtreMotControllerWrapper::ConfigContinuousCurrentLimit(int amps)
{
    m_ConfigContinuousCurrentLimit_amps.Set(amps);

    Send("ConfigContinuousCurrentLimit", amps);
}

void SnobotSim::CtreMotControllerWrapper::EnableCurrentLimit(bool enable)
{
    m_EnableCurrentLimit_enable.Set(enable);

    Send("EnableCurrentLimit", enable);
}

void SnobotSim::CtreMotControllerWrapper::SetLastError(int error)
{
    m_LastError_error.Set(error);

    Send("SetLastError", error);
}

void SnobotSim::CtreMotControllerWrapper::GetAnalogIn(int* param)
{
    RECEIVE_HELPER("GetAnalogIn", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_AnalogIn_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::SetAnalogPosition(int newPosition)
{
    m_AnalogPosition_newPosition.Set(newPosition);

    Send("SetAnalogPosition", newPosition);
}

void SnobotSim::CtreMotControllerWrapper::GetAnalogInRaw(int* param)
{
    RECEIVE_HELPER("GetAnalogInRaw", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_AnalogInRaw_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetAnalogInVel(int* param)
{
    RECEIVE_HELPER("GetAnalogInVel", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_AnalogInVel_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetQuadraturePosition(int* param)
{
    RECEIVE_HELPER("GetQuadraturePosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_QuadraturePosition_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::SetQuadraturePosition(int newPosition)
{
    m_QuadraturePosition_newPosition.Set(newPosition);

    Send("SetQuadraturePosition", newPosition);
}

void SnobotSim::CtreMotControllerWrapper::GetQuadratureVelocity(int* param)
{
    RECEIVE_HELPER("GetQuadratureVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_QuadratureVelocity_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthPosition(int* param)
{
    RECEIVE_HELPER("GetPulseWidthPosition", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_PulseWidthPosition_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::SetPulseWidthPosition(int newPosition)
{
    m_PulseWidthPosition_newPosition.Set(newPosition);

    Send("SetPulseWidthPosition", newPosition);
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthVelocity(int* param)
{
    RECEIVE_HELPER("GetPulseWidthVelocity", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_PulseWidthVelocity_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthRiseToFallUs(int* param)
{
    RECEIVE_HELPER("GetPulseWidthRiseToFallUs", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_PulseWidthRiseToFallUs_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthRiseToRiseUs(int* param)
{
    RECEIVE_HELPER("GetPulseWidthRiseToRiseUs", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_PulseWidthRiseToRiseUs_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetPinStateQuadA(int* param)
{
    RECEIVE_HELPER("GetPinStateQuadA", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_PinStateQuadA_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetPinStateQuadB(int* param)
{
    RECEIVE_HELPER("GetPinStateQuadB", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_PinStateQuadB_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetPinStateQuadIdx(int* param)
{
    RECEIVE_HELPER("GetPinStateQuadIdx", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_PinStateQuadIdx_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::IsFwdLimitSwitchClosed(int* param)
{
    RECEIVE_HELPER("IsFwdLimitSwitchClosed", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_FwdLimitSwitchClosed_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::IsRevLimitSwitchClosed(int* param)
{
    RECEIVE_HELPER("IsRevLimitSwitchClosed", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_RevLimitSwitchClosed_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetFaults(int* param)
{
    RECEIVE_HELPER("GetFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_Faults_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetStickyFaults(int* param)
{
    RECEIVE_HELPER("GetStickyFaults", sizeof(*param));
    PoplateReceiveResults(buffer, param, buffer_pos);

    *param = m_StickyFaults_param.Get();
}

void SnobotSim::CtreMotControllerWrapper::ClearStickyFaults()
{

    Send("ClearStickyFaults");
}

void SnobotSim::CtreMotControllerWrapper::SelectDemandType(bool enable)
{
    m_SelectDemandType_enable.Set(enable);

    Send("SelectDemandType", enable);
}

void SnobotSim::CtreMotControllerWrapper::SetMPEOutput(int MpeOutput)
{
    m_MPEOutput_MpeOutput.Set(MpeOutput);

    Send("SetMPEOutput", MpeOutput);
}

void SnobotSim::CtreMotControllerWrapper::EnableHeadingHold(bool enable)
{
    m_EnableHeadingHold_enable.Set(enable);

    Send("EnableHeadingHold", enable);
}

void SnobotSim::CtreMotControllerWrapper::GetAnalogInAll(int* withOv, int* raw, int* vel)
{
    RECEIVE_HELPER("GetAnalogInAll", sizeof(*withOv) + sizeof(*raw) + sizeof(*vel));
    PoplateReceiveResults(buffer, withOv, buffer_pos);
    PoplateReceiveResults(buffer, raw, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);

    *withOv = m_AnalogInAll_withOv.Get();
    *vel = m_AnalogInAll_vel.Get();
    *raw = m_AnalogInAll_raw.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetQuadratureSensor(int* pos, int* vel)
{
    RECEIVE_HELPER("GetQuadratureSensor", sizeof(*pos) + sizeof(*vel));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);

    *vel = m_QuadratureSensor_vel.Get();
    *pos = m_QuadratureSensor_pos.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetPulseWidthAll(int* pos, int* vel, int* riseToRiseUs, int* riseToFallUs)
{
    RECEIVE_HELPER("GetPulseWidthAll", sizeof(*pos) + sizeof(*vel) + sizeof(*riseToRiseUs) + sizeof(*riseToFallUs));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);
    PoplateReceiveResults(buffer, riseToRiseUs, buffer_pos);
    PoplateReceiveResults(buffer, riseToFallUs, buffer_pos);

    *vel = m_PulseWidthAll_vel.Get();
    *riseToRiseUs = m_PulseWidthAll_riseToRiseUs.Get();
    *riseToFallUs = m_PulseWidthAll_riseToFallUs.Get();
    *pos = m_PulseWidthAll_pos.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetQuadPinStates(int* quadA, int* quadB, int* quadIdx)
{
    RECEIVE_HELPER("GetQuadPinStates", sizeof(*quadA) + sizeof(*quadB) + sizeof(*quadIdx));
    PoplateReceiveResults(buffer, quadA, buffer_pos);
    PoplateReceiveResults(buffer, quadB, buffer_pos);
    PoplateReceiveResults(buffer, quadIdx, buffer_pos);

    *quadIdx = m_QuadPinStates_quadIdx.Get();
    *quadB = m_QuadPinStates_quadB.Get();
    *quadA = m_QuadPinStates_quadA.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetLimitSwitchState(int* isFwdClosed, int* isRevClosed)
{
    RECEIVE_HELPER("GetLimitSwitchState", sizeof(*isFwdClosed) + sizeof(*isRevClosed));
    PoplateReceiveResults(buffer, isFwdClosed, buffer_pos);
    PoplateReceiveResults(buffer, isRevClosed, buffer_pos);

    *isRevClosed = m_LimitSwitchState_isRevClosed.Get();
    *isFwdClosed = m_LimitSwitchState_isFwdClosed.Get();
}

void SnobotSim::CtreMotControllerWrapper::GetClosedLoopTarget(int* value, int pidIdx)
{
    RECEIVE_HELPER("GetClosedLoopTarget", sizeof(*value) + sizeof(pidIdx));
    PoplateReceiveResults(buffer, value, buffer_pos);
    PoplateReceiveResults(buffer, &pidIdx, buffer_pos);

    *value = m_slotted_variables[pidIdx].m_ClosedLoopTarget_value.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigMotorCommutation(ctre::phoenix::motorcontrol::MotorCommutation motorCommutation)
{
//    m_ConfigMotorCommutation_motorCommutation.Set(motorCommutation);

    Send("ConfigMotorCommutation", motorCommutation);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetMotorCommutation(ctre::phoenix::motorcontrol::MotorCommutation* motorCommutation)
{
    RECEIVE_HELPER("ConfigGetMotorCommutation", sizeof(*motorCommutation));
    PoplateReceiveResults(buffer, motorCommutation, buffer_pos);

//    *motorCommutation = m_ConfigGetMotorCommutation_motorCommutation.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigSupplyCurrentLimit(const double* params, int paramCnt)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("ConfigSupplyCurrentLimit", sizeof(*params) + sizeof(paramCnt) + sizeof(timeoutMs));
    //    PoplateReceiveResults(buffer, params, buffer_pos);
    //    PoplateReceiveResults(buffer, &paramCnt, buffer_pos);
    //    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigStatorCurrentLimit(const double* params, int paramCnt)
{
    LOG_UNSUPPORTED_CAN_FUNC("")
    //    RECEIVE_HELPER("ConfigStatorCurrentLimit", sizeof(*params) + sizeof(paramCnt) + sizeof(timeoutMs));
    //    PoplateReceiveResults(buffer, params, buffer_pos);
    //    PoplateReceiveResults(buffer, &paramCnt, buffer_pos);
    //    PoplateReceiveResults(buffer, &timeoutMs, buffer_pos);
}

void SnobotSim::CtreMotControllerWrapper::ConfigSupplyCurrentLimitEnable(bool enable)
{
    m_ConfigSupplyCurrentLimitEnable_enable.Set(enable);

    Send("ConfigSupplyCurrentLimitEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigStatorCurrentLimitEnable(bool enable)
{
    m_ConfigStatorCurrentLimitEnable_enable.Set(enable);

    Send("ConfigStatorCurrentLimitEnable", enable);
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetSupplyCurrentLimit(double* toFill, int* fillCnt, int fillCapacity)
{
    RECEIVE_HELPER("ConfigGetSupplyCurrentLimit", sizeof(*toFill) + sizeof(*fillCnt) + sizeof(fillCapacity));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, fillCnt, buffer_pos);
    PoplateReceiveResults(buffer, &fillCapacity, buffer_pos);

    *toFill = m_ConfigGetSupplyCurrentLimit_toFill.Get();
    *fillCnt = m_ConfigGetSupplyCurrentLimit_fillCnt.Get();
//    *fillCapacity = m_ConfigGetSupplyCurrentLimit_fillCapacity.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigGetStatorCurrentLimit(double* toFill, int* fillCnt, int fillCapacity)
{
    RECEIVE_HELPER("ConfigGetStatorCurrentLimit", sizeof(*toFill) + sizeof(*fillCnt) + sizeof(fillCapacity));
    PoplateReceiveResults(buffer, toFill, buffer_pos);
    PoplateReceiveResults(buffer, fillCnt, buffer_pos);
    PoplateReceiveResults(buffer, &fillCapacity, buffer_pos);

    *toFill = m_ConfigGetStatorCurrentLimit_toFill.Get();
    *fillCnt = m_ConfigGetStatorCurrentLimit_fillCnt.Get();
//    *fillCapacity = m_ConfigGetStatorCurrentLimit_fillCapacity.Get();
}

void SnobotSim::CtreMotControllerWrapper::SetIntegratedSensorPosition(double newpos)
{
    m_IntegratedSensorPosition_newpos.Set(newpos);

    Send("SetIntegratedSensorPosition", newpos);
}

void SnobotSim::CtreMotControllerWrapper::SetIntegratedSensorPositionToAbsolute()
{

    Send("SetIntegratedSensorPositionToAbsolute");
}

void SnobotSim::CtreMotControllerWrapper::GetIntegratedSensor(double* pos, double* absPos, double* vel)
{
    RECEIVE_HELPER("GetIntegratedSensor", sizeof(*pos) + sizeof(*absPos) + sizeof(*vel));
    PoplateReceiveResults(buffer, pos, buffer_pos);
    PoplateReceiveResults(buffer, absPos, buffer_pos);
    PoplateReceiveResults(buffer, vel, buffer_pos);

    *vel = m_IntegratedSensor_vel.Get();
    *pos = m_IntegratedSensor_pos.Get();
    *absPos = m_IntegratedSensor_absPos.Get();
}

void SnobotSim::CtreMotControllerWrapper::ConfigIntegratedSensorAbsoluteRange(ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange)
{
    m_ConfigIntegratedSensorAbsoluteRange_absoluteSensorRange.Set(absoluteSensorRange);

    Send("ConfigIntegratedSensorAbsoluteRange", absoluteSensorRange);
}

void SnobotSim::CtreMotControllerWrapper::ConfigIntegratedSensorOffset(double offsetDegrees)
{
    m_ConfigIntegratedSensorOffset_offsetDegrees.Set(offsetDegrees);

    Send("ConfigIntegratedSensorOffset", offsetDegrees);
}

void SnobotSim::CtreMotControllerWrapper::ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy)
{
    m_ConfigIntegratedSensorInitializationStrategy_initializationStrategy.Set(initializationStrategy);

    Send("ConfigIntegratedSensorInitializationStrategy", initializationStrategy);
}

ctre::phoenix::ErrorCode SnobotSim::CtreMotControllerWrapper::GetLastError()
{
    int lastError = 0;
    RECEIVE_HELPER("GetLastError", sizeof(lastError));
    PoplateReceiveResults(buffer, &lastError, buffer_pos);
    return (ctre::phoenix::ErrorCode)lastError;
}

