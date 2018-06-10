package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TestMotorControllerFunctions {
	private CtreCallback mTestCallback = new CtreCallback() {

		@Override
		public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount) {
			System.out.println("Getting callback " + aName);
		}
	};

    @Test
    public void testAllFunctions()
    {
        TalonSRX talon = new TalonSRX(0);
        CtreJni.registerCanMotorCallback(mTestCallback);

        talon.getHandle();
        talon.getDeviceID();
        for(ControlMode controlMode : ControlMode.values())
        {
            talon.set(controlMode, 0);
        }
        for(ControlMode controlMode : ControlMode.values())
        {
            talon.set(controlMode, 0, 0);
        }
        talon.neutralOutput();
        for(NeutralMode neutralMode : NeutralMode.values())
        {
            talon.setNeutralMode(neutralMode);
        }
        talon.enableHeadingHold(false);
        talon.selectDemandType(false);
        talon.setSensorPhase(false);
        talon.setInverted(false);
        talon.getInverted();
        talon.configOpenloopRamp(0, 0);
        talon.configClosedloopRamp(0, 0);
        talon.configPeakOutputForward(0, 0);
        talon.configPeakOutputReverse(0, 0);
        talon.configNominalOutputForward(0, 0);
        talon.configNominalOutputReverse(0, 0);
        talon.configNeutralDeadband(0, 0);
        talon.configVoltageCompSaturation(0, 0);
        talon.configVoltageMeasurementFilter(0, 0);
        talon.enableVoltageCompensation(false);
        talon.getBusVoltage();
        talon.getMotorOutputPercent();
        talon.getMotorOutputVoltage();
        talon.getOutputCurrent();
        talon.getTemperature();
        for(RemoteFeedbackDevice remoteFeedbackDevice : RemoteFeedbackDevice.values())
        {
        	talon.configSelectedFeedbackSensor(remoteFeedbackDevice, 0, 0);
        }
        for(FeedbackDevice feedbackDevice : FeedbackDevice.values())
        {
        	talon.configSelectedFeedbackSensor(feedbackDevice, 0, 0);
        }
        for(RemoteSensorSource remoteSensorSource : RemoteSensorSource.values())
        {
        	talon.configRemoteFeedbackFilter(0, remoteSensorSource, 0, 0);
        }
        for(SensorTerm sensorTerm : SensorTerm.values())
        {
            for(FeedbackDevice feedbackDevice : FeedbackDevice.values())
            {
            	talon.configSensorTerm(sensorTerm, feedbackDevice, 0);
            }
        }
        talon.getSelectedSensorPosition(0);
        talon.getSelectedSensorVelocity(0);
        talon.setSelectedSensorPosition(0, 0, 0);
        for(ControlFrame controlFrame : ControlFrame.values())
        {
        	talon.setControlFramePeriod(controlFrame, 0);
        }
        talon.setControlFramePeriod(0, 0);
        talon.setStatusFramePeriod(0, 0, 0);
        for(StatusFrame statusFrame : StatusFrame.values())
        {
        	talon.setStatusFramePeriod(statusFrame, 0, 0);
        }
        talon.getStatusFramePeriod(0, 0);
        for(StatusFrame statusFrame : StatusFrame.values())
        {
        	talon.getStatusFramePeriod(statusFrame, 0);
        }
        for(StatusFrameEnhanced statusFrame : StatusFrameEnhanced.values())
        {
        	talon.getStatusFramePeriod(statusFrame, 0);
        }
        for(VelocityMeasPeriod velocityMeasPeriod : VelocityMeasPeriod.values())
        {
        	talon.configVelocityMeasurementPeriod(velocityMeasPeriod, 0);
        }
        talon.configVelocityMeasurementWindow(0, 0);
        for(RemoteLimitSwitchSource remoteLimitSwitchSource : RemoteLimitSwitchSource.values())
        {
            for(LimitSwitchNormal limitSwitchNormal : LimitSwitchNormal.values())
            {
            	talon.configForwardLimitSwitchSource(remoteLimitSwitchSource, limitSwitchNormal, 0, 0);
            }
        }
        for(RemoteLimitSwitchSource remoteLimitSwitchSource : RemoteLimitSwitchSource.values())
        {
            for(LimitSwitchNormal limitSwitchNormal : LimitSwitchNormal.values())
            {
            	talon.configReverseLimitSwitchSource(remoteLimitSwitchSource, limitSwitchNormal, 0, 0);
            }
        }
        for(LimitSwitchSource limitSwitchSource : LimitSwitchSource.values())
        {
            for(LimitSwitchNormal limitSwitchNormal : LimitSwitchNormal.values())
            {
            	talon.configForwardLimitSwitchSource(limitSwitchSource, limitSwitchNormal, 0);
            }
        }
        talon.overrideLimitSwitchesEnable(false);
        talon.configForwardSoftLimitThreshold(0, 0);
        talon.configReverseSoftLimitThreshold(0, 0);
        talon.configForwardSoftLimitEnable(false, 0);
        talon.configReverseSoftLimitEnable(false, 0);
        talon.overrideSoftLimitsEnable(false);
        talon.config_kP(0, 0, 0);
        talon.config_kI(0, 0, 0);
        talon.config_kD(0, 0, 0);
        talon.config_kF(0, 0, 0);
        talon.config_IntegralZone(0, 0, 0);
        talon.configAllowableClosedloopError(0, 0, 0);
        talon.configMaxIntegralAccumulator(0, 0, 0);
        talon.setIntegralAccumulator(0, 0, 0);
        talon.getClosedLoopError(0);
        talon.getIntegralAccumulator(0);
        talon.getErrorDerivative(0);
        talon.selectProfileSlot(0, 0);
        talon.getActiveTrajectoryPosition();
        talon.getActiveTrajectoryVelocity();
        talon.getActiveTrajectoryHeading();
        talon.configMotionCruiseVelocity(0, 0);
        talon.configMotionAcceleration(0, 0);
        talon.clearMotionProfileTrajectories();
        talon.getMotionProfileTopLevelBufferCount();
        talon.pushMotionProfileTrajectory(new TrajectoryPoint());
        talon.isMotionProfileTopLevelBufferFull();
        talon.processMotionProfileBuffer();
        talon.getMotionProfileStatus(new MotionProfileStatus());
        talon.clearMotionProfileHasUnderrun(0);
        talon.changeMotionControlFramePeriod(0);
        talon.getLastError();
        talon.getFaults(new Faults());
        talon.getStickyFaults(new StickyFaults());
        talon.clearStickyFaults(0);
        talon.getFirmwareVersion();
        talon.hasResetOccurred();
        talon.configSetCustomParam(0, 0, 0);
        talon.configGetCustomParam(0, 0);
        for(ParamEnum paramEnum : ParamEnum.values())
        {
        	talon.configSetParameter(paramEnum, 0, 0, 0, 0);
        }
        talon.configSetParameter(0, 0, 0, 0, 0);
        for(ParamEnum paramEnum : ParamEnum.values())
        {
        	talon.configGetParameter(paramEnum, 0, 0);
        }
        talon.configGetParameter(0, 0, 0);
        talon.getBaseID();
        talon.valueUpdated();
        talon.getSensorCollection();
        talon.getControlMode();
    }

}
