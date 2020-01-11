package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

public class TestTalonSRXFunctions
{
    private CtreCallback mTestCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting callback " + aName);
        }
    };
    private CtreCallback mTestBuffTrajPointStreamCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting TrajPoint callback " + aName);
        }
    };

    @Test
    public void testAllFunctions()
    {
        CtreJni.registerCanBuffTrajPointStreamCallback(mTestBuffTrajPointStreamCallback);

        TalonSRX talon = new TalonSRX(0);
        CtreJni.registerCanMotorCallback(mTestCallback);

        TrajectoryPoint trajectoryPoint = new TrajectoryPoint();
        trajectoryPoint.timeDur = 10;

        //////////////////////////////////////////////

        talon.getSensorCollection();
        for (StatusFrameEnhanced statusFrameEnhanced : StatusFrameEnhanced.values())
        {
            talon.setStatusFramePeriod(statusFrameEnhanced, 0, 0);
        }
        for (StatusFrameEnhanced statusFrameEnhanced : StatusFrameEnhanced.values())
        {
            talon.setStatusFramePeriod(statusFrameEnhanced, 0);
        }
        for (StatusFrameEnhanced statusFrameEnhanced : StatusFrameEnhanced.values())
        {
            talon.getStatusFramePeriod(statusFrameEnhanced, 0);
        }
        for (StatusFrameEnhanced statusFrameEnhanced : StatusFrameEnhanced.values())
        {
            talon.getStatusFramePeriod(statusFrameEnhanced);
        }
        talon.getOutputCurrent();
        for (VelocityMeasPeriod velocityMeasPeriod : VelocityMeasPeriod.values())
        {
            talon.configVelocityMeasurementPeriod(velocityMeasPeriod, 0);
        }
        for (VelocityMeasPeriod velocityMeasPeriod : VelocityMeasPeriod.values())
        {
            talon.configVelocityMeasurementPeriod(velocityMeasPeriod);
        }
        talon.configVelocityMeasurementWindow(0, 0);
        talon.configVelocityMeasurementWindow(0);
        for (LimitSwitchSource limitSwitchSource : LimitSwitchSource.values())
        {
            for (LimitSwitchNormal limitSwitchNormal : LimitSwitchNormal.values())
            {
                talon.configForwardLimitSwitchSource(limitSwitchSource, limitSwitchNormal, 0);
            }
        }
        for (LimitSwitchSource limitSwitchSource : LimitSwitchSource.values())
        {
            for (LimitSwitchNormal limitSwitchNormal : LimitSwitchNormal.values())
            {
                talon.configForwardLimitSwitchSource(limitSwitchSource, limitSwitchNormal);
            }
        }
        for (LimitSwitchSource limitSwitchSource : LimitSwitchSource.values())
        {
            for (LimitSwitchNormal limitSwitchNormal : LimitSwitchNormal.values())
            {
                talon.configReverseLimitSwitchSource(limitSwitchSource, limitSwitchNormal, 0);
            }
        }
        for (LimitSwitchSource limitSwitchSource : LimitSwitchSource.values())
        {
            for (LimitSwitchNormal limitSwitchNormal : LimitSwitchNormal.values())
            {
                talon.configReverseLimitSwitchSource(limitSwitchSource, limitSwitchNormal);
            }
        }
        talon.configPeakCurrentLimit(0, 0);
        talon.configPeakCurrentLimit(0);
        talon.configPeakCurrentDuration(0, 0);
        talon.configPeakCurrentDuration(0);
        talon.configContinuousCurrentLimit(0, 0);
        talon.configContinuousCurrentLimit(0);
        talon.enableCurrentLimit(false);
        talon.configurePID(new TalonSRXPIDSetConfiguration(), 0, 0, false);
        talon.configurePID(new TalonSRXPIDSetConfiguration());
        talon.getPIDConfigs(new TalonSRXPIDSetConfiguration(), 0, 0);
        talon.getPIDConfigs(new TalonSRXPIDSetConfiguration());
        talon.configAllSettings(new TalonSRXConfiguration(), 0);
        talon.configAllSettings(new TalonSRXConfiguration());
        talon.getAllConfigs(new TalonSRXConfiguration(), 0);
        talon.getAllConfigs(new TalonSRXConfiguration());

        ///////////////////////////////////////

        talon.DestroyObject();
    }

}
