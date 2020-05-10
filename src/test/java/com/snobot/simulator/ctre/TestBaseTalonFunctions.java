package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TestBaseTalonFunctions
{
    private CtreCallback mTestCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting BaseTalon callback " + aName + "' with size of " + aBuffer.capacity() + ", " + aCount); // NOPMD
        }
    };

    @Test
    public void testAllFunctions()
    {

        TalonSRX talon = new TalonSRX(0);
        CtreJni.registerCanMotorCallback(mTestCallback);

        TrajectoryPoint trajectoryPoint = new TrajectoryPoint();
        trajectoryPoint.timeDur = 10;
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
        talon.getStatorCurrent();
        talon.getSupplyCurrent();
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
        talon.isFwdLimitSwitchClosed();
        talon.isRevLimitSwitchClosed();

    }
}
