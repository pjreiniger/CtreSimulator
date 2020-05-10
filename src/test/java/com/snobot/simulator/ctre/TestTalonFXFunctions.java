package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class TestTalonFXFunctions
{
    private CtreCallback mTestCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting TalonFX callback " + aName + "' with size of " + aBuffer.capacity() + ", " + aCount); // NOPMD
        }
    };

    @Test
    public void testAllFunctions()
    {
        TalonFX talonFX = new TalonFX(0);
        for (TalonFXControlMode talonFXControlMode : TalonFXControlMode.values())
        {
//            talonFX.set(talonFXControlMode, 0);
        }
        for (TalonFXControlMode talonFXControlMode : TalonFXControlMode.values())
        {
            for (DemandType demandType : DemandType.values())
            {
                // talonFX.set(talonFXControlMode, 0, demandType, 0);
            }
        }
        for (TalonFXInvertType talonFXInvertType : TalonFXInvertType.values())
        {
            talonFX.setInverted(talonFXInvertType);
        }
        for (TalonFXFeedbackDevice talonFXFeedbackDevice : TalonFXFeedbackDevice.values())
        {
            // talonFX.configSelectedFeedbackSensor(talonFXFeedbackDevice, 0,
            // 0);
        }
        talonFX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(), 0);
        talonFX.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(), 0);
        talonFX.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(), 0);
        talonFX.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(), 0);
        talonFX.getSensorCollection();
        talonFX.getPIDConfigs(new TalonFXPIDSetConfiguration(), 0, 0);
        talonFX.getPIDConfigs(new TalonFXPIDSetConfiguration());
        talonFX.configAllSettings(new TalonFXConfiguration(), 0);
        talonFX.getAllConfigs(new TalonFXConfiguration(), 0);
        // talonFX.getAllConfigs(new TalonSRXConfiguration());
        for (MotorCommutation motorCommutation : MotorCommutation.values())
        {
            talonFX.configMotorCommutation(motorCommutation, 0);
        }
        talonFX.configGetMotorCommutation(0);
        for (AbsoluteSensorRange absoluteSensorRange : AbsoluteSensorRange.values())
        {
            // talonFX.configIntegratedSensorAbsoluteRange(absoluteSensorRange,
            // 0);
        }
        // talonFX.configIntegratedSensorOffset(0, 0);
        for (SensorInitializationStrategy sensorInitializationStrategy : SensorInitializationStrategy.values())
        {
            // talonFX.configIntegratedSensorInitializationStrategy(sensorInitializationStrategy,
            // 0);
        }

    }
}
