package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class TestCANCoderFunctions
{
    private CtreCallback mTestCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting CANCoder callback " + aName + "' with size of " + aBuffer.capacity() + ", " + aCount); // NOPMD
        }
    };

    @Test
    public void testAllFunctions()
    {
        CANCoder canCoder = new CANCoder(0);
        CtreJni.registerCanPigeonImuCallback(mTestCallback);

        canCoder.getPosition();
        canCoder.setPosition(0, 0);
        canCoder.setPosition(0);
        canCoder.setPositionToAbsolute(0);
        canCoder.setPositionToAbsolute();
        canCoder.getVelocity();
        canCoder.getAbsolutePosition();
        for (SensorVelocityMeasPeriod sensorVelocityMeasPeriod : SensorVelocityMeasPeriod.values())
        {
            canCoder.configVelocityMeasurementPeriod(sensorVelocityMeasPeriod, 0);
        }
        for (SensorVelocityMeasPeriod sensorVelocityMeasPeriod : SensorVelocityMeasPeriod.values())
        {
            canCoder.configVelocityMeasurementPeriod(sensorVelocityMeasPeriod);
        }
        canCoder.configVelocityMeasurementWindow(0, 0);
        canCoder.configVelocityMeasurementWindow(0);
        for (AbsoluteSensorRange absoluteSensorRange : AbsoluteSensorRange.values())
        {
            canCoder.configAbsoluteSensorRange(absoluteSensorRange, 0);
        }
        for (AbsoluteSensorRange absoluteSensorRange : AbsoluteSensorRange.values())
        {
            canCoder.configAbsoluteSensorRange(absoluteSensorRange);
        }
        canCoder.configMagnetOffset(0, 0);
        canCoder.configMagnetOffset(0);
        for (SensorInitializationStrategy sensorInitializationStrategy : SensorInitializationStrategy.values())
        {
            canCoder.configSensorInitializationStrategy(sensorInitializationStrategy, 0);
        }
        for (SensorInitializationStrategy sensorInitializationStrategy : SensorInitializationStrategy.values())
        {
            canCoder.configSensorInitializationStrategy(sensorInitializationStrategy);
        }
        for (SensorTimeBase sensorTimeBase : SensorTimeBase.values())
        {
            canCoder.configFeedbackCoefficient(0, "", sensorTimeBase, 0);
        }
        for (SensorTimeBase sensorTimeBase : SensorTimeBase.values())
        {
            canCoder.configFeedbackCoefficient(0, "", sensorTimeBase);
        }
        canCoder.getBusVoltage();
        canCoder.getMagnetFieldStrength();
        canCoder.configSensorDirection(false, 0);
        canCoder.configSensorDirection(false);
        canCoder.getLastError();
        canCoder.getLastUnitString();
        canCoder.getLastTimestamp();
        canCoder.configSetCustomParam(0, 0, 0);
        canCoder.configSetCustomParam(0, 0);
        canCoder.configGetCustomParam(0, 0);
        canCoder.configGetCustomParam(0);
        for (ParamEnum paramEnum : ParamEnum.values())
        {
            canCoder.configSetParameter(paramEnum, 0, 0, 0, 0);
        }
        for (ParamEnum paramEnum : ParamEnum.values())
        {
            canCoder.configSetParameter(paramEnum, 0, 0, 0);
        }
        for (ParamEnum paramEnum : ParamEnum.values())
        {
            canCoder.configGetParameter(paramEnum, 0, 0);
        }
        for (ParamEnum paramEnum : ParamEnum.values())
        {
            canCoder.configGetParameter(paramEnum, 0);
        }
        for (CANCoderStatusFrame canCoderStatusFrame : CANCoderStatusFrame.values())
        {
            canCoder.setStatusFramePeriod(canCoderStatusFrame, 0, 0);
        }
        for (CANCoderStatusFrame canCoderStatusFrame : CANCoderStatusFrame.values())
        {
            canCoder.setStatusFramePeriod(canCoderStatusFrame, 0);
        }
        for (CANCoderStatusFrame canCoderStatusFrame : CANCoderStatusFrame.values())
        {
            canCoder.getStatusFramePeriod(canCoderStatusFrame, 0);
        }
        for (CANCoderStatusFrame canCoderStatusFrame : CANCoderStatusFrame.values())
        {
            canCoder.getStatusFramePeriod(canCoderStatusFrame);
        }
        canCoder.getFirmwareVersion();
        canCoder.hasResetOccurred();
        canCoder.getFaults(new CANCoderFaults());
        canCoder.getStickyFaults(new CANCoderStickyFaults());
        canCoder.clearStickyFaults(0);
        canCoder.clearStickyFaults();
        canCoder.configAllSettings(new CANCoderConfiguration(), 0);
        canCoder.configAllSettings(new CANCoderConfiguration());
        canCoder.getAllConfigs(new CANCoderConfiguration(), 0);
        canCoder.getAllConfigs(new CANCoderConfiguration());
        canCoder.configFactoryDefault(0);
        canCoder.configFactoryDefault();

    }
}
