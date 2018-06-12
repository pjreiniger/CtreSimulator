package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;
import com.ctre.phoenix.sensors.PigeonIMU.GeneralStatus;
import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;
import com.ctre.phoenix.sensors.PigeonIMU_Faults;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU_StickyFaults;

@Tag("CTRE")
public class TestPigeonFunctions
{
	private CtreCallback mTestCallback = new CtreCallback() {

		@Override
		public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount) {
			System.out.println("Getting callback " + aName);
		}
	};

    @Test
    public void testAllFunctions()
    {
        PigeonIMU imu = new PigeonIMU(0);
        CtreJni.registerCanPigeonImuCallback(mTestCallback);

        imu.setYaw(0, 0);
        imu.addYaw(0, 0);
        imu.setYawToCompass(0);
        imu.setFusedHeading(0, 0);
        imu.addFusedHeading(0, 0);
        imu.setFusedHeadingToCompass(0);
        imu.setAccumZAngle(0, 0);
        imu.configTemperatureCompensationEnable(true, 0);
        imu.configTemperatureCompensationEnable(false, 0);
        imu.setCompassDeclination(0, 0);
        imu.setCompassAngle(0, 0);
        for(CalibrationMode calMode : CalibrationMode.values())
        {
        	imu.enterCalibrationMode(calMode, 0);
        }
    	imu.getGeneralStatus(new GeneralStatus());
        imu.getLastError();
        imu.get6dQuaternion(new double[4]);
        imu.getYawPitchRoll(new double[3]);
        imu.getAccumGyro(new double[3]);
        imu.getAbsoluteCompassHeading();
        imu.getCompassHeading();
        imu.getCompassFieldStrength();
        imu.getTemp();
        imu.getState();
        imu.getUpTime();
        imu.getRawMagnetometer(new short[3]);
        imu.getBiasedMagnetometer(new short[3]);
        imu.getBiasedAccelerometer(new short[3]);
        imu.getRawGyro(new double[3]);
        imu.getAccelerometerAngles(new double[3]);
        imu.getFusedHeading(new FusionStatus());
        imu.getFusedHeading();
        imu.getFirmwareVersion();
        imu.hasResetOccurred();
        imu.configSetCustomParam(0, 0, 0);
        imu.configGetCustomParam(0, 0);
        for(ParamEnum paramEnum : ParamEnum.values())
        {
        	imu.configSetParameter(paramEnum, 0, 0, 0, 0);
        }
        imu.configSetParameter(0, 0, 0, 0, 0);
        for(ParamEnum paramEnum : ParamEnum.values())
        {
        	imu.configGetParameter(paramEnum, 0, 0);
        }
        imu.configGetParameter(0, 0, 0);
        for(PigeonIMU_StatusFrame statusFrame : PigeonIMU_StatusFrame.values())
        {
            imu.setStatusFramePeriod(statusFrame, 0, 0);
        }
        imu.setStatusFramePeriod(0, 0, 0);
        for(PigeonIMU_StatusFrame statusFrame : PigeonIMU_StatusFrame.values())
        {
            imu.getStatusFramePeriod(statusFrame, 0);
        }
        for(PigeonIMU_ControlFrame statusFrame : PigeonIMU_ControlFrame.values())
        {
            imu.setControlFramePeriod(statusFrame, 0);
        }
        imu.setControlFramePeriod(0, 0);
        imu.getFaults(new PigeonIMU_Faults());
        imu.getStickyFaults(new PigeonIMU_StickyFaults());
        imu.clearStickyFaults(0);
    }

}
