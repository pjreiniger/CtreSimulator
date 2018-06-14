package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.CANifier.PinValues;
import com.ctre.phoenix.CANifierControlFrame;
import com.ctre.phoenix.CANifierFaults;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.CANifierStickyFaults;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TestCanifierFunctions {

	private CtreCallback mTestCallback = new CtreCallback() {

		@Override
		public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount) {
			System.out.println("Getting callback " + aName);
		}
	};

    @Test
    public void testAllFunctions()
    {
        CANifier canifier = new CANifier(0);
        CtreJni.registerCanCanifierCallback(mTestCallback);


        for(LEDChannel ledChannel : LEDChannel.values())
        {
            canifier.setLEDOutput(0, ledChannel);
        }
        for(GeneralPin generalPin : GeneralPin.values())
        {
            canifier.setGeneralOutput(generalPin, false, false);
        }
        canifier.setGeneralOutputs(0, 0);
        canifier.getGeneralInputs(new PinValues());
        for(GeneralPin generalPin : GeneralPin.values())
        {
            canifier.getGeneralInput(generalPin);
        }
        canifier.getLastError();
        canifier.setPWMOutput(0, 0);
        canifier.enablePWMOutput(0, false);
        for(PWMChannel pwmChannel : PWMChannel.values())
        {
            canifier.getPWMInput(pwmChannel, new double[2]);
        }
        canifier.configSetCustomParam(0, 0, 0);
        canifier.configGetCustomParam(0, 0);
        for(ParamEnum paramEnum : ParamEnum.values())
        {
            canifier.configSetParameter(paramEnum, 0, 0, 0, 0);
        }
        canifier.configSetParameter(0, 0, 0, 0, 0);
        for(ParamEnum paramEnum : ParamEnum.values())
        {
            canifier.configGetParameter(paramEnum, 0, 0);
        }
        for(CANifierStatusFrame canifierStatusFrame : CANifierStatusFrame.values())
        {
            canifier.setStatusFramePeriod(canifierStatusFrame, 0, 0);
        }
        canifier.setStatusFramePeriod(0, 0, 0);
        for(CANifierStatusFrame canifierStatusFrame : CANifierStatusFrame.values())
        {
            canifier.getStatusFramePeriod(canifierStatusFrame, 0);
        }
        for(CANifierControlFrame canifierControlFrame : CANifierControlFrame.values())
        {
            canifier.setControlFramePeriod(canifierControlFrame, 0);
        }
        canifier.setControlFramePeriod(0, 0);
        canifier.getFirmwareVersion();
        canifier.hasResetOccurred();
        canifier.getFaults(new CANifierFaults());
        canifier.getStickyFaults(new CANifierStickyFaults());
        canifier.clearStickyFaults(0);
        canifier.getBusVoltage();



    }

}
