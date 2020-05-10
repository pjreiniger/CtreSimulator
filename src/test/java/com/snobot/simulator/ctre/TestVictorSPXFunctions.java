package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;

public class TestVictorSPXFunctions
{
    private CtreCallback mTestCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting VictorSPX callback " + aName + "' with size of " + aBuffer.capacity() + ", " + aCount); // NOPMD
        }
    };

    @Test
    public void testAllFunctions()
    {
        CtreJni.registerCanMotorCallback(mTestCallback);

        VictorSPX victor = new VictorSPX(0);
        victor.getPIDConfigs(new VictorSPXPIDSetConfiguration(), 0, 0);
        victor.getPIDConfigs(new VictorSPXPIDSetConfiguration());
        victor.configAllSettings(new VictorSPXConfiguration(), 0);
        victor.configAllSettings(new VictorSPXConfiguration());
        victor.getAllConfigs(new VictorSPXConfiguration(), 0);
        victor.getAllConfigs(new VictorSPXConfiguration());

    }
}
