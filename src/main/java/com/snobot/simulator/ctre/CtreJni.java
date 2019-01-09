
package com.snobot.simulator.ctre;

public final class CtreJni
{
    private CtreJni()
    {

    }

    public static native void registerCanMotorCallback(CtreCallback aCallback);

    public static native void cancelCanMotorCallback();

    public static native void registerCanPigeonImuCallback(CtreCallback aCallback);

    public static native void cancelCanPigeonImuCallback();

    public static native void registerCanCanifierCallback(CtreCallback aCallback);

    public static native void cancelCanCanifierCallback();

    public static native void registerCanBuffTrajPointStreamCallback(CtreCallback aCallback);

    public static native void cancelCanCanBuffTrajPointStreamCallback();
}
