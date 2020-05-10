/*
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files ( *.crf) and Software
 * API Libraries ONLY when in use with Cross The Road Electronics hardware products.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
package com.ctre.phoenix.sensors;
import com.ctre.phoenix.CTREJNIWrapper;

public class CANCoderJNI extends CTREJNIWrapper {

    public static native long Create(int deviceNumber);

    public static native int Destroy(long handle);

    public static native int GetLastError(long handle);

    public static native String GetLastUnitString(long handle);

    public static native double GetLastTimestamp(long handle);

    public static native double GetBusVoltage(long handle);

    public static native int GetMagnetFieldStrength(long handle);

    public static native double GetPosition(long handle);

    public static native int SetPosition(long handle, double pos, int timeoutMs);

    public static native int SetPositionToAbsolute(long handle, int timeoutMs);

    public static native int ConfigSensorDirection(long handle, int bDirection, int timeoutMs);

    public static native double GetVelocity(long handle);

    public static native double GetAbsolutePosition(long handle);

    public static native int ConfigVelocityMeasurementPeriod(long handle, int period, int timeoutMs);

    public static native int ConfigVelocityMeasurementWindow(long handle, int window, int timeoutMs);

    public static native int ConfigAbsoluteSensorRange(long handle, int absoluteSensorRante, int timeoutMs);

    public static native int ConfigMagnetOffset(long handle, double offsetDegrees, int timeoutMs);

    public static native int ConfigSensorInitializationStrategy(long handle, int initStrategy, int timeoutMs);

    public static native int ConfigFeedbackCoefficient(long handle, double sensorCoefficient, String unitString, int sensortimeBase, int timeoutMs);

    public static native int ConfigSetParameter(long handle, int param, double value, int subValue, int ordinal, int timeoutMs);

    public static native double ConfigGetParameter(long handle, int param, int ordinal, int timeoutMs);

    //public static native double ConfigGetParameter_6(long handle, int param, int valueToSend, int subValue, int ordinal, int timeoutMs);

	public static native int ConfigSetCustomParam(long handle, int newValue, int paramIndex, int timeoutMs);

	public static native int ConfigGetCustomParam(long handle, int paramIndex, int timoutMs);
	
    public static native int ConfigFactoryDefault(long handle, int timeoutMs);

	public static native int GetFaults(long handle);

	public static native int GetStickyFaults(long handle);

	public static native int ClearStickyFaults(long handle, int timeoutMs);
	
	public static native int GetFirmwareVersion(long handle);
	  
	public static native boolean HasResetOccurred(long handle);

	public static native int SetStatusFramePeriod(long handle, int statusFrame, int periodMs, int timeoutMs);
	
	public static native int GetStatusFramePeriod(long handle, int frame, int timeoutMs);
}