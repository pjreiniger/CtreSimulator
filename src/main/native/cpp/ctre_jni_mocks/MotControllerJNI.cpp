
#include <jni.h>

#include <cassert>

#include "com_ctre_phoenix_motorcontrol_can_MotControllerJNI.h"
#include "ctre/phoenix/cci/MotController_CCI.h"
#include "CtreSimMocks/CtreMotorControllerWrapper.h"
#include "CtreSimMocks/MockHookUtilities.h"


SnobotSim::CtreMotorControllerWrapper* ConvertToMotorControllerWrapper(jlong aHandle)
{
    return reinterpret_cast<SnobotSim::CtreMotorControllerWrapper*>(aHandle);
}

extern "C" {

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Create
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Create
  (JNIEnv *, jclass, jint baseArbId)
{
    return (jlong)c_MotController_Create1(baseArbId);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Create2
 * Signature: (Ijava/lang/String;)J
 */
JNIEXPORT jlong JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Create2
  (JNIEnv *, jclass, jint deviceID, jstring)
{
    return (jlong)c_MotController_Create2(deviceID, "");
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    JNI_destroy_MotController
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_JNI_1destroy_1MotController
  (JNIEnv *, jclass, jlong handle)
{
    return (jint) c_MotController_Destroy(ConvertToMotorControllerWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetDeviceNumber
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetDeviceNumber
  (JNIEnv *, jclass, jlong handle)
{
    int deviceNumber = 0;
    c_MotController_GetDeviceNumber(ConvertToMotorControllerWrapper(handle), &deviceNumber);
    return deviceNumber;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetBaseID
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetBaseID
  (JNIEnv *, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetDemand
 * Signature: (JIII)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetDemand
  (JNIEnv *, jclass, jlong handle, jint mode, jint demand0, jint demand1)
{
    c_MotController_SetDemand(ConvertToMotorControllerWrapper(handle), mode, demand0, demand1);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Set_4
 * Signature: (JIDDI)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Set_14
  (JNIEnv *, jclass, jlong handle, jint mode, jdouble demand0, jdouble demand1, jint demand1Type)
{
	c_MotController_Set_4(ConvertToMotorControllerWrapper(handle), mode, demand0, demand1, demand1Type);
}


/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetNeutralMode
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetNeutralMode
  (JNIEnv *, jclass, jlong handle, jint neutralMode)
{
    c_MotController_SetNeutralMode(ConvertToMotorControllerWrapper(handle), neutralMode);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetSensorPhase
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetSensorPhase
  (JNIEnv *, jclass, jlong handle, jboolean phaseSensor)
{
    c_MotController_SetSensorPhase(ConvertToMotorControllerWrapper(handle), phaseSensor);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetInverted
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetInverted
  (JNIEnv *, jclass, jlong handle, jboolean inverted)
{
    c_MotController_SetInverted(ConvertToMotorControllerWrapper(handle), inverted);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetInverted_2
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetInverted_12
  (JNIEnv *, jclass, jlong handle, jint inverted)
{
    c_MotController_SetInverted_2(ConvertToMotorControllerWrapper(handle), inverted);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigFactoryDefault
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigFactoryDefault
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    return c_MotController_ConfigFactoryDefault(ConvertToMotorControllerWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigOpenLoopRamp
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigOpenLoopRamp
  (JNIEnv *, jclass, jlong handle, jdouble secondsFromNeutralToFull, jint timeoutMs)
{
    return (jint)c_MotController_ConfigOpenLoopRamp(ConvertToMotorControllerWrapper(handle), secondsFromNeutralToFull, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClosedLoopRamp
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClosedLoopRamp
  (JNIEnv *, jclass, jlong handle, jdouble secondsFromNeutralToFull, jint timeoutMs)
{
    return (jint)c_MotController_ConfigClosedLoopRamp(ConvertToMotorControllerWrapper(handle), secondsFromNeutralToFull, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPeakOutputForward
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPeakOutputForward
  (JNIEnv *, jclass, jlong handle, jdouble percentOut, jint timeoutMs)
{
    return (jint)c_MotController_ConfigPeakOutputForward(ConvertToMotorControllerWrapper(handle), percentOut, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPeakOutputReverse
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPeakOutputReverse
  (JNIEnv *, jclass, jlong handle, jdouble percentOut, jint timeoutMs)
{
    return (jint)c_MotController_ConfigPeakOutputReverse(ConvertToMotorControllerWrapper(handle), percentOut, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigNominalOutputForward
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigNominalOutputForward
  (JNIEnv *, jclass, jlong handle, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigNominalOutputForward(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigNominalOutputReverse
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigNominalOutputReverse
  (JNIEnv *, jclass, jlong handle, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigNominalOutputReverse(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigNeutralDeadband
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigNeutralDeadband
  (JNIEnv *, jclass, jlong handle, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigNeutralDeadband(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigVoltageCompSaturation
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigVoltageCompSaturation
  (JNIEnv *, jclass, jlong handle, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigVoltageCompSaturation(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigVoltageMeasurementFilter
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigVoltageMeasurementFilter
  (JNIEnv *, jclass, jlong handle, jint value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigVoltageMeasurementFilter(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    EnableVoltageCompensation
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_EnableVoltageCompensation
  (JNIEnv *, jclass, jlong handle, jboolean value)
{
    c_MotController_EnableVoltageCompensation(ConvertToMotorControllerWrapper(handle), value);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetInverted
 * Signature: (J)B
 */
JNIEXPORT jboolean JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetInverted
  (JNIEnv *, jclass, jlong handle)
{
    bool inverted = false;
    c_MotController_GetInverted(ConvertToMotorControllerWrapper(handle), &inverted);
    return inverted;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetBusVoltage
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetBusVoltage
  (JNIEnv *, jclass, jlong handle)
{
    double output = 0;
    c_MotController_GetBusVoltage(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetMotorOutputPercent
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotorOutputPercent
  (JNIEnv *, jclass, jlong handle)
{
    double percentage = 0;
    c_MotController_GetMotorOutputPercent(ConvertToMotorControllerWrapper(handle), &percentage);
    return percentage;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetOutputCurrent
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetOutputCurrent
  (JNIEnv *, jclass, jlong handle)
{
    double output = 0;
    c_MotController_GetOutputCurrent(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}


/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetSupplyCurrent
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetSupplyCurrent
  (JNIEnv *, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetStatorCurrent
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetStatorCurrent
  (JNIEnv *, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetTemperature
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetTemperature
  (JNIEnv *, jclass, jlong handle)
{
    double output = 0;
    c_MotController_GetTemperature(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigRemoteFeedbackFilter
 * Signature: (JIIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigRemoteFeedbackFilter
  (JNIEnv *, jclass, jlong handle, jint arbId, jint peripheralIdx, jint reserved, jint timeoutMs)
{
    return (jint)c_MotController_ConfigRemoteFeedbackFilter(ConvertToMotorControllerWrapper(handle), arbId, peripheralIdx, reserved, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSelectedFeedbackSensor
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSelectedFeedbackSensor
  (JNIEnv *, jclass, jlong handle, jint feedbackDevice, jint pidIdx, jint timeoutMs)
{
    return (jint)c_MotController_ConfigSelectedFeedbackSensor(ConvertToMotorControllerWrapper(handle), feedbackDevice, pidIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSensorTerm
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSensorTerm
  (JNIEnv *, jclass, jlong handle, jint sensorTerm, jint feedbackDevice, jint timeoutMs)
{
	return (int) c_MotController_ConfigSensorTerm(ConvertToMotorControllerWrapper(handle), sensorTerm, feedbackDevice, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetSelectedSensorPosition
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetSelectedSensorPosition
  (JNIEnv *, jclass, jlong handle, jint pidIdx)
{
    int output = 0;
    c_MotController_GetSelectedSensorPosition(ConvertToMotorControllerWrapper(handle), &output, pidIdx);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetSelectedSensorVelocity
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetSelectedSensorVelocity
  (JNIEnv *, jclass, jlong handle, jint pidIdx)
{
    int output = 0;
    c_MotController_GetSelectedSensorVelocity(ConvertToMotorControllerWrapper(handle), &output, pidIdx);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetSelectedSensorPosition
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetSelectedSensorPosition
  (JNIEnv *, jclass, jlong handle, jint value, jint pidIdx, jint timeoutMs)
{
    return (jint)c_MotController_SetSelectedSensorPosition(ConvertToMotorControllerWrapper(handle), value, pidIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetControlFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetControlFramePeriod
  (JNIEnv *, jclass, jlong handle, jint value, jint timeoutMs)
{
    return (jint)c_MotController_SetControlFramePeriod(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetStatusFramePeriod
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetStatusFramePeriod
  (JNIEnv *, jclass, jlong handle, jint frame, jint periodMs, jint timeoutMs)
{
    return (jint)c_MotController_SetStatusFramePeriod(ConvertToMotorControllerWrapper(handle), frame, periodMs, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetStatusFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetStatusFramePeriod
  (JNIEnv *, jclass, jlong handle, jint frame, jint timeoutMs)
{
    int status = 0;
    c_MotController_GetStatusFramePeriod(ConvertToMotorControllerWrapper(handle), frame, &status, timeoutMs);
    return status;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigVelocityMeasurementPeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigVelocityMeasurementPeriod
  (JNIEnv *, jclass, jlong handle, jint value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigVelocityMeasurementPeriod(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigVelocityMeasurementWindow
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigVelocityMeasurementWindow
  (JNIEnv *, jclass, jlong handle, jint value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigVelocityMeasurementWindow(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigForwardLimitSwitchSource
 * Signature: (JIIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigForwardLimitSwitchSource
  (JNIEnv *, jclass, jlong handle, jint type, jint normalOpenOrClose, jint deviceId, jint timeoutMs)
{
    return (jint)c_MotController_ConfigForwardLimitSwitchSource(ConvertToMotorControllerWrapper(handle), type, normalOpenOrClose, deviceId, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigReverseLimitSwitchSource
 * Signature: (JIIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigReverseLimitSwitchSource
  (JNIEnv *, jclass, jlong handle, jint type, jint normalOpenOrClose, jint deviceID, jint timeoutMs)
{
    return (jint)c_MotController_ConfigReverseLimitSwitchSource(ConvertToMotorControllerWrapper(handle), type, normalOpenOrClose, deviceID, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    OverrideLimitSwitchesEnable
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_OverrideLimitSwitchesEnable
  (JNIEnv *, jclass, jlong aHandle, jboolean aEnable)
{
    c_MotController_OverrideLimitSwitchesEnable(ConvertToMotorControllerWrapper(aHandle), aEnable);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigForwardSoftLimitThreshold
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigForwardSoftLimitThreshold
  (JNIEnv *, jclass, jlong handle, jint forwardSensorLimit, jint timeoutMs)
{
	return (int) c_MotController_ConfigForwardSoftLimitThreshold(ConvertToMotorControllerWrapper(handle), forwardSensorLimit, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigReverseSoftLimitThreshold
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigReverseSoftLimitThreshold
  (JNIEnv *, jclass, jlong handle, jint value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigReverseSoftLimitThreshold(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigForwardSoftLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigForwardSoftLimitEnable
  (JNIEnv *, jclass, jlong handle, jboolean value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigForwardSoftLimitEnable(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigReverseSoftLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigReverseSoftLimitEnable
  (JNIEnv *, jclass, jlong aHandle, jboolean aEnable, jint aTimeout)
{
    return (jint) c_MotController_ConfigReverseSoftLimitEnable(ConvertToMotorControllerWrapper(aHandle), aEnable, aTimeout);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    OverrideSoftLimitsEnable
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_OverrideSoftLimitsEnable
  (JNIEnv *, jclass, jlong handle, jboolean value)
{
    c_MotController_OverrideSoftLimitsEnable(ConvertToMotorControllerWrapper(handle), value);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Config_kP
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1kP
  (JNIEnv *, jclass, jlong handle, jint slot, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_Config_kP(ConvertToMotorControllerWrapper(handle), slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Config_kI
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1kI
  (JNIEnv *, jclass, jlong handle, jint slot, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_Config_kI(ConvertToMotorControllerWrapper(handle), slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Config_kD
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1kD
  (JNIEnv *, jclass, jlong handle, jint slot, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_Config_kD(ConvertToMotorControllerWrapper(handle), slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Config_kF
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1kF
  (JNIEnv *, jclass, jlong handle, jint slot, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_Config_kF(ConvertToMotorControllerWrapper(handle), slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Config_IntegralZone
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1IntegralZone
  (JNIEnv *, jclass, jlong handle, jint slot, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_Config_IntegralZone(ConvertToMotorControllerWrapper(handle), slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigAllowableClosedloopError
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigAllowableClosedloopError
  (JNIEnv *, jclass, jlong handle, jint slotIdx, jint allowableClosedLoopError, jint timeoutMs)
{
    return (jint)c_MotController_ConfigAllowableClosedloopError(ConvertToMotorControllerWrapper(handle), slotIdx, allowableClosedLoopError, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMaxIntegralAccumulator
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMaxIntegralAccumulator
  (JNIEnv *, jclass, jlong handle, jint slotIdx, jdouble iaccum, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMaxIntegralAccumulator(ConvertToMotorControllerWrapper(handle), slotIdx, iaccum, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetIntegralAccumulator
 * Signature: (JDII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetIntegralAccumulator
  (JNIEnv *, jclass, jlong handle, jdouble value, jint pidIdx, jint timeoutMs)
{
    return (jint)c_MotController_SetIntegralAccumulator(ConvertToMotorControllerWrapper(handle), value, pidIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetClosedLoopError
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetClosedLoopError
  (JNIEnv *, jclass, jlong handle, jint slotIdx)
{
    int closedLoopError = 0;
    c_MotController_GetClosedLoopError(ConvertToMotorControllerWrapper(handle), &closedLoopError, slotIdx);
    return closedLoopError;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetIntegralAccumulator
 * Signature: (JI)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetIntegralAccumulator
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    double output = 0;
    c_MotController_GetIntegralAccumulator(ConvertToMotorControllerWrapper(handle), &output, timeoutMs);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetErrorDerivative
 * Signature: (JI)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetErrorDerivative
  (JNIEnv *, jclass, jlong handle, jint slotIdx)
{
    double output = 0;
    c_MotController_GetErrorDerivative(ConvertToMotorControllerWrapper(handle), &output, slotIdx);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SelectProfileSlot
 * Signature: (JII)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SelectProfileSlot
  (JNIEnv *, jclass, jlong handle, jint slotIdx, jint pidIdx)
{
    c_MotController_SelectProfileSlot(ConvertToMotorControllerWrapper(handle), slotIdx, pidIdx);
}


/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryPosition3
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryPosition3
  (JNIEnv *, jclass, jlong handle, jint pidIdx)
{
    int output = 0;
    c_MotController_GetActiveTrajectoryPosition_3(ConvertToMotorControllerWrapper(handle), &output, pidIdx);
    return output;
}


/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryVelocity3
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryVelocity3
  (JNIEnv *, jclass, jlong handle, jint pidIdx)
{
    int output = 0;
    c_MotController_GetActiveTrajectoryVelocity_3(ConvertToMotorControllerWrapper(handle), &output, pidIdx);
    return output;
}


/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryArbFeedFwd3
 * Signature: (JI)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryArbFeedFwd3
  (JNIEnv *, jclass, jlong handle, jint pidIdx)
{
    double output = 0;
    c_MotController_GetActiveTrajectoryArbFeedFwd_3(ConvertToMotorControllerWrapper(handle), &output, pidIdx);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryPosition
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryPosition
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetActiveTrajectoryPosition(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryVelocity
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryVelocity
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetActiveTrajectoryVelocity(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryHeading
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryHeading
  (JNIEnv *, jclass, jlong handle)
{
    double output = 0;
    c_MotController_GetActiveTrajectoryHeading(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionCruiseVelocity
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionCruiseVelocity
  (JNIEnv *, jclass, jlong handle, jint sensorUnitsPer100ms, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMotionCruiseVelocity(ConvertToMotorControllerWrapper(handle), sensorUnitsPer100ms, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionAcceleration
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionAcceleration
  (JNIEnv *, jclass, jlong handle, jint sensorUnitsPer100msPerSec, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMotionAcceleration(ConvertToMotorControllerWrapper(handle), sensorUnitsPer100msPerSec, timeoutMs);
}


/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionSCurveStrength
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionSCurveStrength
  (JNIEnv *, jclass, jlong handle, jint curveStrength, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMotionSCurveStrength(ConvertToMotorControllerWrapper(handle), curveStrength, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ClearMotionProfileTrajectories
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ClearMotionProfileTrajectories
  (JNIEnv *, jclass, jlong handle)
{
    return (jint)c_MotController_ClearMotionProfileTrajectories(ConvertToMotorControllerWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetMotionProfileTopLevelBufferCount
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotionProfileTopLevelBufferCount
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetMotionProfileTopLevelBufferCount(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    PushMotionProfileTrajectory
 * Signature: (JDDDIZZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_PushMotionProfileTrajectory
  (JNIEnv *, jclass, jlong handle, jdouble position, jdouble velocity, jdouble headingDeg, jint profileSlotSelect, jboolean isLastPoint, jboolean zeroPos)
{
    return (jint)c_MotController_PushMotionProfileTrajectory(ConvertToMotorControllerWrapper(handle), position, velocity, headingDeg, profileSlotSelect, isLastPoint, zeroPos);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    PushMotionProfileTrajectory2
 * Signature: (JDDDIIZZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_PushMotionProfileTrajectory2
  (JNIEnv *, jclass, jlong handle, jdouble position, jdouble velocity, jdouble headingDeg,
        jint profileSlotSelect0, jint profileSlotSelect1, jboolean isLastPoint, jboolean zeroPos, jint durationMs)
{
    return (jint) c_MotController_PushMotionProfileTrajectory_2(
            ConvertToMotorControllerWrapper(handle), position, velocity, headingDeg,
            profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos, durationMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    PushMotionProfileTrajectory3
 * Signature: (JDDDDDDIIZZIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_PushMotionProfileTrajectory3
  (JNIEnv *, jclass, jlong handle, jdouble position, jdouble velocity,
          jdouble arbFeedFwd, jdouble auxiliaryPos, jdouble auxiliaryVel, jdouble auxiliaryArbFeedFwd,
          jint profileSlotSelect0, jint profileSlotSelect1, jboolean isLastPoint, jboolean zeroPos0, jint timeDur, jboolean useAuxPID)
{
    return (jint) c_MotController_PushMotionProfileTrajectory_3(
            ConvertToMotorControllerWrapper(handle), position, velocity,
            arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd,
            profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos0, timeDur, useAuxPID);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    StartMotionProfile
 * Signature: (JJII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_StartMotionProfile
  (JNIEnv *, jclass, jlong handle, jlong streamHandle, jint minBufferedPts, jint controlMode)
{
    c_MotController_StartMotionProfile(ConvertToMotorControllerWrapper(handle), NULL, minBufferedPts, (ctre::phoenix::motorcontrol::ControlMode) controlMode);
    return 0;
}


/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    IsMotionProfileTopLevelBufferFull
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_IsMotionProfileTopLevelBufferFull
  (JNIEnv *, jclass, jlong handle)
{
    bool output = 0;
    c_MotController_IsMotionProfileTopLevelBufferFull(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    IsMotionProfileFinished
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_IsMotionProfileFinished
  (JNIEnv *, jclass, jlong handle)
{
    bool output = false;
    c_MotController_IsMotionProfileTopLevelBufferFull(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ProcessMotionProfileBuffer
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ProcessMotionProfileBuffer
  (JNIEnv *, jclass, jlong handle)
{
    return (jint)c_MotController_ProcessMotionProfileBuffer(ConvertToMotorControllerWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetMotionProfileStatus
 * Signature: (J[I)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotionProfileStatus
  (JNIEnv * env, jclass, jlong handle, jintArray result)
{
    static const int kSize = 9;
    int output[kSize];

    size_t topBufferRem;
    size_t topBufferCnt;
    bool hasUnderrun = false;
    bool isUnderrun = false;
    bool activePointValid = false;
    bool isLast = false;

    c_MotController_GetMotionProfileStatus(ConvertToMotorControllerWrapper(handle),
            &topBufferRem, &topBufferCnt, &output[2],
            &hasUnderrun, &isUnderrun, &activePointValid,
            &isLast, &output[7], &output[8]);

    output[0] = topBufferRem;
    output[1] = topBufferCnt;
    output[3] = hasUnderrun;
    output[4] = isUnderrun;
    output[5] = activePointValid;
    output[6] = isLast;

    jint fill[kSize];
    for (int i = 0; i < kSize; ++i)
    {
        fill[i] = output[i];
    }

    env->SetIntArrayRegion(result, 0, kSize, fill);

    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetMotionProfileStatus2
 * Signature: (J[I)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotionProfileStatus2
  (JNIEnv* env, jclass, jlong handle, jintArray result)
{
    static const int kSize = 11;
    int output[kSize];

    size_t topBufferRem;
    size_t topBufferCnt;
    bool hasUnderrun = false;
    bool isUnderrun = false;
    bool activePointValid = false;
    bool isLast = false;

    c_MotController_GetMotionProfileStatus_2(ConvertToMotorControllerWrapper(handle),
            &topBufferRem, &topBufferCnt, &output[2],
            &hasUnderrun, &isUnderrun, &activePointValid,
            &isLast, &output[7], &output[8],
            &output[9], &output[10]);

    output[0] = topBufferRem;
    output[1] = topBufferCnt;
    output[3] = hasUnderrun;
    output[4] = isUnderrun;
    output[5] = activePointValid;
    output[6] = isLast;

    jint fill[kSize];
    for (int i = 0; i < kSize; ++i)
    {
        fill[i] = output[i];
    }

    env->SetIntArrayRegion(result, 0, kSize, fill);

    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ClearMotionProfileHasUnderrun
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ClearMotionProfileHasUnderrun
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_MotController_ClearMotionProfileHasUnderrun(ConvertToMotorControllerWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ChangeMotionControlFramePeriod
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ChangeMotionControlFramePeriod
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_MotController_ChangeMotionControlFramePeriod(ConvertToMotorControllerWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionProfileTrajectoryPeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionProfileTrajectoryPeriod
  (JNIEnv*, jclass, jlong handle, jint durationMs, jint timeoutMs)
{
    return (jint) c_MotController_ConfigMotionProfileTrajectoryPeriod(ConvertToMotorControllerWrapper(handle), durationMs, timeoutMs);
}


/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionProfileTrajectoryInterpolationEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionProfileTrajectoryInterpolationEnable
  (JNIEnv *, jclass, jlong handle, jboolean enable, jint timeoutMs)
{
    return c_MotController_ConfigMotionProfileTrajectoryInterpolationEnable(ConvertToMotorControllerWrapper(handle), enable, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigFeedbackNotContinuous
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigFeedbackNotContinuous
  (JNIEnv *, jclass, jlong handle, jboolean feedbackNotContinuous, jint timeoutMs)
{
    return c_MotController_ConfigFeedbackNotContinuous(ConvertToMotorControllerWrapper(handle), feedbackNotContinuous, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigRemoteSensorClosedLoopDisableNeutralOnLOS
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS
  (JNIEnv *, jclass, jlong handle, jboolean remoteSensorClosedLoopDisableNeutralOnLOS, jint timeoutMs)
{
    return c_MotController_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(ConvertToMotorControllerWrapper(handle),
            remoteSensorClosedLoopDisableNeutralOnLOS, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClearPositionOnLimitF
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClearPositionOnLimitF
  (JNIEnv *, jclass, jlong handle, jboolean clearPositionOnLimitF, jint timeoutMs)
{
    return c_MotController_ConfigClearPositionOnLimitF(ConvertToMotorControllerWrapper(handle),
            clearPositionOnLimitF, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClearPositionOnLimitR
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClearPositionOnLimitR
  (JNIEnv *, jclass, jlong handle, jboolean clearPositionOnLimitF, jint timeoutMs)
{
    return c_MotController_ConfigClearPositionOnLimitR(ConvertToMotorControllerWrapper(handle),
            clearPositionOnLimitF, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClearPositionOnQuadIdx
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClearPositionOnQuadIdx
  (JNIEnv *, jclass, jlong handle, jboolean clearPositionOnQuadIdx, jint timeoutMs)
{
    return c_MotController_ConfigClearPositionOnQuadIdx(ConvertToMotorControllerWrapper(handle),
            clearPositionOnQuadIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigLimitSwitchDisableNeutralOnLOS
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigLimitSwitchDisableNeutralOnLOS
  (JNIEnv *, jclass, jlong handle, jboolean limitSwitchDisableNeutralOnLOS, jint timeoutMs)
{
    return c_MotController_ConfigLimitSwitchDisableNeutralOnLOS(ConvertToMotorControllerWrapper(handle),
            limitSwitchDisableNeutralOnLOS, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSoftLimitDisableNeutralOnLOS
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSoftLimitDisableNeutralOnLOS
  (JNIEnv *, jclass, jlong handle, jboolean limitSwitchDisableNeutralOnLOS, jint timeoutMs)
{
    return c_MotController_ConfigSoftLimitDisableNeutralOnLOS(ConvertToMotorControllerWrapper(handle),
            limitSwitchDisableNeutralOnLOS, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPulseWidthPeriod_EdgesPerRot
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPulseWidthPeriod_1EdgesPerRot
  (JNIEnv *, jclass, jlong handle, jint pulseWidthPeriod_EdgesPerRot, jint timeoutMs)
{
    return c_MotController_ConfigPulseWidthPeriod_EdgesPerRot(ConvertToMotorControllerWrapper(handle),
            pulseWidthPeriod_EdgesPerRot, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPulseWidthPeriod_FilterWindowSz
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPulseWidthPeriod_1FilterWindowSz
  (JNIEnv *, jclass, jlong handle, jint pulseWidthPeriod_FilterWindowSz, jint timeoutMs)
{
    return c_MotController_ConfigPulseWidthPeriod_FilterWindowSz(ConvertToMotorControllerWrapper(handle),
            pulseWidthPeriod_FilterWindowSz, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetLastError
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetLastError
  (JNIEnv *, jclass, jlong handle)
{
    return (jint)c_MotController_GetLastError(ConvertToMotorControllerWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetFirmwareVersion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetFirmwareVersion
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetFirmwareVersion(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    HasResetOccurred
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_HasResetOccurred
  (JNIEnv *, jclass, jlong handle)
{
    bool value = false;
    c_MotController_HasResetOccurred(ConvertToMotorControllerWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSetCustomParam
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSetCustomParam
  (JNIEnv *, jclass, jlong handle, jint newValue, jint paramIndex, jint timeoutMs)
{
    return (jint)c_MotController_ConfigSetCustomParam(ConvertToMotorControllerWrapper(handle), newValue, paramIndex, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetCustomParam
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetCustomParam
  (JNIEnv *, jclass, jlong handle, jint paramIndex, jint timeoutMs)
{
    int output = 0;
    c_MotController_ConfigGetCustomParam(ConvertToMotorControllerWrapper(handle), &output, paramIndex, timeoutMs);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSetParameter
 * Signature: (JIDIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSetParameter
  (JNIEnv *, jclass, jlong handle, jint param, jdouble value, jint subValue, jint ordinal, jint timeoutMs)
{
    return (jint)c_MotController_ConfigSetParameter(ConvertToMotorControllerWrapper(handle), param, value, subValue, ordinal, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetParameter
 * Signature: (JIII)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetParameter
  (JNIEnv *, jclass, jlong handle, jint param, jint ordinal, jint timeoutMs)
{
    double output = 0;
    c_MotController_ConfigGetParameter(ConvertToMotorControllerWrapper(handle), param, &output, ordinal, timeoutMs);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPeakCurrentLimit
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPeakCurrentLimit
  (JNIEnv *, jclass, jlong handle, jint value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigPeakCurrentLimit(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPeakCurrentDuration
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPeakCurrentDuration
  (JNIEnv *, jclass, jlong handle, jint value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigPeakCurrentDuration(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigContinuousCurrentLimit
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigContinuousCurrentLimit
  (JNIEnv *, jclass, jlong handle, jint value, jint timeoutMs)
{
    return (jint)c_MotController_ConfigContinuousCurrentLimit(ConvertToMotorControllerWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    EnableCurrentLimit
 * Signature: (JZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_EnableCurrentLimit
  (JNIEnv *, jclass, jlong handle, jboolean value)
{
    return (jint) c_MotController_EnableCurrentLimit(ConvertToMotorControllerWrapper(handle), value);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetAnalogIn
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetAnalogIn
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetAnalogIn(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetAnalogPosition
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetAnalogPosition
  (JNIEnv *, jclass, jlong handle, jint newPosition, jint timeoutMs)
{
    return (jint) c_MotController_SetAnalogPosition(ConvertToMotorControllerWrapper(handle), newPosition, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetAnalogInRaw
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetAnalogInRaw
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetAnalogInRaw(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetAnalogInVel
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetAnalogInVel
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetAnalogInVel(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetQuadraturePosition
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetQuadraturePosition
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetQuadraturePosition(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetQuadraturePosition
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetQuadraturePosition
  (JNIEnv *, jclass, jlong handle, jint newPosition, jint timeoutMs)
{
    return (jint) c_MotController_SetQuadraturePosition(ConvertToMotorControllerWrapper(handle), newPosition, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetQuadratureVelocity
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetQuadratureVelocity
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetQuadratureVelocity(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPulseWidthPosition
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPulseWidthPosition
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetPulseWidthPosition(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetPulseWidthPosition
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetPulseWidthPosition
  (JNIEnv *, jclass, jlong handle, jint newPosition, jint timeoutMs)
{
    return (jint) c_MotController_SetPulseWidthPosition(ConvertToMotorControllerWrapper(handle), newPosition, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPulseWidthVelocity
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPulseWidthVelocity
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetPulseWidthVelocity(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPulseWidthRiseToFallUs
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPulseWidthRiseToFallUs
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetPulseWidthRiseToFallUs(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPulseWidthRiseToRiseUs
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPulseWidthRiseToRiseUs
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetPulseWidthRiseToRiseUs(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPinStateQuadA
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPinStateQuadA
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetPinStateQuadA(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPinStateQuadB
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPinStateQuadB
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetPinStateQuadB(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPinStateQuadIdx
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPinStateQuadIdx
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetPinStateQuadIdx(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    IsFwdLimitSwitchClosed
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_IsFwdLimitSwitchClosed
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_IsFwdLimitSwitchClosed(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    IsRevLimitSwitchClosed
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_IsRevLimitSwitchClosed
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_IsRevLimitSwitchClosed(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetFaults
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetFaults(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetStickyFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetStickyFaults
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_MotController_GetStickyFaults(ConvertToMotorControllerWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ClearStickyFaults
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ClearStickyFaults
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    return (jint) c_MotController_ClearStickyFaults(ConvertToMotorControllerWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SelectDemandType
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SelectDemandType
  (JNIEnv *, jclass, jlong handle, jint value)
{
    return (jint)c_MotController_SelectDemandType(ConvertToMotorControllerWrapper(handle), value);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetMPEOutput
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetMPEOutput
  (JNIEnv *, jclass, jlong handle, jint value)
{
    return (jint)c_MotController_SetMPEOutput(ConvertToMotorControllerWrapper(handle), value);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    EnableHeadingHold
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_EnableHeadingHold
  (JNIEnv *, jclass, jlong handle, jint value)
{
    return (jint)c_MotController_EnableHeadingHold(ConvertToMotorControllerWrapper(handle), value);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetClosedLoopTarget
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetClosedLoopTarget
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    int output = 0;
    c_MotController_GetClosedLoopTarget(ConvertToMotorControllerWrapper(handle), &output, pidIdx);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSelectedFeedbackCoefficient
 * Signature: (JDII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSelectedFeedbackCoefficient
  (JNIEnv *, jclass, jlong handle, jdouble coefficient, jint pidIdx, jint timeoutMs)
{
    return (jint)c_MotController_ConfigSelectedFeedbackCoefficient(ConvertToMotorControllerWrapper(handle), coefficient, pidIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClosedLoopPeakOutput
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClosedLoopPeakOutput
  (JNIEnv *, jclass, jlong handle, jint coefficient, jdouble pidIdx, jint timeoutMs)
{
    return (jint)c_MotController_ConfigClosedLoopPeakOutput(ConvertToMotorControllerWrapper(handle), coefficient, pidIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClosedLoopPeriod
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClosedLoopPeriod
  (JNIEnv *, jclass, jlong handle, jint slotIdx, jint loopTimeMs, jint timeoutMs)
{
    return (jint)c_MotController_ConfigClosedLoopPeriod(ConvertToMotorControllerWrapper(handle), slotIdx, loopTimeMs, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotorCommutation
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotorCommutation
  (JNIEnv *, jclass, jlong, jint, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetMotorCommutation
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetMotorCommutation
  (JNIEnv *, jclass, jlong, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSupplyCurrentLimit
 * Signature: (J[DII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSupplyCurrentLimit
  (JNIEnv *, jclass, jlong, jdoubleArray, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigStatorCurrentLimit
 * Signature: (J[DII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigStatorCurrentLimit
  (JNIEnv *, jclass, jlong, jdoubleArray, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigBrakeCurrentLimit
 * Signature: (J[DII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigBrakeCurrentLimit
  (JNIEnv *, jclass, jlong, jdoubleArray, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSupplyCurrentLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSupplyCurrentLimitEnable
  (JNIEnv *, jclass, jlong, jboolean, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigStatorCurrentLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigStatorCurrentLimitEnable
  (JNIEnv *, jclass, jlong, jboolean, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigBrakeCurrentLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigBrakeCurrentLimitEnable
  (JNIEnv *, jclass, jlong, jboolean, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetSupplyCurrentLimit
 * Signature: (J[DII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetSupplyCurrentLimit
  (JNIEnv *, jclass, jlong, jdoubleArray, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetStatorCurrentLimit
 * Signature: (J[DII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetStatorCurrentLimit
  (JNIEnv *, jclass, jlong, jdoubleArray, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetIntegratedSensorPosition
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetIntegratedSensorPosition
  (JNIEnv *, jclass, jlong, jdouble, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetIntegratedSensorPositionToAbsolute
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetIntegratedSensorPositionToAbsolute
  (JNIEnv *, jclass, jlong, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetIntegratedSensorPosition
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetIntegratedSensorPosition
  (JNIEnv *, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetIntegratedSensorAbsolutePosition
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetIntegratedSensorAbsolutePosition
  (JNIEnv *, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetIntegratedSensorVelocity
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetIntegratedSensorVelocity
  (JNIEnv *, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigAbsoluteSensorRange
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigAbsoluteSensorRange
  (JNIEnv *, jclass, jlong, jint, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMagnetOffset
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMagnetOffset
  (JNIEnv *, jclass, jlong, jdouble, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSensorInitializationStrategy
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSensorInitializationStrategy
  (JNIEnv *, jclass, jlong, jint, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

}  // extern "C"
