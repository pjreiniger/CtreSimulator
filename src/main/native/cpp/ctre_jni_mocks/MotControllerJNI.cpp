
#include <jni.h>

#include <cassert>

#include "CtreSimMocks/CtreMotControllerWrapper.h"
#include "CtreSimUtils/MockHookUtilities.h"
#include "com_ctre_phoenix_motorcontrol_can_MotControllerJNI.h"
#include "ctre/phoenix/cci/MotController_CCI.h"

namespace
{
void* ConvertToWrapper(jlong aHandle)
{
    return reinterpret_cast<SnobotSim::CtreMotControllerWrapper*>(aHandle);
}
} // namespace

extern "C" {

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Create
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Create
  (JNIEnv*, jclass, jint baseArbId)
{
    return (jlong)c_MotController_Create1(baseArbId);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    Create2
 * Signature: (ILjava/lang/String;)J
 */
JNIEXPORT jlong JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Create2
  (JNIEnv*, jclass, jint deviceID, jstring)
{
    return (jlong)c_MotController_Create2(deviceID, "");
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_JNI_1destroy
 * Method:    1MotController
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_JNI_1destroy_1MotController
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_MotController_Destroy(ConvertToWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetDeviceNumber
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetDeviceNumber
  (JNIEnv*, jclass, jlong handle)
{
    int deviceNumber = 0;
    c_MotController_GetDeviceNumber(ConvertToWrapper(handle), &deviceNumber);
    return deviceNumber;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetBaseID
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetBaseID
  (JNIEnv*, jclass, jlong handle)
{
    int baseArbId = 0;
    c_MotController_GetBaseID(ConvertToWrapper(handle), &baseArbId);
    return baseArbId;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetDemand
 * Signature: (JIII)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetDemand
  (JNIEnv*, jclass, jlong handle, jint mode, jint demand0, jint demand1)
{
    c_MotController_SetDemand(ConvertToWrapper(handle), mode, demand0, demand1);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Set
 * Method:    14
 * Signature: (JIDDI)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Set_14
  (JNIEnv*, jclass, jlong handle, jint mode, jdouble demand0, jdouble demand1,
   jint demand1Type)
{
    c_MotController_Set_4(ConvertToWrapper(handle), mode, demand0, demand1, demand1Type);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetNeutralMode
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetNeutralMode
  (JNIEnv*, jclass, jlong handle, jint neutralMode)
{
    c_MotController_SetNeutralMode(ConvertToWrapper(handle), neutralMode);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetSensorPhase
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetSensorPhase
  (JNIEnv*, jclass, jlong handle, jboolean PhaseSensor)
{
    c_MotController_SetSensorPhase(ConvertToWrapper(handle), PhaseSensor);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetInverted
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetInverted
  (JNIEnv*, jclass, jlong handle, jboolean invert)
{
    c_MotController_SetInverted(ConvertToWrapper(handle), invert);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetInverted
 * Method:    12
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetInverted_12
  (JNIEnv*, jclass, jlong handle, jint invertType)
{
    c_MotController_SetInverted_2(ConvertToWrapper(handle), invertType);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigFactoryDefault
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigFactoryDefault
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_MotController_ConfigFactoryDefault(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigOpenLoopRamp
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigOpenLoopRamp
  (JNIEnv*, jclass, jlong handle, jdouble secondsFromNeutralToFull,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigOpenLoopRamp(ConvertToWrapper(handle), secondsFromNeutralToFull, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClosedLoopRamp
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClosedLoopRamp
  (JNIEnv*, jclass, jlong handle, jdouble secondsFromNeutralToFull,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigClosedLoopRamp(ConvertToWrapper(handle), secondsFromNeutralToFull, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPeakOutputForward
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPeakOutputForward
  (JNIEnv*, jclass, jlong handle, jdouble percentOut, jint timeoutMs)
{
    return (jint)c_MotController_ConfigPeakOutputForward(ConvertToWrapper(handle), percentOut, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPeakOutputReverse
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPeakOutputReverse
  (JNIEnv*, jclass, jlong handle, jdouble percentOut, jint timeoutMs)
{
    return (jint)c_MotController_ConfigPeakOutputReverse(ConvertToWrapper(handle), percentOut, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigNominalOutputForward
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigNominalOutputForward
  (JNIEnv*, jclass, jlong handle, jdouble percentOut, jint timeoutMs)
{
    return (jint)c_MotController_ConfigNominalOutputForward(ConvertToWrapper(handle), percentOut, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigNominalOutputReverse
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigNominalOutputReverse
  (JNIEnv*, jclass, jlong handle, jdouble percentOut, jint timeoutMs)
{
    return (jint)c_MotController_ConfigNominalOutputReverse(ConvertToWrapper(handle), percentOut, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigNeutralDeadband
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigNeutralDeadband
  (JNIEnv*, jclass, jlong handle, jdouble percentDeadband, jint timeoutMs)
{
    return (jint)c_MotController_ConfigNeutralDeadband(ConvertToWrapper(handle), percentDeadband, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigVoltageCompSaturation
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigVoltageCompSaturation
  (JNIEnv*, jclass, jlong handle, jdouble voltage, jint timeoutMs)
{
    return (jint)c_MotController_ConfigVoltageCompSaturation(ConvertToWrapper(handle), voltage, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigVoltageMeasurementFilter
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigVoltageMeasurementFilter
  (JNIEnv*, jclass, jlong handle, jint filterWindowSamples, jint timeoutMs)
{
    return (jint)c_MotController_ConfigVoltageMeasurementFilter(ConvertToWrapper(handle), filterWindowSamples, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    EnableVoltageCompensation
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_EnableVoltageCompensation
  (JNIEnv*, jclass, jlong handle, jboolean enable)
{
    c_MotController_EnableVoltageCompensation(ConvertToWrapper(handle), enable);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetInverted
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetInverted
  (JNIEnv*, jclass, jlong handle)
{
    bool invert = false;
    c_MotController_GetInverted(ConvertToWrapper(handle), &invert);
    return invert;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetBusVoltage
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetBusVoltage
  (JNIEnv*, jclass, jlong handle)
{
    double voltage = 0;
    c_MotController_GetBusVoltage(ConvertToWrapper(handle), &voltage);
    return voltage;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetMotorOutputPercent
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotorOutputPercent
  (JNIEnv*, jclass, jlong handle)
{
    double percentOutput = 0;
    c_MotController_GetMotorOutputPercent(ConvertToWrapper(handle), &percentOutput);
    return percentOutput;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetOutputCurrent
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetOutputCurrent
  (JNIEnv*, jclass, jlong handle)
{
    double current = 0;
    c_MotController_GetOutputCurrent(ConvertToWrapper(handle), &current);
    return current;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetSupplyCurrent
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetSupplyCurrent
  (JNIEnv*, jclass, jlong handle)
{
    double current = 0;
    c_MotController_GetSupplyCurrent(ConvertToWrapper(handle), &current);
    return current;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetStatorCurrent
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetStatorCurrent
  (JNIEnv*, jclass, jlong handle)
{
    double current = 0;
    c_MotController_GetStatorCurrent(ConvertToWrapper(handle), &current);
    return current;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetTemperature
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetTemperature
  (JNIEnv*, jclass, jlong handle)
{
    double temperature = 0;
    c_MotController_GetTemperature(ConvertToWrapper(handle), &temperature);
    return temperature;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigRemoteFeedbackFilter
 * Signature: (JIIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigRemoteFeedbackFilter
  (JNIEnv*, jclass, jlong handle, jint deviceID, jint remoteSensorSource,
   jint remoteOrdinal, jint timeoutMs)
{
    return (jint)c_MotController_ConfigRemoteFeedbackFilter(ConvertToWrapper(handle), deviceID, remoteSensorSource, remoteOrdinal, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSelectedFeedbackSensor
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSelectedFeedbackSensor
  (JNIEnv*, jclass, jlong handle, jint feedbackDevice, jint pidIdx,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigSelectedFeedbackSensor(ConvertToWrapper(handle), feedbackDevice, pidIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSensorTerm
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSensorTerm
  (JNIEnv*, jclass, jlong handle, jint sensorTerm, jint feedbackDevice,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigSensorTerm(ConvertToWrapper(handle), sensorTerm, feedbackDevice, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetSelectedSensorPosition
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetSelectedSensorPosition
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    int param = 0;
    c_MotController_GetSelectedSensorPosition(ConvertToWrapper(handle), &param, pidIdx);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetSelectedSensorVelocity
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetSelectedSensorVelocity
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    int param = 0;
    c_MotController_GetSelectedSensorVelocity(ConvertToWrapper(handle), &param, pidIdx);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetSelectedSensorPosition
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetSelectedSensorPosition
  (JNIEnv*, jclass, jlong handle, jint sensorPos, jint pidIdx, jint timeoutMs)
{
    return (jint)c_MotController_SetSelectedSensorPosition(ConvertToWrapper(handle), sensorPos, pidIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetControlFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetControlFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint periodMs)
{
    return (jint)c_MotController_SetControlFramePeriod(ConvertToWrapper(handle), frame, periodMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetStatusFramePeriod
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint periodMs, jint timeoutMs)
{
    return (jint)c_MotController_SetStatusFramePeriod(ConvertToWrapper(handle), frame, periodMs, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetStatusFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint timeoutMs)
{
    int periodMs = 0;
    c_MotController_GetStatusFramePeriod(ConvertToWrapper(handle), frame, &periodMs, timeoutMs);
    return periodMs;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigVelocityMeasurementPeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigVelocityMeasurementPeriod
  (JNIEnv*, jclass, jlong handle, jint period, jint timeoutMs)
{
    return (jint)c_MotController_ConfigVelocityMeasurementPeriod(ConvertToWrapper(handle), period, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigVelocityMeasurementWindow
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigVelocityMeasurementWindow
  (JNIEnv*, jclass, jlong handle, jint windowSize, jint timeoutMs)
{
    return (jint)c_MotController_ConfigVelocityMeasurementWindow(ConvertToWrapper(handle), windowSize, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigForwardLimitSwitchSource
 * Signature: (JIIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigForwardLimitSwitchSource
  (JNIEnv*, jclass, jlong handle, jint type, jint normalOpenOrClose,
   jint deviceID, jint timeoutMs)
{
    return (jint)c_MotController_ConfigForwardLimitSwitchSource(ConvertToWrapper(handle), type, normalOpenOrClose, deviceID, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigReverseLimitSwitchSource
 * Signature: (JIIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigReverseLimitSwitchSource
  (JNIEnv*, jclass, jlong handle, jint type, jint normalOpenOrClose,
   jint deviceID, jint timeoutMs)
{
    return (jint)c_MotController_ConfigReverseLimitSwitchSource(ConvertToWrapper(handle), type, normalOpenOrClose, deviceID, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    OverrideLimitSwitchesEnable
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_OverrideLimitSwitchesEnable
  (JNIEnv*, jclass, jlong handle, jboolean enable)
{
    c_MotController_OverrideLimitSwitchesEnable(ConvertToWrapper(handle), enable);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigForwardSoftLimitThreshold
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigForwardSoftLimitThreshold
  (JNIEnv*, jclass, jlong handle, jint forwardSensorLimit, jint timeoutMs)
{
    return (jint)c_MotController_ConfigForwardSoftLimitThreshold(ConvertToWrapper(handle), forwardSensorLimit, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigReverseSoftLimitThreshold
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigReverseSoftLimitThreshold
  (JNIEnv*, jclass, jlong handle, jint reverseSensorLimit, jint timeoutMs)
{
    return (jint)c_MotController_ConfigReverseSoftLimitThreshold(ConvertToWrapper(handle), reverseSensorLimit, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigForwardSoftLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigForwardSoftLimitEnable
  (JNIEnv*, jclass, jlong handle, jboolean enable, jint timeoutMs)
{
    return (jint)c_MotController_ConfigForwardSoftLimitEnable(ConvertToWrapper(handle), enable, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigReverseSoftLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigReverseSoftLimitEnable
  (JNIEnv*, jclass, jlong handle, jboolean enable, jint timeoutMs)
{
    return (jint)c_MotController_ConfigReverseSoftLimitEnable(ConvertToWrapper(handle), enable, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    OverrideSoftLimitsEnable
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_OverrideSoftLimitsEnable
  (JNIEnv*, jclass, jlong handle, jboolean enable)
{
    c_MotController_OverrideSoftLimitsEnable(ConvertToWrapper(handle), enable);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config
 * Method:    1kP
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1kP
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_Config_kP(ConvertToWrapper(handle), slotIdx, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config
 * Method:    1kI
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1kI
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_Config_kI(ConvertToWrapper(handle), slotIdx, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config
 * Method:    1kD
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1kD
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_Config_kD(ConvertToWrapper(handle), slotIdx, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config
 * Method:    1kF
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1kF
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jdouble value, jint timeoutMs)
{
    return (jint)c_MotController_Config_kF(ConvertToWrapper(handle), slotIdx, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config
 * Method:    1IntegralZone
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Config_1IntegralZone
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jdouble izone, jint timeoutMs)
{
    return (jint)c_MotController_Config_IntegralZone(ConvertToWrapper(handle), slotIdx, izone, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigAllowableClosedloopError
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigAllowableClosedloopError
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jint allowableClosedLoopError,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigAllowableClosedloopError(ConvertToWrapper(handle), slotIdx, allowableClosedLoopError, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMaxIntegralAccumulator
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMaxIntegralAccumulator
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jdouble iaccum, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMaxIntegralAccumulator(ConvertToWrapper(handle), slotIdx, iaccum, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetIntegralAccumulator
 * Signature: (JDII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetIntegralAccumulator
  (JNIEnv*, jclass, jlong handle, jdouble iaccum, jint pidIdx, jint timeoutMs)
{
    return (jint)c_MotController_SetIntegralAccumulator(ConvertToWrapper(handle), iaccum, pidIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetClosedLoopError
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetClosedLoopError
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    int closedLoopError = 0;
    c_MotController_GetClosedLoopError(ConvertToWrapper(handle), &closedLoopError, pidIdx);
    return closedLoopError;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetIntegralAccumulator
 * Signature: (JI)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetIntegralAccumulator
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    double iaccum = 0;
    c_MotController_GetIntegralAccumulator(ConvertToWrapper(handle), &iaccum, pidIdx);
    return iaccum;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetErrorDerivative
 * Signature: (JI)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetErrorDerivative
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    double derror = 0;
    c_MotController_GetErrorDerivative(ConvertToWrapper(handle), &derror, pidIdx);
    return derror;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SelectProfileSlot
 * Signature: (JII)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SelectProfileSlot
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jint pidIdx)
{
    c_MotController_SelectProfileSlot(ConvertToWrapper(handle), slotIdx, pidIdx);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryPosition3
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryPosition3
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    int param = 0;
    c_MotController_GetActiveTrajectoryPosition_3(ConvertToWrapper(handle), &param, pidIdx);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryVelocity3
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryVelocity3
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    int param = 0;
    c_MotController_GetActiveTrajectoryVelocity_3(ConvertToWrapper(handle), &param, pidIdx);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryArbFeedFwd3
 * Signature: (JI)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryArbFeedFwd3
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    double param = 0;
    c_MotController_GetActiveTrajectoryArbFeedFwd_3(ConvertToWrapper(handle), &param, pidIdx);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryPosition
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryPosition
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetActiveTrajectoryPosition(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryVelocity
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryVelocity
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetActiveTrajectoryVelocity(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetActiveTrajectoryHeading
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetActiveTrajectoryHeading
  (JNIEnv*, jclass, jlong handle)
{
    double param = 0;
    c_MotController_GetActiveTrajectoryHeading(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionCruiseVelocity
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionCruiseVelocity
  (JNIEnv*, jclass, jlong handle, jint sensorUnitsPer100ms, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMotionCruiseVelocity(ConvertToWrapper(handle), sensorUnitsPer100ms, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionAcceleration
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionAcceleration
  (JNIEnv*, jclass, jlong handle, jint sensorUnitsPer100msPerSec,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigMotionAcceleration(ConvertToWrapper(handle), sensorUnitsPer100msPerSec, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionSCurveStrength
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionSCurveStrength
  (JNIEnv*, jclass, jlong handle, jint curveStrength, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMotionSCurveStrength(ConvertToWrapper(handle), curveStrength, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ClearMotionProfileTrajectories
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ClearMotionProfileTrajectories
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_MotController_ClearMotionProfileTrajectories(ConvertToWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetMotionProfileTopLevelBufferCount
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotionProfileTopLevelBufferCount
  (JNIEnv*, jclass, jlong handle)
{
    int value = 0;
    c_MotController_GetMotionProfileTopLevelBufferCount(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    PushMotionProfileTrajectory
 * Signature: (JDDDIZZ)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_PushMotionProfileTrajectory
  (JNIEnv*, jclass, jlong handle, jdouble position, jdouble velocity,
   jdouble headingDeg, jint profileSlotSelect, jboolean isLastPoint,
   jboolean zeroPos)
{
    return (jint)c_MotController_PushMotionProfileTrajectory(ConvertToWrapper(handle), position, velocity, headingDeg, profileSlotSelect, isLastPoint, zeroPos);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    PushMotionProfileTrajectory2
 * Signature: (JDDDIIZZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_PushMotionProfileTrajectory2
  (JNIEnv*, jclass, jlong handle, jdouble position, jdouble velocity,
   jdouble headingDeg, jint profileSlotSelect0, jint profileSlotSelect1,
   jboolean isLastPoint, jboolean zeroPos, jint durationMs)
{
    return (jint)c_MotController_PushMotionProfileTrajectory_2(ConvertToWrapper(handle), position, velocity, headingDeg, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos, durationMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    PushMotionProfileTrajectory3
 * Signature: (JDDDDDDIIZZIZ)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_PushMotionProfileTrajectory3
  (JNIEnv*, jclass, jlong handle, jdouble position, jdouble velocity,
   jdouble arbFeedFwd, jdouble auxiliaryPos, jdouble auxiliaryVel,
   jdouble auxiliaryArbFeedFwd, jint profileSlotSelect0,
   jint profileSlotSelect1, jboolean isLastPoint, jboolean zeroPos0,
   jint timeDur, jboolean useAuxPID)
{
    return (jint)c_MotController_PushMotionProfileTrajectory_3(ConvertToWrapper(handle), position, velocity, arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos0, timeDur, useAuxPID);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    StartMotionProfile
 * Signature: (JJII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_StartMotionProfile
  (JNIEnv*, jclass, jlong handle, jlong streamHandle, jint minBufferedPts,
   jint controlMode)
{
    return (jint)c_MotController_StartMotionProfile(ConvertToWrapper(handle), NULL, minBufferedPts, (ctre::phoenix::motorcontrol::ControlMode)controlMode);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    IsMotionProfileTopLevelBufferFull
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_IsMotionProfileTopLevelBufferFull
  (JNIEnv*, jclass, jlong handle)
{
    bool value = false;
    c_MotController_IsMotionProfileTopLevelBufferFull(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    IsMotionProfileFinished
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_IsMotionProfileFinished
  (JNIEnv*, jclass, jlong handle)
{
    bool value = false;
    c_MotController_IsMotionProfileFinished(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ProcessMotionProfileBuffer
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ProcessMotionProfileBuffer
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_MotController_ProcessMotionProfileBuffer(ConvertToWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetMotionProfileStatus
 * Signature: (J[I)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotionProfileStatus
  (JNIEnv* env, jclass, jlong handle, jintArray result)
{
    static const int kSize = 9;
    int output[kSize];

    size_t topBufferRem;
    size_t topBufferCnt;
    bool hasUnderrun = false;
    bool isUnderrun = false;
    bool activePointValid = false;
    bool isLast = false;

    c_MotController_GetMotionProfileStatus(ConvertToWrapper(handle),
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
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotionProfileStatus2
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

    c_MotController_GetMotionProfileStatus_2(ConvertToWrapper(handle),
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
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ClearMotionProfileHasUnderrun
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_MotController_ClearMotionProfileHasUnderrun(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ChangeMotionControlFramePeriod
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ChangeMotionControlFramePeriod
  (JNIEnv*, jclass, jlong handle, jint periodMs)
{
    return (jint)c_MotController_ChangeMotionControlFramePeriod(ConvertToWrapper(handle), periodMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionProfileTrajectoryPeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionProfileTrajectoryPeriod
  (JNIEnv*, jclass, jlong handle, jint durationMs, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMotionProfileTrajectoryPeriod(ConvertToWrapper(handle), durationMs, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotionProfileTrajectoryInterpolationEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotionProfileTrajectoryInterpolationEnable
  (JNIEnv*, jclass, jlong handle, jboolean enable, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMotionProfileTrajectoryInterpolationEnable(ConvertToWrapper(handle), enable, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigFeedbackNotContinuous
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigFeedbackNotContinuous
  (JNIEnv*, jclass, jlong handle, jboolean feedbackNotContinuous,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigFeedbackNotContinuous(ConvertToWrapper(handle), feedbackNotContinuous, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigRemoteSensorClosedLoopDisableNeutralOnLOS
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS
  (JNIEnv*, jclass, jlong handle,
   jboolean remoteSensorClosedLoopDisableNeutralOnLOS, jint timeoutMs)
{
    return (jint)c_MotController_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(ConvertToWrapper(handle), remoteSensorClosedLoopDisableNeutralOnLOS, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClearPositionOnLimitF
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClearPositionOnLimitF
  (JNIEnv*, jclass, jlong handle, jboolean clearPositionOnLimitF,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigClearPositionOnLimitF(ConvertToWrapper(handle), clearPositionOnLimitF, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClearPositionOnLimitR
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClearPositionOnLimitR
  (JNIEnv*, jclass, jlong handle, jboolean clearPositionOnLimitR,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigClearPositionOnLimitR(ConvertToWrapper(handle), clearPositionOnLimitR, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClearPositionOnQuadIdx
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClearPositionOnQuadIdx
  (JNIEnv*, jclass, jlong handle, jboolean clearPositionOnQuadIdx,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigClearPositionOnQuadIdx(ConvertToWrapper(handle), clearPositionOnQuadIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigLimitSwitchDisableNeutralOnLOS
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigLimitSwitchDisableNeutralOnLOS
  (JNIEnv*, jclass, jlong handle, jboolean limitSwitchDisableNeutralOnLOS,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigLimitSwitchDisableNeutralOnLOS(ConvertToWrapper(handle), limitSwitchDisableNeutralOnLOS, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSoftLimitDisableNeutralOnLOS
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSoftLimitDisableNeutralOnLOS
  (JNIEnv*, jclass, jlong handle, jboolean softLimitDisableNeutralOnLOS,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigSoftLimitDisableNeutralOnLOS(ConvertToWrapper(handle), softLimitDisableNeutralOnLOS, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPulseWidthPeriod
 * Method:    1EdgesPerRot
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPulseWidthPeriod_1EdgesPerRot
  (JNIEnv*, jclass, jlong handle, jint pulseWidthPeriod_EdgesPerRot,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigPulseWidthPeriod_EdgesPerRot(ConvertToWrapper(handle), pulseWidthPeriod_EdgesPerRot, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPulseWidthPeriod
 * Method:    1FilterWindowSz
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPulseWidthPeriod_1FilterWindowSz
  (JNIEnv*, jclass, jlong handle, jint pulseWidthPeriod_FilterWindowSz,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigPulseWidthPeriod_FilterWindowSz(ConvertToWrapper(handle), pulseWidthPeriod_FilterWindowSz, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetLastError
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetLastError
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_MotController_GetLastError(ConvertToWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetFirmwareVersion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetFirmwareVersion
  (JNIEnv*, jclass, jlong handle)
{
    int version = 0;
    c_MotController_GetFirmwareVersion(ConvertToWrapper(handle), &version);
    return version;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    HasResetOccurred
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_HasResetOccurred
  (JNIEnv*, jclass, jlong handle)
{
    bool output = false;
    c_MotController_HasResetOccurred(ConvertToWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSetCustomParam
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSetCustomParam
  (JNIEnv*, jclass, jlong handle, jint newValue, jint paramIndex,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigSetCustomParam(ConvertToWrapper(handle), newValue, paramIndex, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetCustomParam
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetCustomParam
  (JNIEnv*, jclass, jlong handle, jint paramIndex, jint timoutMs)
{
    int readValue = 0;
    c_MotController_ConfigGetCustomParam(ConvertToWrapper(handle), &readValue, paramIndex, timoutMs);
    return readValue;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSetParameter
 * Signature: (JIDIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSetParameter
  (JNIEnv*, jclass, jlong handle, jint param, jdouble value, jint subValue,
   jint ordinal, jint timeoutMs)
{
    return (jint)c_MotController_ConfigSetParameter(ConvertToWrapper(handle), param, value, subValue, ordinal, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetParameter
 * Signature: (JIII)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetParameter
  (JNIEnv*, jclass, jlong handle, jint param, jint ordinal, jint timeoutMs)
{
    double value = 0;
    c_MotController_ConfigGetParameter(ConvertToWrapper(handle), param, &value, ordinal, timeoutMs);
    return value;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPeakCurrentLimit
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPeakCurrentLimit
  (JNIEnv*, jclass, jlong handle, jint amps, jint timeoutMs)
{
    return (jint)c_MotController_ConfigPeakCurrentLimit(ConvertToWrapper(handle), amps, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigPeakCurrentDuration
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigPeakCurrentDuration
  (JNIEnv*, jclass, jlong handle, jint milliseconds, jint timeoutMs)
{
    return (jint)c_MotController_ConfigPeakCurrentDuration(ConvertToWrapper(handle), milliseconds, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigContinuousCurrentLimit
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigContinuousCurrentLimit
  (JNIEnv*, jclass, jlong handle, jint amps, jint timeoutMs)
{
    return (jint)c_MotController_ConfigContinuousCurrentLimit(ConvertToWrapper(handle), amps, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    EnableCurrentLimit
 * Signature: (JZ)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_EnableCurrentLimit
  (JNIEnv*, jclass, jlong handle, jboolean enable)
{
    return (jint)c_MotController_EnableCurrentLimit(ConvertToWrapper(handle), enable);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetAnalogIn
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetAnalogIn
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetAnalogIn(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetAnalogPosition
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetAnalogPosition
  (JNIEnv*, jclass, jlong handle, jint newPosition, jint timeoutMs)
{
    return (jint)c_MotController_SetAnalogPosition(ConvertToWrapper(handle), newPosition, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetAnalogInRaw
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetAnalogInRaw
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetAnalogInRaw(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetAnalogInVel
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetAnalogInVel
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetAnalogInVel(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetQuadraturePosition
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetQuadraturePosition
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetQuadraturePosition(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetQuadraturePosition
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetQuadraturePosition
  (JNIEnv*, jclass, jlong handle, jint newPosition, jint timeoutMs)
{
    return (jint)c_MotController_SetQuadraturePosition(ConvertToWrapper(handle), newPosition, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetQuadratureVelocity
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetQuadratureVelocity
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetQuadratureVelocity(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPulseWidthPosition
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPulseWidthPosition
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetPulseWidthPosition(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetPulseWidthPosition
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetPulseWidthPosition
  (JNIEnv*, jclass, jlong handle, jint newPosition, jint timeoutMs)
{
    return (jint)c_MotController_SetPulseWidthPosition(ConvertToWrapper(handle), newPosition, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPulseWidthVelocity
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPulseWidthVelocity
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetPulseWidthVelocity(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPulseWidthRiseToFallUs
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPulseWidthRiseToFallUs
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetPulseWidthRiseToFallUs(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPulseWidthRiseToRiseUs
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPulseWidthRiseToRiseUs
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetPulseWidthRiseToRiseUs(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPinStateQuadA
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPinStateQuadA
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetPinStateQuadA(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPinStateQuadB
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPinStateQuadB
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetPinStateQuadB(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetPinStateQuadIdx
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetPinStateQuadIdx
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetPinStateQuadIdx(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    IsFwdLimitSwitchClosed
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_IsFwdLimitSwitchClosed
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_IsFwdLimitSwitchClosed(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    IsRevLimitSwitchClosed
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_IsRevLimitSwitchClosed
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_IsRevLimitSwitchClosed(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetFaults
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetFaults(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetStickyFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetStickyFaults
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_MotController_GetStickyFaults(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ClearStickyFaults
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ClearStickyFaults
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_MotController_ClearStickyFaults(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SelectDemandType
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SelectDemandType
  (JNIEnv*, jclass, jlong handle, jint enable)
{
    return (jint)c_MotController_SelectDemandType(ConvertToWrapper(handle), enable);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetMPEOutput
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetMPEOutput
  (JNIEnv*, jclass, jlong handle, jint MpeOutput)
{
    return (jint)c_MotController_SetMPEOutput(ConvertToWrapper(handle), MpeOutput);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    EnableHeadingHold
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_EnableHeadingHold
  (JNIEnv*, jclass, jlong handle, jint enable)
{
    return (jint)c_MotController_EnableHeadingHold(ConvertToWrapper(handle), enable);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetClosedLoopTarget
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetClosedLoopTarget
  (JNIEnv*, jclass, jlong handle, jint pidIdx)
{
    int value = 0;
    c_MotController_GetClosedLoopTarget(ConvertToWrapper(handle), &value, pidIdx);
    return value;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSelectedFeedbackCoefficient
 * Signature: (JDII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSelectedFeedbackCoefficient
  (JNIEnv*, jclass, jlong handle, jdouble coefficient, jint pidIdx,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigSelectedFeedbackCoefficient(ConvertToWrapper(handle), coefficient, pidIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClosedLoopPeakOutput
 * Signature: (JIDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClosedLoopPeakOutput
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jdouble percentOut,
   jint timeoutMs)
{
    return (jint)c_MotController_ConfigClosedLoopPeakOutput(ConvertToWrapper(handle), slotIdx, percentOut, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigClosedLoopPeriod
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigClosedLoopPeriod
  (JNIEnv*, jclass, jlong handle, jint slotIdx, jint loopTimeMs, jint timeoutMs)
{
    return (jint)c_MotController_ConfigClosedLoopPeriod(ConvertToWrapper(handle), slotIdx, loopTimeMs, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigMotorCommutation
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigMotorCommutation
  (JNIEnv*, jclass, jlong handle, jint motorCommutation, jint timeoutMs)
{
    return (jint)c_MotController_ConfigMotorCommutation(ConvertToWrapper(handle), (ctre::phoenix::motorcontrol::MotorCommutation)motorCommutation, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetMotorCommutation
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetMotorCommutation
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    ctre::phoenix::motorcontrol::MotorCommutation motorCommutation;
    c_MotController_ConfigGetMotorCommutation(ConvertToWrapper(handle), &motorCommutation, timeoutMs);
    return (jint)motorCommutation;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSupplyCurrentLimit
 * Signature: (J[DI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSupplyCurrentLimit
  (JNIEnv*, jclass, jlong handle, jdoubleArray paramCnt, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigStatorCurrentLimit
 * Signature: (J[DI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigStatorCurrentLimit
  (JNIEnv*, jclass, jlong handle, jdoubleArray paramCnt, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigSupplyCurrentLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSupplyCurrentLimitEnable
  (JNIEnv*, jclass, jlong handle, jboolean enable, jint timeoutMs)
{
    return (jint)c_MotController_ConfigSupplyCurrentLimitEnable(ConvertToWrapper(handle), enable, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigStatorCurrentLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigStatorCurrentLimitEnable
  (JNIEnv*, jclass, jlong handle, jboolean enable, jint timeoutMs)
{
    return (jint)c_MotController_ConfigStatorCurrentLimitEnable(ConvertToWrapper(handle), enable, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigBrakeCurrentLimitEnable
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigBrakeCurrentLimitEnable
  (JNIEnv*, jclass, jlong, jboolean, jint)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetSupplyCurrentLimit
 * Signature: (J[DI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetSupplyCurrentLimit
  (JNIEnv*, jclass, jlong handle, jdoubleArray fillCapacity, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigGetStatorCurrentLimit
 * Signature: (J[DI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetStatorCurrentLimit
  (JNIEnv*, jclass, jlong handle, jdoubleArray fillCapacity, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetIntegratedSensorPosition
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetIntegratedSensorPosition
  (JNIEnv*, jclass, jlong handle, jdouble newpos, jint timeoutMs)
{
    return (jint)c_MotController_SetIntegratedSensorPosition(ConvertToWrapper(handle), newpos, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    SetIntegratedSensorPositionToAbsolute
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_SetIntegratedSensorPositionToAbsolute
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_MotController_SetIntegratedSensorPositionToAbsolute(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetIntegratedSensorPosition
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetIntegratedSensorPosition
  (JNIEnv*, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetIntegratedSensorAbsolutePosition
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetIntegratedSensorAbsolutePosition
  (JNIEnv*, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    GetIntegratedSensorVelocity
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetIntegratedSensorVelocity
  (JNIEnv*, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigIntegratedSensorAbsoluteRange
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigIntegratedSensorAbsoluteRange
  (JNIEnv*, jclass, jlong handle, jint absoluteSensorRange, jint timeoutMs)
{
    return (jint)c_MotController_ConfigIntegratedSensorAbsoluteRange(ConvertToWrapper(handle), (ctre::phoenix::sensors::AbsoluteSensorRange)absoluteSensorRange, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigIntegratedSensorOffset
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigIntegratedSensorOffset
  (JNIEnv*, jclass, jlong handle, jdouble offsetDegrees, jint timeoutMs)
{
    return (jint)c_MotController_ConfigIntegratedSensorOffset(ConvertToWrapper(handle), offsetDegrees, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_motorcontrol_can_MotControllerJNI
 * Method:    ConfigIntegratedSensorInitializationStrategy
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigIntegratedSensorInitializationStrategy
  (JNIEnv*, jclass, jlong handle, jint initializationStrategy, jint timeoutMs)
{
    return (jint)c_MotController_ConfigIntegratedSensorInitializationStrategy(ConvertToWrapper(handle), (ctre::phoenix::sensors::SensorInitializationStrategy)initializationStrategy, timeoutMs);
}

} // extern "C"
