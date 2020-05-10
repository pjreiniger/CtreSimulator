
#include <jni.h>

#include <cassert>

#include "CtreSimMocks/CtreCANifierWrapper.h"
#include "CtreSimUtils/MockHookUtilities.h"
#include "com_ctre_phoenix_CANifierJNI.h"
#include "ctre/phoenix/cci/CANifier_CCI.h"

namespace
{
SnobotSim::CtreCANifierWrapper* ConvertToWrapper(jlong handle)
{
    return reinterpret_cast<SnobotSim::CtreCANifierWrapper*>(handle);
}
} // namespace

extern "C" {

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI_1new
 * Method:    1CANifier
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1new_1CANifier
  (JNIEnv*, jclass, jint deviceNumber)
{
    return (jlong)c_CANifier_Create1(deviceNumber);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI_1destroy
 * Method:    1CANifier
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1destroy_1CANifier
  (JNIEnv*, jclass, jlong aHandle)
{
    return c_CANifier_Destroy(ConvertToWrapper(aHandle));
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1SetLEDOutput
 * Signature: (JII)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1SetLEDOutput
  (JNIEnv*, jclass, jlong handle, jint dutyCycle, jint ledChannel)
{
    c_CANifier_SetLEDOutput(ConvertToWrapper(handle), dutyCycle, ledChannel);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1SetGeneralOutputs
 * Signature: (JII)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1SetGeneralOutputs
  (JNIEnv*, jclass, jlong handle, jint outputsBits, jint isOutputBits)
{
    c_CANifier_SetGeneralOutputs(ConvertToWrapper(handle), outputsBits, isOutputBits);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1SetGeneralOutput
 * Signature: (JIZZ)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1SetGeneralOutput
  (JNIEnv*, jclass, jlong handle, jint outputPin, jboolean outputValue,
   jboolean outputEnable)
{
    c_CANifier_SetGeneralOutput(ConvertToWrapper(handle), outputPin, outputValue, outputEnable);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1SetPWMOutput
 * Signature: (JII)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1SetPWMOutput
  (JNIEnv*, jclass, jlong handle, jint pwmChannel, jint dutyCycle)
{
    c_CANifier_SetPWMOutput(ConvertToWrapper(handle), pwmChannel, dutyCycle);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1EnablePWMOutput
 * Signature: (JIZ)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1EnablePWMOutput
  (JNIEnv*, jclass, jlong handle, jint pwmChannel, jboolean bEnable)
{
    c_CANifier_EnablePWMOutput(ConvertToWrapper(handle), pwmChannel, bEnable);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetGeneralInputs
 * Signature: (J[Z)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetGeneralInputs
  (JNIEnv*, jclass, jlong aHandle, jbooleanArray allPinsPtr)
{
    const int kCapacity = 11;
    bool allPins[kCapacity];

    c_CANifier_GetGeneralInputs(ConvertToWrapper(aHandle), allPins, kCapacity);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetGeneralInput
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetGeneralInput
  (JNIEnv*, jclass, jlong handle, jint inputPin)
{
    bool measuredInput = false;
    c_CANifier_GetGeneralInput(ConvertToWrapper(handle), inputPin, &measuredInput);
    return measuredInput;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetPWMInput
 * Signature: (JI[D)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetPWMInput
  (JNIEnv*, jclass, jlong handle, jint pwmChannel,
   jdoubleArray dutyCycleAndPeriodPtr)
{
    double dutyCycleAndPeriod[2];
    c_CANifier_GetPWMInput(ConvertToWrapper(handle), pwmChannel, dutyCycleAndPeriod);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetLastError
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetLastError
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_CANifier_GetLastError(ConvertToWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetBatteryVoltage
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetBatteryVoltage
  (JNIEnv*, jclass, jlong)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetQuadraturePosition
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetQuadraturePosition
  (JNIEnv*, jclass, jlong handle)
{
    int pos = 0;
    c_CANifier_GetQuadraturePosition(ConvertToWrapper(handle), &pos);
    return pos;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1SetQuadraturePosition
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1SetQuadraturePosition
  (JNIEnv*, jclass, jlong handle, jint pos, jint timeoutMs)
{
    return (jint)c_CANifier_SetQuadraturePosition(ConvertToWrapper(handle), pos, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetQuadratureVelocity
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetQuadratureVelocity
  (JNIEnv*, jclass, jlong handle)
{
    int vel = 0;
    c_CANifier_GetQuadratureVelocity(ConvertToWrapper(handle), &vel);
    return vel;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigVelocityMeasurementPeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigVelocityMeasurementPeriod
  (JNIEnv*, jclass, jlong handle, jint period, jint timeoutMs)
{
    return (jint)c_CANifier_ConfigVelocityMeasurementPeriod(ConvertToWrapper(handle), period, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigVelocityMeasurementWindow
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigVelocityMeasurementWindow
  (JNIEnv*, jclass, jlong handle, jint window, jint timeoutMs)
{
    return (jint)c_CANifier_ConfigVelocityMeasurementWindow(ConvertToWrapper(handle), window, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigClearPositionOnLimitF
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigClearPositionOnLimitF
  (JNIEnv*, jclass, jlong handle, jboolean clearPositionOnLimitF,
   jint timeoutMs)
{
    return (jint)c_CANifier_ConfigClearPositionOnLimitF(ConvertToWrapper(handle), clearPositionOnLimitF, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigClearPositionOnLimitR
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigClearPositionOnLimitR
  (JNIEnv*, jclass, jlong handle, jboolean clearPositionOnLimitR,
   jint timeoutMs)
{
    return (jint)c_CANifier_ConfigClearPositionOnLimitR(ConvertToWrapper(handle), clearPositionOnLimitR, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigClearPositionOnQuadIdx
 * Signature: (JZI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigClearPositionOnQuadIdx
  (JNIEnv*, jclass, jlong handle, jboolean clearPositionOnQuadIdx,
   jint timeoutMs)
{
    return (jint)c_CANifier_ConfigClearPositionOnQuadIdx(ConvertToWrapper(handle), clearPositionOnQuadIdx, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigSetCustomParam
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigSetCustomParam
  (JNIEnv*, jclass, jlong handle, jint newValue, jint paramIndex,
   jint timeoutMs)
{
    return (jint)c_CANifier_ConfigSetCustomParam(ConvertToWrapper(handle), newValue, paramIndex, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigGetCustomParam
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigGetCustomParam
  (JNIEnv*, jclass, jlong handle, jint paramIndex, jint timoutMs)
{
    int readValue = 0;
    c_CANifier_ConfigGetCustomParam(ConvertToWrapper(handle), &readValue, paramIndex, timoutMs);
    return readValue;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigSetParameter
 * Signature: (JIDIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigSetParameter
  (JNIEnv*, jclass, jlong handle, jint param, jdouble value, jint subValue,
   jint ordinal, jint timeoutMs)
{
    return (jint)c_CANifier_ConfigSetParameter(ConvertToWrapper(handle), param, value, subValue, ordinal, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigGetParameter
 * Signature: (JIII)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigGetParameter
  (JNIEnv*, jclass, jlong handle, jint param, jint ordinal, jint timeoutMs)
{
    double value = 0;
    c_CANifier_ConfigGetParameter(ConvertToWrapper(handle), param, &value, ordinal, timeoutMs);
    return value;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ConfigFactoryDefault
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ConfigFactoryDefault
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_CANifier_ConfigFactoryDefault(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1SetStatusFramePeriod
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1SetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint periodMs, jint timeoutMs)
{
    return (jint)c_CANifier_SetStatusFramePeriod(ConvertToWrapper(handle), frame, periodMs, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetStatusFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint timeoutMs)
{
    int periodMs = 0;
    c_CANifier_GetStatusFramePeriod(ConvertToWrapper(handle), frame, &periodMs, timeoutMs);
    return periodMs;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1SetControlFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1SetControlFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint periodMs)
{
    return (jint)c_CANifier_SetControlFramePeriod(ConvertToWrapper(handle), frame, periodMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetFirmwareVersion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetFirmwareVersion
  (JNIEnv*, jclass, jlong handle)
{
    int firmwareVers = 0;
    c_CANifier_GetFirmwareVersion(ConvertToWrapper(handle), &firmwareVers);
    return firmwareVers;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1HasResetOccurred
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1HasResetOccurred
  (JNIEnv*, jclass, jlong handle)
{
    bool hasReset = false;
    c_CANifier_HasResetOccurred(ConvertToWrapper(handle), &hasReset);
    return hasReset;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetFaults
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_CANifier_GetFaults(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetStickyFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetStickyFaults
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_CANifier_GetStickyFaults(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1ClearStickyFaults
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1ClearStickyFaults
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_CANifier_ClearStickyFaults(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_CANifierJNI_JNI
 * Method:    1GetBusVoltage
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_CANifierJNI_JNI_1GetBusVoltage
  (JNIEnv*, jclass, jlong handle)
{
    double batteryVoltage = 0;
    c_CANifier_GetBusVoltage(ConvertToWrapper(handle), &batteryVoltage);
    return batteryVoltage;
}

} // extern "C"
