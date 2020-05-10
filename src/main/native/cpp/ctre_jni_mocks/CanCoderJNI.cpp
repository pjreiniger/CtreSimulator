
#include <jni.h>

#include <cassert>

#include "CtreSimMocks/CtreCANCoderWrapper.h"
#include "CtreSimUtils/MockHookUtilities.h"
#include "com_ctre_phoenix_sensors_CANCoderJNI.h"
#include "ctre/phoenix/cci/CANCoder_CCI.h"

namespace
{
SnobotSim::CtreCANCoderWrapper* ConvertToWrapper(jlong handle)
{
    return reinterpret_cast<SnobotSim::CtreCANCoderWrapper*>(handle);
}
} // namespace

extern "C" {

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    Create
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_Create
  (JNIEnv*, jclass, jint deviceId)
{
    return (jlong)c_CANCoder_Create1(deviceId);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    Destroy
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_Destroy
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_CANCoder_Destroy(ConvertToWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetLastError
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetLastError
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_CANCoder_GetLastError(ConvertToWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetLastUnitString
 * Signature: (J)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetLastUnitString
  (JNIEnv*, jclass, jlong handle)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return nullptr;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetLastTimestamp
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetLastTimestamp
  (JNIEnv*, jclass, jlong handle)
{
    double timestamp = 0;
    c_CANCoder_GetLastTimestamp(ConvertToWrapper(handle), &timestamp);
    return timestamp;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetBusVoltage
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetBusVoltage
  (JNIEnv*, jclass, jlong handle)
{
    double batteryVoltage = 0;
    c_CANCoder_GetBusVoltage(ConvertToWrapper(handle), &batteryVoltage);
    return batteryVoltage;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetMagnetFieldStrength
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetMagnetFieldStrength
  (JNIEnv*, jclass, jlong handle)
{
    ctre::phoenix::sensors::MagnetFieldStrength magnetFieldStrength = (ctre::phoenix::sensors::MagnetFieldStrength)0;
    c_CANCoder_GetMagnetFieldStrength(ConvertToWrapper(handle), &magnetFieldStrength);
    return magnetFieldStrength;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetPosition
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetPosition
  (JNIEnv*, jclass, jlong handle)
{
    double pos = 0;
    c_CANCoder_GetPosition(ConvertToWrapper(handle), &pos);
    return pos;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    SetPosition
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_SetPosition
  (JNIEnv*, jclass, jlong handle, jdouble pos, jint timeoutMs)
{
    return (jint)c_CANCoder_SetPosition(ConvertToWrapper(handle), pos, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    SetPositionToAbsolute
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_SetPositionToAbsolute
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_CANCoder_SetPositionToAbsolute(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigSensorDirection
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigSensorDirection
  (JNIEnv*, jclass, jlong handle, jint bDirection, jint timeoutMs)
{
    return (jint)c_CANCoder_ConfigSensorDirection(ConvertToWrapper(handle), bDirection, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetVelocity
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetVelocity
  (JNIEnv*, jclass, jlong handle)
{
    double vel = 0;
    c_CANCoder_GetVelocity(ConvertToWrapper(handle), &vel);
    return vel;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetAbsolutePosition
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetAbsolutePosition
  (JNIEnv*, jclass, jlong handle)
{
    double pos = 0;
    c_CANCoder_GetAbsolutePosition(ConvertToWrapper(handle), &pos);
    return pos;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigVelocityMeasurementPeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigVelocityMeasurementPeriod
  (JNIEnv*, jclass, jlong handle, jint period, jint timeoutMs)
{
    return (jint)c_CANCoder_ConfigVelocityMeasurementPeriod(ConvertToWrapper(handle), period, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigVelocityMeasurementWindow
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigVelocityMeasurementWindow
  (JNIEnv*, jclass, jlong handle, jint window, jint timeoutMs)
{
    return (jint)c_CANCoder_ConfigVelocityMeasurementWindow(ConvertToWrapper(handle), window, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigAbsoluteSensorRange
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigAbsoluteSensorRange
  (JNIEnv*, jclass, jlong handle, jint absoluteSensorRange, jint timeoutMs)
{
    return (jint)c_CANCoder_ConfigAbsoluteSensorRange(ConvertToWrapper(handle), (ctre::phoenix::sensors::AbsoluteSensorRange)absoluteSensorRange, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigMagnetOffset
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigMagnetOffset
  (JNIEnv*, jclass, jlong handle, jdouble offsetDegrees, jint timeoutMs)
{
    return (jint)c_CANCoder_ConfigMagnetOffset(ConvertToWrapper(handle), offsetDegrees, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigSensorInitializationStrategy
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigSensorInitializationStrategy
  (JNIEnv*, jclass, jlong handle, jint initializationStrategy, jint timeoutMs)
{
    return (jint)c_CANCoder_ConfigSensorInitializationStrategy(ConvertToWrapper(handle), (ctre::phoenix::sensors::SensorInitializationStrategy)initializationStrategy, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigFeedbackCoefficient
 * Signature: (JDLjava/lang/String;II)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigFeedbackCoefficient
  (JNIEnv*, jclass, jlong handle, jdouble sensorCoefficient,
   jstring outUnitString, jint sensortimeBase, jint timeoutMs)
{
    char* unitString = nullptr;
    c_CANCoder_ConfigFeedbackCoefficient(ConvertToWrapper(handle), sensorCoefficient, unitString, (ctre::phoenix::sensors::SensorTimeBase)sensortimeBase, timeoutMs);
    return 0;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigSetParameter
 * Signature: (JIDIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigSetParameter
  (JNIEnv*, jclass, jlong handle, jint param, jdouble value, jint subValue,
   jint ordinal, jint timeoutMs)
{
    return (jint)c_CANCoder_ConfigSetParameter(ConvertToWrapper(handle), param, value, subValue, ordinal, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigGetParameter
 * Signature: (JIII)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigGetParameter
  (JNIEnv*, jclass, jlong handle, jint param, jint ordinal, jint timeoutMs)
{
    double value = 0;
    c_CANCoder_ConfigGetParameter(ConvertToWrapper(handle), param, &value, ordinal, timeoutMs);
    return value;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigSetCustomParam
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigSetCustomParam
  (JNIEnv*, jclass, jlong handle, jint newValue, jint paramIndex,
   jint timeoutMs)
{
    return (jint)c_CANCoder_ConfigSetCustomParam(ConvertToWrapper(handle), newValue, paramIndex, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigGetCustomParam
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigGetCustomParam
  (JNIEnv*, jclass, jlong handle, jint paramIndex, jint timoutMs)
{
    int readValue = 0;
    c_CANCoder_ConfigGetCustomParam(ConvertToWrapper(handle), &readValue, paramIndex, timoutMs);
    return readValue;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ConfigFactoryDefault
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigFactoryDefault
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_CANCoder_ConfigFactoryDefault(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetFaults
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_CANCoder_GetFaults(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetStickyFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetStickyFaults
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_CANCoder_GetStickyFaults(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    ClearStickyFaults
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_ClearStickyFaults
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_CANCoder_ClearStickyFaults(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetFirmwareVersion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetFirmwareVersion
  (JNIEnv*, jclass, jlong handle)
{
    int firmwareVers = 0;
    c_CANCoder_GetFirmwareVersion(ConvertToWrapper(handle), &firmwareVers);
    return firmwareVers;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    HasResetOccurred
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_HasResetOccurred
  (JNIEnv*, jclass, jlong handle)
{
    bool hasReset = false;
    c_CANCoder_HasResetOccurred(ConvertToWrapper(handle), &hasReset);
    return hasReset;
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    SetStatusFramePeriod
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_SetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint periodMs, jint timeoutMs)
{
    return (jint)c_CANCoder_SetStatusFramePeriod(ConvertToWrapper(handle), frame, periodMs, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_CANCoderJNI
 * Method:    GetStatusFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_CANCoderJNI_GetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint timeoutMs)
{
    int periodMs = 0;
    c_CANCoder_GetStatusFramePeriod(ConvertToWrapper(handle), frame, &periodMs, timeoutMs);
    return periodMs;
}

} // extern "C"
