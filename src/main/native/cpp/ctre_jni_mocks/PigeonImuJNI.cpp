
#include <jni.h>

#include <cassert>

#include "CtreSimMocks/MockHookUtilities.h"
#include "CtreSimMocks/CtrePigeonImuWrapper.h"
#include "com_ctre_phoenix_sensors_PigeonImuJNI.h"
#include "ctre/phoenix/CCI/PigeonIMU_CCI.h"

#define GET_THREE_AXIS(type, capType, funcName, size)      \
                                                           \
    type angles[size]; /* NOLINT */                        \
    funcName(ConvertToPigeonWrapper(handle), angles);                             \
                                                           \
    j##type fill[size];                                    \
    for (int i = 0; i < size; ++i)                         \
    {                                                      \
        fill[i] = angles[i];                               \
    }                                                      \
                                                           \
    env->Set##capType##ArrayRegion(result, 0, size, fill); \
    return 0;


SnobotSim::CtrePigeonImuWrapper* ConvertToPigeonWrapper(jlong aHandle)
{
    return reinterpret_cast<SnobotSim::CtrePigeonImuWrapper*>(aHandle);
}

extern "C" {
/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_new_PigeonImu_Talon
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1new_1PigeonImu_1Talon
  (JNIEnv *, jclass, jint talonId)
{
    return (jlong)c_PigeonIMU_Create2(talonId);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_new_PigeonImu
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1new_1PigeonImu
  (JNIEnv *, jclass, jint pigeonId)
{
    return (jlong)c_PigeonIMU_Create1(pigeonId);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_ConfigSetCustomParam
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigSetCustomParam
  (JNIEnv *, jclass, jlong handle, jint newValue, jint paramIndex, jint timeoutMs)
{
    return (jint)c_PigeonIMU_ConfigSetCustomParam(ConvertToPigeonWrapper(handle), newValue, paramIndex, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_ConfigGetCustomParam
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigGetCustomParam
  (JNIEnv *, jclass, jlong aHandle, jint paramIndex, jint timoutMs)
{
    int output = 0;
    c_PigeonIMU_ConfigGetCustomParam(ConvertToPigeonWrapper(aHandle), &output, paramIndex, timoutMs);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_ConfigSetParameter
 * Signature: (JIDIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigSetParameter
  (JNIEnv *, jclass, jlong handle, jint param, jdouble paramValue, jint subValue, jint ordinal, jint timeoutMs)
{
	return (int) c_PigeonIMU_ConfigSetParameter(ConvertToPigeonWrapper(handle), param, paramValue, subValue, ordinal, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_ConfigGetParameter
 * Signature: (JIII)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigGetParameter
  (JNIEnv *, jclass, jlong handle, jint param, jint ordinal, jint timeoutMs)
{
    double output = 0;
    c_PigeonIMU_ConfigGetParameter(ConvertToPigeonWrapper(handle), param, &output, ordinal, timeoutMs);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_SetStatusFramePeriod
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetStatusFramePeriod
  (JNIEnv *, jclass, jlong handle, jint frame, jint periodMs, jint timeoutMs)
{
    return c_PigeonIMU_SetStatusFramePeriod(ConvertToPigeonWrapper(handle), frame, periodMs, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_SetYaw
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetYaw
  (JNIEnv *, jclass, jlong handle, jdouble value, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetYaw(ConvertToPigeonWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_AddYaw
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1AddYaw
  (JNIEnv *, jclass, jlong handle, jdouble value, jint timeoutMs)
{
    return (jint)c_PigeonIMU_AddYaw(ConvertToPigeonWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_SetYawToCompass
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetYawToCompass
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetYawToCompass(ConvertToPigeonWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_SetFusedHeading
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetFusedHeading
  (JNIEnv *, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetFusedHeading(ConvertToPigeonWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_AddFusedHeading
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1AddFusedHeading
  (JNIEnv *, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_AddFusedHeading(ConvertToPigeonWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_SetFusedHeadingToCompass
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetFusedHeadingToCompass
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetFusedHeadingToCompass(ConvertToPigeonWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_SetAccumZAngle
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetAccumZAngle
  (JNIEnv *, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetAccumZAngle(ConvertToPigeonWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_ConfigTemperatureCompensationEnable
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigTemperatureCompensationEnable
  (JNIEnv *, jclass, jlong handle, jint bTempCompEnable, jint timeoutMs)
{
    return (jint)c_PigeonIMU_ConfigTemperatureCompensationEnable(ConvertToPigeonWrapper(handle), bTempCompEnable, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_SetCompassDeclination
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetCompassDeclination
  (JNIEnv *, jclass, jlong handle, jdouble angleDegOffset, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetCompassDeclination(ConvertToPigeonWrapper(handle), angleDegOffset, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_SetCompassAngle
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetCompassAngle
  (JNIEnv *, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetCompassAngle(ConvertToPigeonWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_EnterCalibrationMode
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1EnterCalibrationMode
  (JNIEnv *, jclass, jlong handle, jint value, jint timeoutMs)
{
    return (jint)c_PigeonIMU_EnterCalibrationMode(ConvertToPigeonWrapper(handle), value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetGeneralStatus
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetGeneralStatus
  (JNIEnv *, jclass, jlong handle, jdoubleArray)
{
    int state = 0;
    int currentMode = 0;
    int calibrationError = 0;
    int bCalIsBooting = 0;
    double tempC = 0;
    int upTimeSec = 0;
    int noMotionBiasCount = 0;
    int tempCompensationCount = 0;
    int lastError = 0;

    return c_PigeonIMU_GetGeneralStatus(ConvertToPigeonWrapper(handle), &state, &currentMode, &calibrationError, &bCalIsBooting, &tempC, &upTimeSec, &noMotionBiasCount, &tempCompensationCount, &lastError);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_Get6dQuaternion
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1Get6dQuaternion
  (JNIEnv * env, jclass, jlong handle, jdoubleArray result)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_Get6dQuaternion, 4);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetYawPitchRoll
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetYawPitchRoll
  (JNIEnv * env, jclass, jlong handle, jdoubleArray result)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_GetYawPitchRoll, 3);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetAccumGyro
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetAccumGyro
  (JNIEnv * env, jclass, jlong handle, jdoubleArray result)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_GetAccumGyro, 3);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetAbsoluteCompassHeading
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetAbsoluteCompassHeading
  (JNIEnv *, jclass, jlong handle)
{
    double output = 0;
    c_PigeonIMU_GetAbsoluteCompassHeading(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetCompassHeading
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetCompassHeading
  (JNIEnv *, jclass, jlong handle)
{
    double output = 0;
    c_PigeonIMU_GetCompassHeading(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetCompassFieldStrength
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetCompassFieldStrength
  (JNIEnv *, jclass, jlong handle)
{
    double output = 0;
    c_PigeonIMU_GetCompassFieldStrength(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetTemp
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetTemp
  (JNIEnv *, jclass, jlong handle)
{
    double output = 0;
    c_PigeonIMU_GetTemp(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetUpTime
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetUpTime
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_PigeonIMU_GetUpTime(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetRawMagnetometer
 * Signature: (J[S)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetRawMagnetometer
  (JNIEnv * env, jclass, jlong handle, jshortArray result)
{
    GET_THREE_AXIS(short, Short, c_PigeonIMU_GetRawMagnetometer, 3); // NOLINT
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetBiasedMagnetometer
 * Signature: (J[S)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetBiasedMagnetometer
  (JNIEnv * env, jclass, jlong handle, jshortArray result)
{
    GET_THREE_AXIS(short, Short, c_PigeonIMU_GetBiasedMagnetometer, 3); // NOLINT
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetBiasedAccelerometer
 * Signature: (J[S)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetBiasedAccelerometer
  (JNIEnv * env, jclass, jlong handle, jshortArray result)
{
    GET_THREE_AXIS(short, Short, c_PigeonIMU_GetBiasedAccelerometer, 3); // NOLINT
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetRawGyro
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetRawGyro
  (JNIEnv * env, jclass, jlong handle, jdoubleArray result)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_GetRawGyro, 3);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetAccelerometerAngles
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetAccelerometerAngles
  (JNIEnv * env, jclass, jlong handle, jdoubleArray result)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_GetAccelerometerAngles, 3);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetFusedHeading
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetFusedHeading
  (JNIEnv * env, jclass, jlong handle, jdoubleArray result)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_GetFusedHeading1, 3);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetState
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetState
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_PigeonIMU_GetState(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetResetCount
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetResetCount
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_PigeonIMU_GetResetCount(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetResetFlags
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetResetFlags
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_PigeonIMU_GetResetFlags(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetFirmwareVersion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetFirmwareVersion
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_PigeonIMU_GetFirmwareVersion(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetLastError
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetLastError
  (JNIEnv *, jclass, jlong handle)
{
    return (jint)c_PigeonIMU_GetLastError(ConvertToPigeonWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_HasResetOccurred
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1HasResetOccurred
  (JNIEnv *, jclass, jlong handle)
{
    bool output = 0;
    c_PigeonIMU_HasResetOccurred(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetStatusFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetStatusFramePeriod
  (JNIEnv *, jclass, jlong handle, jint frame, jint timeoutMs)
{
    int output = 0;
    c_PigeonIMU_GetStatusFramePeriod(ConvertToPigeonWrapper(handle), frame, &output, timeoutMs);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_SetControlFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetControlFramePeriod
  (JNIEnv *, jclass, jlong handle, jint frame, jint periodMs)
{
    return c_PigeonIMU_SetControlFramePeriod(ConvertToPigeonWrapper(handle), frame, periodMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetFaults
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_PigeonIMU_GetFaults(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_GetStickyFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetStickyFaults
  (JNIEnv *, jclass, jlong handle)
{
    int output = 0;
    c_PigeonIMU_GetStickyFaults(ConvertToPigeonWrapper(handle), &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI
 * Method:    JNI_ClearStickyFaults
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ClearStickyFaults
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    return c_PigeonIMU_ClearStickyFaults(ConvertToPigeonWrapper(handle), timeoutMs);
}

}  // extern "C"
