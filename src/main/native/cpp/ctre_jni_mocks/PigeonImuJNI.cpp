
#include <jni.h>

#include <cassert>

#include "CtreSimMocks/CtrePigeonIMUWrapper.h"
#include "CtreSimUtils/MockHookUtilities.h"
#include "com_ctre_phoenix_sensors_PigeonImuJNI.h"
#include "ctre/phoenix/cci/PigeonIMU_CCI.h"

#define GET_THREE_AXIS(type, capType, funcName, result, size) \
    type angles[size]; /* NOLINT */                           \
    funcName(ConvertToWrapper(handle), angles);               \
    j##type fill[size];                                       \
    for (int i = 0; i < size; ++i)                            \
    {                                                         \
        fill[i] = angles[i];                                  \
    }                                                         \
    env->Set##capType##ArrayRegion(result, 0, size, fill);    \
    return 0;
namespace
{
void* ConvertToWrapper(jlong aHandle)
{
    return reinterpret_cast<SnobotSim::CtrePigeonIMUWrapper*>(aHandle);
}
} // namespace

extern "C" {

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1new_1PigeonImu
 * Method:    1Talon
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1new_1PigeonImu_1Talon
  (JNIEnv*, jclass, jint deviceNumber)
{
    return (jlong)c_PigeonIMU_Create2(deviceNumber);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1new
 * Method:    1PigeonImu
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1new_1PigeonImu
  (JNIEnv*, jclass, jint deviceNumber)
{
    return (jlong)c_PigeonIMU_Create1(deviceNumber);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1destroy
 * Method:    1PigeonImu
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1destroy_1PigeonImu
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_PigeonIMU_Destroy(ConvertToWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1ConfigSetCustomParam
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigSetCustomParam
  (JNIEnv*, jclass, jlong handle, jint newValue, jint paramIndex,
   jint timeoutMs)
{
    return (jint)c_PigeonIMU_ConfigSetCustomParam(ConvertToWrapper(handle), newValue, paramIndex, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1ConfigGetCustomParam
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigGetCustomParam
  (JNIEnv*, jclass, jlong handle, jint paramIndex, jint timoutMs)
{
    int readValue = 0;
    c_PigeonIMU_ConfigGetCustomParam(ConvertToWrapper(handle), &readValue, paramIndex, timoutMs);
    return readValue;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1ConfigSetParameter
 * Signature: (JIDIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigSetParameter
  (JNIEnv*, jclass, jlong handle, jint param, jdouble value, jint subValue,
   jint ordinal, jint timeoutMs)
{
    return (jint)c_PigeonIMU_ConfigSetParameter(ConvertToWrapper(handle), param, value, subValue, ordinal, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1ConfigGetParameter
 * Signature: (JIII)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigGetParameter
  (JNIEnv*, jclass, jlong handle, jint param, jint ordinal, jint timeoutMs)
{
    double value = 0;
    c_PigeonIMU_ConfigGetParameter(ConvertToWrapper(handle), param, &value, ordinal, timeoutMs);
    return value;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1ConfigFactoryDefault
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ConfigFactoryDefault
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_PigeonIMU_ConfigFactoryDefault(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetStatusFramePeriod
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint periodMs, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetStatusFramePeriod(ConvertToWrapper(handle), frame, periodMs, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetYaw
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetYaw
  (JNIEnv*, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetYaw(ConvertToWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1AddYaw
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1AddYaw
  (JNIEnv*, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_AddYaw(ConvertToWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetYawToCompass
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetYawToCompass
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetYawToCompass(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetFusedHeading
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetFusedHeading
  (JNIEnv*, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetFusedHeading(ConvertToWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1AddFusedHeading
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1AddFusedHeading
  (JNIEnv*, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_AddFusedHeading(ConvertToWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetFusedHeadingToCompass
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetFusedHeadingToCompass
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetFusedHeadingToCompass(ConvertToWrapper(handle), timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetAccumZAngle
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetAccumZAngle
  (JNIEnv*, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetAccumZAngle(ConvertToWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetTemperatureCompensationDisable
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetTemperatureCompensationDisable
  (JNIEnv*, jclass, jlong handle, jint bTempCompDisable, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetTemperatureCompensationDisable(ConvertToWrapper(handle), bTempCompDisable, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetCompassDeclination
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetCompassDeclination
  (JNIEnv*, jclass, jlong handle, jdouble angleDegOffset, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetCompassDeclination(ConvertToWrapper(handle), angleDegOffset, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetCompassAngle
 * Signature: (JDI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetCompassAngle
  (JNIEnv*, jclass, jlong handle, jdouble angleDeg, jint timeoutMs)
{
    return (jint)c_PigeonIMU_SetCompassAngle(ConvertToWrapper(handle), angleDeg, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1EnterCalibrationMode
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1EnterCalibrationMode
  (JNIEnv*, jclass, jlong handle, jint calMode, jint timeoutMs)
{
    return (jint)c_PigeonIMU_EnterCalibrationMode(ConvertToWrapper(handle), calMode, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetGeneralStatus
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetGeneralStatus
  (JNIEnv*, jclass, jlong handle, jdoubleArray)
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
    c_PigeonIMU_GetGeneralStatus(ConvertToWrapper(handle), &state, &currentMode, &calibrationError, &bCalIsBooting, &tempC, &upTimeSec, &noMotionBiasCount, &tempCompensationCount, &lastError);
    return state;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1Get6dQuaternion
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1Get6dQuaternion
  (JNIEnv* env, jclass, jlong handle, jdoubleArray wxyz)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_Get6dQuaternion, wxyz, 4);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetYawPitchRoll
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetYawPitchRoll
  (JNIEnv* env, jclass, jlong handle, jdoubleArray ypr)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_GetYawPitchRoll, ypr, 3);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetAccumGyro
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetAccumGyro
  (JNIEnv* env, jclass, jlong handle, jdoubleArray xyz_deg)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_GetAccumGyro, xyz_deg, 3);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetAbsoluteCompassHeading
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetAbsoluteCompassHeading
  (JNIEnv*, jclass, jlong handle)
{
    double value = 0;
    c_PigeonIMU_GetAbsoluteCompassHeading(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetCompassHeading
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetCompassHeading
  (JNIEnv*, jclass, jlong handle)
{
    double value = 0;
    c_PigeonIMU_GetCompassHeading(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetCompassFieldStrength
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetCompassFieldStrength
  (JNIEnv*, jclass, jlong handle)
{
    double value = 0;
    c_PigeonIMU_GetCompassFieldStrength(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetTemp
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetTemp
  (JNIEnv*, jclass, jlong handle)
{
    double value = 0;
    c_PigeonIMU_GetTemp(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetUpTime
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetUpTime
  (JNIEnv*, jclass, jlong handle)
{
    int value = 0;
    c_PigeonIMU_GetUpTime(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetRawMagnetometer
 * Signature: (J[S)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetRawMagnetometer
  (JNIEnv* env, jclass, jlong handle, jshortArray rm_xyz)
{
    GET_THREE_AXIS(short, Short, c_PigeonIMU_GetRawMagnetometer, rm_xyz, 3); // NOLINT
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetBiasedMagnetometer
 * Signature: (J[S)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetBiasedMagnetometer
  (JNIEnv* env, jclass, jlong handle, jshortArray bm_xyz)
{
    GET_THREE_AXIS(short, Short, c_PigeonIMU_GetBiasedMagnetometer, bm_xyz, 3); // NOLINT
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetBiasedAccelerometer
 * Signature: (J[S)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetBiasedAccelerometer
  (JNIEnv* env, jclass, jlong handle, jshortArray ba_xyz)
{
    GET_THREE_AXIS(short, Short, c_PigeonIMU_GetBiasedAccelerometer, ba_xyz, 3); // NOLINT
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetRawGyro
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetRawGyro
  (JNIEnv* env, jclass, jlong handle, jdoubleArray xyz_dps)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_GetRawGyro, xyz_dps, 3);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetAccelerometerAngles
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetAccelerometerAngles
  (JNIEnv* env, jclass, jlong handle, jdoubleArray tiltAngles)
{
    GET_THREE_AXIS(double, Double, c_PigeonIMU_GetAccelerometerAngles, tiltAngles, 3);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetFusedHeading
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetFusedHeading
  (JNIEnv* env, jclass, jlong handle, jdoubleArray result)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return 0;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetState
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetState
  (JNIEnv*, jclass, jlong handle)
{
    int state = 0;
    c_PigeonIMU_GetState(ConvertToWrapper(handle), &state);
    return state;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetResetCount
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetResetCount
  (JNIEnv*, jclass, jlong handle)
{
    int value = 0;
    c_PigeonIMU_GetResetCount(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetResetFlags
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetResetFlags
  (JNIEnv*, jclass, jlong handle)
{
    int value = 0;
    c_PigeonIMU_GetResetFlags(ConvertToWrapper(handle), &value);
    return value;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetFirmwareVersion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetFirmwareVersion
  (JNIEnv*, jclass, jlong handle)
{
    int firmwareVers = 0;
    c_PigeonIMU_GetFirmwareVersion(ConvertToWrapper(handle), &firmwareVers);
    return firmwareVers;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetLastError
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetLastError
  (JNIEnv*, jclass, jlong handle)
{
    return (jint)c_PigeonIMU_GetLastError(ConvertToWrapper(handle));
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1HasResetOccurred
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1HasResetOccurred
  (JNIEnv*, jclass, jlong handle)
{
    bool hasReset = false;
    c_PigeonIMU_HasResetOccurred(ConvertToWrapper(handle), &hasReset);
    return hasReset;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetStatusFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetStatusFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint timeoutMs)
{
    int periodMs = 0;
    c_PigeonIMU_GetStatusFramePeriod(ConvertToWrapper(handle), frame, &periodMs, timeoutMs);
    return periodMs;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1SetControlFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1SetControlFramePeriod
  (JNIEnv*, jclass, jlong handle, jint frame, jint periodMs)
{
    return (jint)c_PigeonIMU_SetControlFramePeriod(ConvertToWrapper(handle), frame, periodMs);
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetFaults
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_PigeonIMU_GetFaults(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1GetStickyFaults
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetStickyFaults
  (JNIEnv*, jclass, jlong handle)
{
    int param = 0;
    c_PigeonIMU_GetStickyFaults(ConvertToWrapper(handle), &param);
    return param;
}

/*
 * Class:     com_ctre_phoenix_sensors_PigeonImuJNI_JNI
 * Method:    1ClearStickyFaults
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1ClearStickyFaults
  (JNIEnv*, jclass, jlong handle, jint timeoutMs)
{
    return (jint)c_PigeonIMU_ClearStickyFaults(ConvertToWrapper(handle), timeoutMs);
}

} // extern "C"
