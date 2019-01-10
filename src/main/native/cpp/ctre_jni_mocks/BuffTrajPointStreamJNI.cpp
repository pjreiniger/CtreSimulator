
#include <jni.h>

#include <cassert>

#include "CtreSimMocks/MockHookUtilities.h"
#include "CtreSimMocks/CtreBuffTrajPointStreamWrapper.h"

#include "com_ctre_phoenix_motion_BuffTrajPointStreamJNI.h"
#include "ctre/phoenix/cci/BuffTrajPointStream_CCI.h"

void* ConvertToBuffTrajPointStream(jlong aHandle)
{
    return reinterpret_cast<SnobotSim::CtreBuffTrajPointStreamWrapper*>(aHandle);
}

extern "C" {
/*
 * Class:     com_ctre_phoenix_motion_BuffTrajPointStreamJNI
 * Method:    Create1
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_ctre_phoenix_motion_BuffTrajPointStreamJNI_Create1
  (JNIEnv *, jclass)
{
    return (jlong) c_BuffTrajPointStream_Create1();
}

/*
 * Class:     com_ctre_phoenix_motion_BuffTrajPointStreamJNI
 * Method:    DestroyAll
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_motion_BuffTrajPointStreamJNI_DestroyAll
  (JNIEnv *, jclass)
{
    c_BuffTrajPointStream_DestroyAll();
}

/*
 * Class:     com_ctre_phoenix_motion_BuffTrajPointStreamJNI
 * Method:    Destroy
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motion_BuffTrajPointStreamJNI_Destroy
  (JNIEnv *, jclass, jlong handle)
{
    return c_BuffTrajPointStream_Destroy(ConvertToBuffTrajPointStream(handle));
}

/*
 * Class:     com_ctre_phoenix_motion_BuffTrajPointStreamJNI
 * Method:    Clear
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motion_BuffTrajPointStreamJNI_Clear
  (JNIEnv *, jclass, jlong handle)
{
    return c_BuffTrajPointStream_Clear(ConvertToBuffTrajPointStream(handle));
}

/*
 * Class:     com_ctre_phoenix_motion_BuffTrajPointStreamJNI
 * Method:    Write
 * Signature: (JDDDDDDIIZZIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_motion_BuffTrajPointStreamJNI_Write
  (JNIEnv *, jclass, jlong handle, jdouble position, jdouble velocity, jdouble arbFeedFwd, jdouble auxiliaryPos,
          jdouble auxiliaryVel, jdouble auxiliaryArbFeedFwd, jint profileSlotSelect0, jint profileSlotSelect1,
          jboolean isLastPoint, jboolean zeroPos, jint timeDur, jboolean useAuxPID)
{
    return c_BuffTrajPointStream_Write(ConvertToBuffTrajPointStream(handle), position, velocity, arbFeedFwd, auxiliaryPos,
            auxiliaryVel, auxiliaryArbFeedFwd, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos, timeDur, useAuxPID);
}


}  // extern "C"
