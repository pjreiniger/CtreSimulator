
#include <jni.h>

#include <cassert>

#include "CtreSimUtils/MockHookUtilities.h"
#include "com_ctre_phoenix_CTRLoggerJNI.h"
#include "ctre/phoenix/cci/Logger_CCI.h"

extern "C" {

/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger
 * Method:    1Close
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1Close
  (JNIEnv*, jclass)
{
    c_Logger_Close();
}

/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger
 * Method:    1Log
 * Signature: (ILjava/lang/String;Ljava/lang/String;)I
 */
JNIEXPORT jint JNICALL
Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1Log
  (JNIEnv*, jclass, jint errorCode, jstring originPtr, jstring stackTracePtr)
{
    const char* origin = NULL;
    const char* stacktrace = NULL;
    int hierarchy = 0;

    return static_cast<int>(c_Logger_Log((ctre::phoenix::ErrorCode)errorCode, origin, hierarchy, stacktrace));
}

/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger
 * Method:    1Open
 * Signature: (I)V
 */
JNIEXPORT void JNICALL
Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1Open
  (JNIEnv*, jclass, jint language)
{
    c_Logger_Open(language, true);
}

/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger
 * Method:    1GetShort
 * Signature: (I)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1GetShort
  (JNIEnv* env, jclass, jint)
{
    jstring nameString = env->NewStringUTF("");
    LOG_UNSUPPORTED_CAN_FUNC("");
    return nameString;
}

/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger
 * Method:    1GetLong
 * Signature: (I)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1GetLong
  (JNIEnv* env, jclass, jint)
{
    jstring nameString = env->NewStringUTF("");
    LOG_UNSUPPORTED_CAN_FUNC("");
    return nameString;
}

} // extern "C"
