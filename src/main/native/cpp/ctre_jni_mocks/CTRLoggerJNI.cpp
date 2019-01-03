
#include <jni.h>

#include <cassert>

#include "com_ctre_phoenix_CTRLoggerJNI.h"
#include "ctre/phoenix/CCI/Logger_CCI.h"

#include "CtreSimMocks/MockHookUtilities.h"

extern "C" {

/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI
 * Method:    JNI_Logger_Close
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1Close
  (JNIEnv *, jclass)
{
    c_Logger_Close();
}

/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI
 * Method:    JNI_Logger_Log
 * Signature: (ILjava/lang/String;Ljava/lang/String;)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1Log
  (JNIEnv *, jclass, jint errorCode, jstring originPtr, jstring stackTracePtr)
{
	const char* origin = NULL;
	const char* stacktrace = NULL;
	int hierarchy = 0;

	return (int) c_Logger_Log((ctre::phoenix::ErrorCode) errorCode, origin, hierarchy, stacktrace);
}

/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI
 * Method:    JNI_Logger_Open
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1Open
  (JNIEnv *, jclass, jint language)
{
    c_Logger_Open(language, true);
}


/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI
 * Method:    JNI_Logger_GetShort
 * Signature: (I)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1GetShort
  (JNIEnv * env, jclass, jint)
{
    jstring nameString = env->NewStringUTF("");
    LOG_UNSUPPORTED_CAN_FUNC("");
    return nameString;
}

/*
 * Class:     com_ctre_phoenix_CTRLoggerJNI
 * Method:    JNI_Logger_GetLong
 * Signature: (I)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_ctre_phoenix_CTRLoggerJNI_JNI_1Logger_1GetLong
  (JNIEnv * env, jclass, jint)
{
    jstring nameString = env->NewStringUTF("");
    LOG_UNSUPPORTED_CAN_FUNC("");
    return nameString;
}

}  // extern "C"
