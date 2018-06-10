
#include <jni.h>

#include <cassert>

#include "com_ctre_phoenix_CTRLoggerJNI.h"
#include "ctre/phoenix/CCI/Logger_CCI.h"

extern "C" {

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

}  // extern "C"
