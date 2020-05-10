
#include <jni.h>

#include "CtreSimMocks/MockHooks.h"
#include "com_snobot_simulator_ctre_CtreJni.h"

namespace SnobotSimJava
{
JavaVM* sJvm = NULL;
static jclass sCtreBufferCallbackClazz;
static jmethodID sCtreBufferCallbackCallback;
static jobject sCtreMotorCallbackObject = NULL;
static jobject sCtrePigeonCallbackObject = NULL;
static jobject sCtreCanifierCallbackObject = NULL;
static jobject sCtrTrajBufferPointStreamCallbackObject = NULL;
} // namespace SnobotSimJava

extern "C" {

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved)
{
    SnobotSimJava::sJvm = vm;

    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
    {
        return JNI_ERR;
    }

    jclass localCallbackClazz = env->FindClass("com/snobot/simulator/ctre/CtreCallback");
    SnobotSimJava::sCtreBufferCallbackClazz = static_cast<jclass>(env->NewGlobalRef(localCallbackClazz));
    env->DeleteLocalRef(localCallbackClazz);

    if (!SnobotSimJava::sCtreBufferCallbackClazz)
    {
        return JNI_ERR;
    }

    SnobotSimJava::sCtreBufferCallbackCallback = env->GetMethodID(SnobotSimJava::sCtreBufferCallbackClazz, "callback", "(Ljava/lang/String;ILjava/nio/ByteBuffer;I)V");
    if (!SnobotSimJava::sCtreBufferCallbackCallback)
    {
        return JNI_ERR;
    }

    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved)
{
    JNIEnv* env;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
    {
        return;
    }

    env->DeleteGlobalRef(SnobotSimJava::sCtreBufferCallbackClazz);

    if (SnobotSimJava::sCtreMotorCallbackObject)
    {
        env->DeleteGlobalRef(SnobotSimJava::sCtreMotorCallbackObject);
    }
    SnobotSimJava::sCtreMotorCallbackObject = nullptr;

    if (SnobotSimJava::sCtrePigeonCallbackObject)
    {
        env->DeleteGlobalRef(SnobotSimJava::sCtrePigeonCallbackObject);
    }
    SnobotSimJava::sCtrePigeonCallbackObject = nullptr;

    if (SnobotSimJava::sCtreCanifierCallbackObject)
    {
        env->DeleteGlobalRef(SnobotSimJava::sCtreCanifierCallbackObject);
    }
    SnobotSimJava::sCtreCanifierCallbackObject = nullptr;

    SnobotSimJava::sJvm = NULL;
}

/*
 * Class:     com_snobot_simulator_ctre_CtreJni
 * Method:    registerCanMotorCallback
 * Signature: (Ljava/lang/Object;)V
 */
JNIEXPORT void JNICALL
Java_com_snobot_simulator_ctre_CtreJni_registerCanMotorCallback
  (JNIEnv* aEnv, jclass, jobject callback)
{
    SnobotSimJava::sCtreMotorCallbackObject = aEnv->NewGlobalRef(callback);
    auto callbackFunc = [](const char* name,
                                uint32_t messageId,
                                uint8_t* buffer,
                                int length) {
        JavaVMAttachArgs args = { JNI_VERSION_1_6, 0, 0 };
        JNIEnv* env;
        SnobotSimJava::sJvm->AttachCurrentThread(reinterpret_cast<void**>(&env), &args);

        jobject dataBuffer = env->NewDirectByteBuffer(const_cast<uint8_t*>(buffer), static_cast<uint32_t>(length));
        jstring nameString = env->NewStringUTF(name);

        if (SnobotSimJava::sCtreMotorCallbackObject)
        {
            env->CallVoidMethod(SnobotSimJava::sCtreMotorCallbackObject, SnobotSimJava::sCtreBufferCallbackCallback, nameString,
                    messageId, dataBuffer, length);
        }
    };
    SnobotSim::SetMotControllerCallback(callbackFunc);
}

/*
 * Class:     com_snobot_simulator_ctre_CtreJni
 * Method:    cancelCanMotorCallback
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_com_snobot_simulator_ctre_CtreJni_cancelCanMotorCallback
  (JNIEnv* env, jclass)
{
    if (SnobotSimJava::sCtreMotorCallbackObject)
    {
        env->DeleteGlobalRef(SnobotSimJava::sCtreMotorCallbackObject);
    }
    SnobotSimJava::sCtreMotorCallbackObject = NULL;
}

/*
 * Class:     com_snobot_simulator_ctre_CtreJni
 * Method:    registerCanPigeonImuCallback
 * Signature: (Ljava/lang/Object;)V
 */
JNIEXPORT void JNICALL
Java_com_snobot_simulator_ctre_CtreJni_registerCanPigeonImuCallback
  (JNIEnv* aEnv, jclass, jobject callback)
{
    SnobotSimJava::sCtrePigeonCallbackObject = aEnv->NewGlobalRef(callback);

    auto callbackFunc = [](const char* name,
                                uint32_t messageId,
                                uint8_t* buffer,
                                int length) {
        JavaVMAttachArgs args = { JNI_VERSION_1_6, 0, 0 };
        JNIEnv* env;
        SnobotSimJava::sJvm->AttachCurrentThread(reinterpret_cast<void**>(&env), &args);

        jobject dataBuffer = env->NewDirectByteBuffer(const_cast<uint8_t*>(buffer), static_cast<uint32_t>(length));
        jstring nameString = env->NewStringUTF(name);

        if (SnobotSimJava::sCtrePigeonCallbackObject)
        {
            env->CallVoidMethod(SnobotSimJava::sCtrePigeonCallbackObject, SnobotSimJava::sCtreBufferCallbackCallback, nameString,
                    messageId, dataBuffer, length);
        }
    };

    SnobotSim::SetPigeonCallback(callbackFunc);
}

/*
 * Class:     com_snobot_simulator_ctre_CtreJni
 * Method:    cancelCanPigeonImuCallback
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_com_snobot_simulator_ctre_CtreJni_cancelCanPigeonImuCallback
  (JNIEnv* env, jclass)
{
    if (SnobotSimJava::sCtrePigeonCallbackObject)
    {
        env->DeleteGlobalRef(SnobotSimJava::sCtrePigeonCallbackObject);
    }
    SnobotSimJava::sCtrePigeonCallbackObject = NULL;
}

/*
 * Class:     com_snobot_simulator_ctre_CtreJni
 * Method:    registerCanCanifierCallback
 * Signature: (Ljava/lang/Object;)V
 */
JNIEXPORT void JNICALL
Java_com_snobot_simulator_ctre_CtreJni_registerCanCanifierCallback
  (JNIEnv* aEnv, jclass, jobject callback)
{
    SnobotSimJava::sCtreCanifierCallbackObject = aEnv->NewGlobalRef(callback);

    auto callbackFunc = [](const char* name,
                                uint32_t messageId,
                                uint8_t* buffer,
                                int length) {
        JavaVMAttachArgs args = { JNI_VERSION_1_6, 0, 0 };
        JNIEnv* env;
        SnobotSimJava::sJvm->AttachCurrentThread(reinterpret_cast<void**>(&env), &args);

        jobject dataBuffer = env->NewDirectByteBuffer(const_cast<uint8_t*>(buffer), static_cast<uint32_t>(length));
        jstring nameString = env->NewStringUTF(name);

        if (SnobotSimJava::sCtreCanifierCallbackObject)
        {
            env->CallVoidMethod(SnobotSimJava::sCtreCanifierCallbackObject, SnobotSimJava::sCtreBufferCallbackCallback, nameString,
                    messageId, dataBuffer, length);
        }
    };

    SnobotSim::SetCanifierCallback(callbackFunc);
}

/*
 * Class:     com_snobot_simulator_ctre_CtreJni
 * Method:    cancelCanCanifierCallback
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_com_snobot_simulator_ctre_CtreJni_cancelCanCanifierCallback
  (JNIEnv* env, jclass)
{
    if (SnobotSimJava::sCtreCanifierCallbackObject)
    {
        env->DeleteGlobalRef(SnobotSimJava::sCtreCanifierCallbackObject);
    }
    SnobotSimJava::sCtreCanifierCallbackObject = NULL;
}

/*
 * Class:     com_snobot_simulator_ctre_CtreJni
 * Method:    registerCanBuffTrajPointStreamCallback
 * Signature: (Ljava/lang/Object;)V
 */
JNIEXPORT void JNICALL
Java_com_snobot_simulator_ctre_CtreJni_registerCanBuffTrajPointStreamCallback
  (JNIEnv* aEnv, jclass, jobject callback)
{
    SnobotSimJava::sCtrTrajBufferPointStreamCallbackObject = aEnv->NewGlobalRef(callback);

    auto callbackFunc = [](const char* name,
                                uint32_t messageId,
                                uint8_t* buffer,
                                int length) {
        JavaVMAttachArgs args = { JNI_VERSION_1_6, 0, 0 };
        JNIEnv* env;
        SnobotSimJava::sJvm->AttachCurrentThread(reinterpret_cast<void**>(&env), &args);

        jobject dataBuffer = env->NewDirectByteBuffer(const_cast<uint8_t*>(buffer), static_cast<uint32_t>(length));
        jstring nameString = env->NewStringUTF(name);

        if (SnobotSimJava::sCtrTrajBufferPointStreamCallbackObject)
        {
            env->CallVoidMethod(SnobotSimJava::sCtrTrajBufferPointStreamCallbackObject, SnobotSimJava::sCtreBufferCallbackCallback, nameString,
                    messageId, dataBuffer, length);
        }
    };

    SnobotSim::SetBuffTrajPiontStreamCallback(callbackFunc);
}

/*
 * Class:     com_snobot_simulator_ctre_CtreJni
 * Method:    cancelCanCanBuffTrajPointStreamCallback
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_com_snobot_simulator_ctre_CtreJni_cancelCanCanBuffTrajPointStreamCallback
  (JNIEnv* env, jclass)
{
    if (SnobotSimJava::sCtrTrajBufferPointStreamCallbackObject)
    {
        env->DeleteGlobalRef(SnobotSimJava::sCtrTrajBufferPointStreamCallbackObject);
    }
    SnobotSimJava::sCtrTrajBufferPointStreamCallbackObject = NULL;
}

} // extern "C"
