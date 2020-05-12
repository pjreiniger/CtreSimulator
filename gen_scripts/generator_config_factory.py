from lib.generator_config import GeneratorConfig


__CCI_GETTER_TEMPLATE = """{
    RECEIVE_HELPER("{{ callback_name }}", {% for arg in args if arg.name != "handle" and arg.name != "timeoutMs" %}sizeof({{ "*" if "*" in arg.type }}{{arg.name}}){{ " + " if not loop.last }}{% endfor %});{% for arg in args if arg.name != "handle" and arg.name != "timeoutMs" %}
    PoplateReceiveResults(buffer, {{ "&" if "*" not in arg.type }}{{arg.name}}, buffer_pos);{% endfor %}
    {% if return_type == "ctre::phoenix::ErrorCode" %}return (ctre::phoenix::ErrorCode)0;{% endif %}
}"""

__CCI_SETTER_TEMPLATE = """{
    Send("{{ callback_name }}"{% for arg in args if arg.name != "handle" and arg.name != "timeoutMs" %}, {{arg.name}}{% endfor %});{% if return_type == "ctre::phoenix::ErrorCode" %}
    return (ctre::phoenix::ErrorCode)0;{% endif %}
}"""


def generator_config_factory():

    definitions = []

    definitions.append(__create_buff_traj_point_stream_config())
    definitions.append(__create_cancoder_config())
    definitions.append(__create_canifier_config())
    definitions.append(__create_mot_controller_config())
    definitions.append(__create_pigeon_config())


#         config.default_value_lookup["ctre::phoenix::motorcontrol::MotorCommutation *"] = " "
#         config.default_value_lookup["ctre::phoenix::sensors::SensorVelocityMeasPeriod *"] = " = 0"
#         config.default_value_lookup["ctre::phoenix::sensors::AbsoluteSensorRange *"] = " = 0"
#         config.default_value_lookup["ctre::phoenix::sensors::SensorInitializationStrategy *"] = " = 0"
#         config.default_value_lookup["ctre::phoenix::sensors::SensorTimeBase *"] = " = 0"


    return definitions


def __create_canifier_config():
    config = __create_config(
                cci_class_name="CANifier_CCI",
                cci_prefix="c_CANifier_",
                cci_wrapper_type="CtreCANifierWrapper",
                jni_package="com_ctre_phoenix_CANifierJNI",
                jni_class_name="CANifierJNI.cpp",
                java_class_name="CANifier",
            )
    
    config.jni_sanitized_name.append(("new_1CANifier", "Create1"))
    config.jni_sanitized_name.append(("destroy_1CANifier", "Destroy"))
    
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_CANifierJNI_JNI_1new_1CANifier'] = '    return (jlong)c_CANifier_Create1(deviceNumber);'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_CANifierJNI_JNI_1destroy_1CANifier'] = '    return c_CANifier_Destroy(ConvertToWrapper(handle));'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_CANifierJNI_JNI_1GetGeneralInputs'] = '''
    const int kCapacity = 11;
    bool allPinsArray[kCapacity];

    c_CANifier_GetGeneralInputs(ConvertToWrapper(handle), allPinsArray, kCapacity);'''
    
    config.full_jni_package = "com_ctre_phoenix_CANifierJNI_JNI_1"
#         output.include_jni_in_method_name = True

    return config


def __create_buff_traj_point_stream_config():
    config =  __create_config(
                cci_class_name="BuffTrajPointStream_CCI",
                cci_prefix="c_BuffTrajPointStream_",
                cci_wrapper_type="CtreBuffTrajPointStreamWrapper",
                jni_package="com_ctre_phoenix_motion_BuffTrajPointStreamJNI",
                jni_class_name="BuffTrajPointStreamJNI.cpp",
                java_class_name="")

    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_motion_BuffTrajPointStreamJNI_Create1'] = '    return (jlong)c_BuffTrajPointStream_Create1();'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_motion_BuffTrajPointStreamJNI_DestroyAll'] = '        c_BuffTrajPointStream_DestroyAll();'
    
    return config


def __create_cancoder_config():
    config =  __create_config(
                cci_class_name="CANCoder_CCI",
                cci_prefix="c_CANCoder_",
                cci_wrapper_type="CtreCANCoderWrapper",
                jni_package="com_ctre_phoenix_sensors_CANCoderJNI",
                jni_class_name="CanCoderJNI.cpp",
                java_class_name="",
            )
    
    config.jni_sanitized_name.append(("Create", "Create1"))
    
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_sensors_CANCoderJNI_Create'] = '    return (jlong)c_CANCoder_Create1(deviceNumber);'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_sensors_CANCoderJNI_GetLastUnitString'] = '    LOG_UNSUPPORTED_CAN_FUNC("");\n    return nullptr;'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_sensors_CANCoderJNI_ConfigFeedbackCoefficient'] = '''    char* unitString = nullptr;
    c_CANCoder_ConfigFeedbackCoefficient(ConvertToWrapper(handle), sensorCoefficient, unitString, (ctre::phoenix::sensors::SensorTimeBase)sensortimeBase, timeoutMs);
    return 0;'''
    
    config.cpp_enum_types.append("ctre::phoenix::sensors::MagnetFieldStrength")
    config.cpp_enum_types.append("ctre::phoenix::sensors::AbsoluteSensorRange")
    config.cpp_enum_types.append("ctre::phoenix::sensors::SensorInitializationStrategy")
    config.cpp_enum_types.append("ctre::phoenix::sensors::SensorTimeBase")
    
    config.default_value_lookup["ctre::phoenix::sensors::MagnetFieldStrength *"] = " = 0"
    
    return config


def __create_mot_controller_config():
    config = __create_config(
                cci_class_name="MotController_CCI",
                cci_prefix="c_MotController_",
                cci_wrapper_type="CtreMotControllerWrapper",
                jni_package="com_ctre_phoenix_motorcontrol_can_MotControllerJNI",
                jni_class_name="MotControllerJNI.cpp",
                java_class_name="BaseMotorController")
    
    config.jni_sanitized_name.append(("Set_14", "Set_4"))
    config.jni_sanitized_name.append(("SetInverted_12", "SetInverted_2"))
    config.jni_sanitized_name.append(("Config_1", "Config_"))
    config.jni_sanitized_name.append(("JNI_1destroy_1MotController", "Destroy"))
    config.jni_sanitized_name.append(("Set_14", "Set_4"))
    config.jni_sanitized_name.append(("SetInverted_12", "SetInverted_2"))
    config.jni_sanitized_name.append(("ConfigPulseWidthPeriod_1EdgesPerRot", "ConfigPulseWidthPeriod_EdgesPerRot"))
    config.jni_sanitized_name.append(("ConfigPulseWidthPeriod_1FilterWindowSz", "ConfigPulseWidthPeriod_FilterWindowSz"))
    config.jni_sanitized_name.append(("ConfigBrakeCurrentLimit", "ConfigBrakeCurrentLimit"))
    config.jni_sanitized_name.append(("ConfigBrakeCurrentLimitEnable", "ConfigBrakeCurrentLimitEnable"))
    config.jni_sanitized_name.append(("ConfigAbsoluteSensorRange", "ConfigAbsoluteSensorRange"))
    config.jni_sanitized_name.append(("ConfigMagnetOffset", "ConfigMagnetOffset"))
    
    config.default_value_lookup["ctre::phoenix::motorcontrol::MotorCommutation *"] = ""
    
    config.jni_sanitized_name.append(("Create", "Create1"))
    config.jni_sanitized_name.append(("Create12", "Create2"))
    config.jni_sanitized_name.append(("GetActiveTrajectoryPosition3", "GetActiveTrajectoryPosition_3"))
    config.jni_sanitized_name.append(("GetActiveTrajectoryVelocity3", "GetActiveTrajectoryVelocity_3"))
    config.jni_sanitized_name.append(("GetActiveTrajectoryArbFeedFwd3", "GetActiveTrajectoryArbFeedFwd_3"))
    config.jni_sanitized_name.append(("PushMotionProfileTrajectory2", "PushMotionProfileTrajectory_2"))
    config.jni_sanitized_name.append(("PushMotionProfileTrajectory3", "PushMotionProfileTrajectory_3"))
    config.jni_sanitized_name.append(("GetMotionProfileStatus2", "GetMotionProfileStatus_2"))
    config.jni_sanitized_name.append(("GetIntegratedSensorPosition", "GetIntegratedSensorPosition"))
    config.jni_sanitized_name.append(("GetIntegratedSensorAbsolutePosition", "GetIntegratedSensorAbsolutePosition"))
    config.jni_sanitized_name.append(("GetIntegratedSensorVelocity", "GetIntegratedSensorVelocity"))
    config.jni_sanitized_name.append(("ConfigSensorInitializationStrategy", "ConfigSensorInitializationStrategy"))
    
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigSupplyCurrentLimit'] = '    LOG_UNSUPPORTED_CAN_FUNC("");\n    return 0;'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetSupplyCurrentLimit'] = '    LOG_UNSUPPORTED_CAN_FUNC("");\n    return 0;'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_ConfigGetStatorCurrentLimit'] = '    LOG_UNSUPPORTED_CAN_FUNC("");\n    return 0;'
    
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Create'] = '    return (jlong)c_MotController_Create1(baseArbId);'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_Create2'] = '    return (jlong)c_MotController_Create2(deviceID, "");'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotionProfileStatus'] = '''
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

    return 0;'''
    
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_GetMotionProfileStatus2'] = '''
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

    return 0;'''

    config.cpp_enum_types.append("ctre::phoenix::motorcontrol::ControlMode")
    config.cpp_enum_types.append("ctre::phoenix::motorcontrol::MotorCommutation")
    config.cpp_enum_types.append("ctre::phoenix::sensors::AbsoluteSensorRange")
    config.cpp_enum_types.append("ctre::phoenix::sensors::SensorInitializationStrategy")
    
#     config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1Get6dQuaternion"] = ''
    
    return config


def __create_pigeon_config():
    config = __create_config(
            cci_class_name="PigeonIMU_CCI",
            cci_prefix="c_PigeonIMU_",
            cci_wrapper_type="CtrePigeonIMUWrapper",
            jni_package="com_ctre_phoenix_sensors_PigeonImuJNI",
            jni_class_name="PigeonImuJNI.cpp",
            java_class_name="PigeonIMU",
        )
    

    default = [{'name': 'True'}]

    getter_overrides = {}
    getter_overrides['c_PigeonIMU_GetAccelerometerAngles'] = default
    getter_overrides['c_PigeonIMU_GetRawGyro'] = default
    getter_overrides['c_PigeonIMU_GetFusedHeading1'] = default
    getter_overrides['c_PigeonIMU_GetAccumGyro'] = default
    getter_overrides['c_PigeonIMU_GetYawPitchRoll'] = default
    getter_overrides['c_PigeonIMU_Get6dQuaternion'] = default
    getter_overrides['c_PigeonIMU_GetRawMagnetometer'] = default
    getter_overrides['c_PigeonIMU_GetBiasedMagnetometer'] = default
    getter_overrides['c_PigeonIMU_GetBiasedAccelerometer'] = default
    getter_overrides['c_CANifier_GetPWMInput'] = default
    config.cci_getter_overrides = getter_overrides
    
    config.full_jni_package = "com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1"
    
    config.extra_jni_header = """#define GET_THREE_AXIS(type, capType, funcName, result, size)      \
                                                           \
    type angles[size]; /* NOLINT */                        \
    funcName(ConvertToWrapper(handle), angles);            \
                                                           \
    j##type fill[size];                                    \
    for (int i = 0; i < size; ++i)                         \
    {                                                      \
        fill[i] = angles[i];                               \
    }                                                      \
                                                           \
    env->Set##capType##ArrayRegion(result, 0, size, fill); \
    return 0;"""
    
    config.jni_sanitized_name.append(("new_1PigeonImu", "Create1"))
    config.jni_sanitized_name.append(("Create1_1Talon", "Create1"))
    config.jni_sanitized_name.append(("new_1PigeonImu_1Talon", "Create2"))
    config.jni_sanitized_name.append(("destroy_1PigeonImu", "Destroy"))

    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1new_1PigeonImu_1Talon"] = '    return (jlong)c_PigeonIMU_Create2(deviceNumber);'
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1new_1PigeonImu"] = '    return (jlong)c_PigeonIMU_Create1(deviceNumber);'
    
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1Get6dQuaternion"] = 'GET_THREE_AXIS(double, Double, c_PigeonIMU_Get6dQuaternion, wxyz, 4);'
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetYawPitchRoll"] = 'GET_THREE_AXIS(double, Double, c_PigeonIMU_GetYawPitchRoll, ypr, 3);'
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetAccumGyro"] = 'GET_THREE_AXIS(double, Double, c_PigeonIMU_GetAccumGyro, xyz_deg, 3);'
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetRawMagnetometer"] = 'GET_THREE_AXIS(short, Short, c_PigeonIMU_GetRawMagnetometer, rm_xyz, 3); // NOLINT'
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetBiasedMagnetometer"] = 'GET_THREE_AXIS(short, Short, c_PigeonIMU_GetBiasedMagnetometer, bm_xyz, 3); // NOLINT'
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetBiasedAccelerometer"] = 'GET_THREE_AXIS(short, Short, c_PigeonIMU_GetBiasedAccelerometer, ba_xyz, 3); // NOLINT'
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetRawGyro"] = 'GET_THREE_AXIS(double, Double, c_PigeonIMU_GetRawGyro, xyz_dps, 3);'
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetAccelerometerAngles"] = 'GET_THREE_AXIS(double, Double, c_PigeonIMU_GetAccelerometerAngles, tiltAngles, 3);'
    config.cci_overriden_function_bodies["Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1GetFusedHeading"] = 'GET_THREE_AXIS(double, Double, c_PigeonIMU_GetFusedHeading1, 3);'
    config.cci_overriden_function_bodies['Java_com_ctre_phoenix_sensors_CANCoderJNI_GetLastUnitString'] = """
    char toFill = 0;
    int numBytesFilled = 0;
    c_CANCoder_GetLastUnitString(ConvertToWrapper(handle), &toFill, toFillByteSz, &numBytesFilled);
    return toFill;"""

    return config


def __create_config(**kargs):
    
    class CtreGeneratorConfig(GeneratorConfig):
        def get_stripped_cci_class_name(self):
            return self.cci_class_name[:-4]
            
        def get_wrapper_class_name(self):
            return "Ctre" + self.get_stripped_cci_class_name() + "Wrapper"
    
    output = CtreGeneratorConfig(**kargs)

#     if output.jni_package == "com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1":
#         output.full_jni_package = "com_ctre_phoenix_sensors_PigeonImuJNI_JNI_1"
#     print(output.full_jni_package)
#     if output.jni_package == "com_ctre_phoenix_sensors_PigeonImuJNI":
#         print("OVERRIDING FULL NAME")
#         output.full_jni_package += "JNI_11"
#         output.include_jni_in_method_name = True
#     elif output.jni_package == "com_ctre_phoenix_CANifierJNI":
#         print("OVERRIDING FULL NAME")
#         output.full_jni_package += "JNI_11"
#         output.include_jni_in_method_name = True
#     else:
    output.full_jni_package += "_"

    output.cci_getter_template = __CCI_GETTER_TEMPLATE
    output.cci_setter_template = __CCI_SETTER_TEMPLATE
    output.error_ret_code = 'ctre::phoenix::ErrorCode'

    if "BuffTraj" in output.cci_class_name:
        output.cci_overriden_function_bodies[kargs['cci_prefix'] + 'Create1'] = '{\n    auto* output = new SnobotSim::%s();\n    return output;\n}' % kargs['cci_wrapper_type']
    else:
        output.cci_overriden_function_bodies[kargs['cci_prefix'] + 'Create1'] = '{\n    auto* output = new SnobotSim::%s(deviceNumber);\n    return output;\n}' % kargs['cci_wrapper_type']
    output.cci_overriden_function_bodies[kargs['cci_prefix'] + 'Create2'] = '{\n    auto* output = new SnobotSim::%s(deviceNumber);\n    return output;\n}' % kargs['cci_wrapper_type']
    output.cci_overriden_function_bodies[kargs['cci_prefix'] + 'DestroyAll'] = '{\n    LOG_UNSUPPORTED_CAN_FUNC("");\n}'
    output.cci_overriden_function_bodies[kargs['cci_prefix'] + 'Destroy'] = '{\n    auto* wrapper = ConvertToWrapper(handle);\n    delete wrapper;\n    return (ctre::phoenix::ErrorCode)0;\n}'
    output.cci_overriden_function_bodies[kargs['cci_prefix'] + 'GetLastError'] = '{\n    return ConvertToWrapper(handle)->GetLastError();\n}'

    return output
