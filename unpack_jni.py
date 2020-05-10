from __future__ import print_function
import os
import re
import collections


class CciFunctionDef:

    def __init__(self, rettype, func_name, args):
        self.rettype = rettype
        self.func_name = func_name
        self.args = args

    def sanitized_args(self):
        output = []
        for arg in self.args:
            t, n = arg
            if "*" in n:
                t += "*"
                n = n[1:]


#             if self.func_name == 'c_PigeonIMU_Get6dQuaternion':
#                 print(t)
            if "[" in n:
                find_stuff = re.findall(r'(.*)\[([0-9]+)\]', n)
                if find_stuff:
                    arry_name, arr_length = re.findall(r'(.*)\[([0-9]+)\]',
                                                       n)[0]
                    print("Got an array", n, arr_length)
                    for i in range(int(arr_length)):
                        output.append((t + "*", arry_name + "[%s]" % i))
                else:
                    output.append((t, n))
            else:
                output.append((t, n))
        return output


class JniFunctionDef:

    def __init__(self, comment, rettype, func_name, args):
        self.comment = comment
        self.rettype = rettype
        self.func_name = func_name
        self.args = args


def parse_cci_args(function_text):
    #     print(function_text)
    rettype, func_name, arg_str = re.findall(r"CCIEXPORT (.*?) (.*)\((.*)\)",
                                             function_text)[0]

    #     print("Parsing '" + function_text + "'")
    arg_parts = arg_str.split(",")
    args = []
    for arg_pair in arg_parts:
        arg_pair = arg_pair.strip()
        if arg_pair == "c_SparkMax_handle":
            t, n = "c_SparkMax_handle", "handle"
        elif len(arg_pair) == 0:
            t, n = None, None
        else:
            arg_parts = arg_pair.strip().split(" ")
            #             print(arg_parts, len(arg_parts))
            if len(arg_parts) > 2:
                t = " ".join(arg_parts[:-1])
                n = arg_parts[-1]
            else:
                t, n = arg_pair.strip().split(" ")
        if t is not None:
            args.append((t, n))

    return CciFunctionDef(rettype, func_name, args)


def parse_jni_args(function_text, comments):
    ret_code, func_name, arg_str = re.findall(
        r"JNIEXPORT (.*?) JNICALL (.*)\((.*)\)", function_text)[0]
    ret_code, func_name, arg_str = ret_code.strip(), func_name.strip(
    ), arg_str.strip()

    arg_parts = arg_str.split(",")
    args = []
    for arg_pair in arg_parts:
        arg_pair = arg_pair.strip()
        args.append(arg_pair)

    return JniFunctionDef(comments, ret_code, func_name, args)


def parse_cci_file(cci_file):
    output = collections.OrderedDict()
    with open(cci_file, 'r') as f:
        lines_iter = iter(f.readlines())
        try:
            while True:
                line = next(lines_iter).strip()
                if not (line.startswith("CCIEXPORT")):
                    continue
                function_text = line
                while not line.endswith(");"):
                    line = next(lines_iter).strip()
                    function_text += " " + line
                cci_def = parse_cci_args(function_text)
                output[cci_def.func_name] = cci_def
        except StopIteration:
            pass

    return output


def parse_jni_file(jni_file):
    output = []
    with open(jni_file, 'r') as f:
        lines_iter = iter(f.readlines())
        try:
            while True:
                line = next(lines_iter).strip()
                comment_text = ""
                if line == "/*":
                    comment_text += line + "\n"
                    comment_text += next(lines_iter)
                    comment_text += next(lines_iter)
                    comment_text += next(lines_iter)
                    comment_text += next(lines_iter)

                    #                     function_text = "XXXX"
                    function_text = next(lines_iter).strip()
                    function_text += next(lines_iter).strip()
                    jni_def = parse_jni_args(function_text, comment_text)

                    output.append(jni_def)

        except StopIteration:
            pass
        print("After")

    return output


def dump_jni_set_function(jni_def, cci_function):

    if len(jni_def.args) - 1 != len(cci_function.args):
        print("UH OH", jni_def.func_name, len(jni_def.args),
              len(cci_function.args))
        return dump_unknown_function(jni_def)


#     if jni_def.func_name != "Java_com_revrobotics_jni_CANSparkMaxJNI_c_1SparkMax_1GetAppliedOutput":
#         return dump_unknown_function(jni_def)

#     print("Good")

    output_type = None
    for arg_type, arg_name in cci_function.args:
        if "*" in arg_type:
            output_type = arg_type, arg_name

    arg_text = ""
    arg_text += "  ("
    arg_text += ", ".join(str(argg) for argg in jni_def.args[:2])

    ret_def = cci_function.rettype
    func_args = ""

    for i in range(len(cci_function.args)):
        if cci_function.args[i][0] == "c_SparkMax_handle":
            func_args += "ConvertToMotorControllerWrapper(handle), "
        else:
            cci_type = ""

            if cci_function.args[i] == output_type:
                cci_type = "&"
            elif cci_function.args[i][0] not in [
                    "int", "float", "uint8_t", "uint16_t", "uint32_t"
            ]:
                print("Not listed '%s'" % cci_function.args[i][0])
                cci_type = "(%s) " % cci_function.args[i][0]
            func_args += "%s%s, " % (cci_type, cci_function.args[i][1])

        if cci_function.args[i] == output_type:
            print("GOT IT", cci_function.args[i])
        else:
            print("NOPE IT", cci_function.args[i])

            arg_text += ", " + jni_def.args[i +
                                            2] + " " + cci_function.args[i][1]

    func_args = func_args[:-2]
    #         print(arg)

    arg_text += ")"

    output = arg_text
    output += "\n{\n"
    output += '   %s %s;\n' % (output_type[0].replace("*", ""), output_type[1])
    output += '   %s(%s);\n' % (cci_function.func_name, func_args)
    if jni_def.rettype.strip() != "void":
        output += '   return (%s) %s;\n' % (jni_def.rettype, output_type[1])
    output += "}\n"

    return output


def dump_known_function(jni_def, cci_function):

    if len(jni_def.args) - 2 != len(cci_function.args):
        is_setter = False
        for arg_type, _ in cci_function.args:
            #             print(arg_type)
            if "*" in arg_type:
                is_setter = True
                break

        if is_setter:
            #             print("IS SETTER")
            return dump_jni_set_function(jni_def, cci_function)
        else:
            #             print("UH OH", jni_def.func_name, len(jni_def.args), len(cci_function.args))
            return dump_unknown_function(jni_def)


#     print("Good")

    arg_text = ""
    arg_text += "  ("
    arg_text += ", ".join(str(argg) for argg in jni_def.args[:2])

    ret_def = cci_function.rettype
    func_args = ""

    for i in range(len(jni_def.args[2:])):
        if cci_function.args[i][0] == "c_SparkMax_handle":
            func_args += "ConvertToMotorControllerWrapper(handle), "
        else:
            cci_type = ""
            if cci_function.args[i][0] not in [
                    "int", "float", "uint8_t", "uint16_t", "uint32_t"
            ]:
                #                 print("Not listed '%s'" % cci_function.args[i][0])
                cci_type = "(%s) " % cci_function.args[i][0]
            func_args += "%s%s, " % (cci_type, cci_function.args[i][1])

        arg_text += ", " + jni_def.args[i + 2] + " " + cci_function.args[i][1]

    func_args = func_args[:-2]
    #         print(arg)

    arg_text += ")"

    output = arg_text
    output += "\n{\n"
    output += '   %s output = %s(%s);\n' % (ret_def, cci_function.func_name,
                                            func_args)
    if jni_def.rettype.strip() != "void":
        output += '   return (%s) output;\n' % jni_def.rettype
    output += "}\n"

    return output


def dump_unknown_function(jni_def):

    output = "  ("
    output += ", ".join(str(argg) for argg in jni_def.args)
    output += ")"
    output += "\n{\n"
    output += '   LOG_UNSUPPORTED_CAN_FUNC("");\n'
    if jni_def.rettype.strip() != "void":
        output += '   return 0;\n'
    output += "}\n"

    return output


def guess_jni_function(package_name, jni_def, cci_functions):

    stripped_func_name = prefix_name + jni_def.func_name[
        len(package_name + "JNI_c_1_1SparkMax_1"):]
    #     print("XXXXXXXXXXXXX", stripped_func_name)

    if stripped_func_name in cci_functions:
        output = dump_known_function(jni_def, cci_functions[stripped_func_name])
    else:
        output = dump_unknown_function(jni_def)

    return output


def dump_updated_jni(jni_output_file, package_name, cci_functions,
                     jni_functions):
    with open(jni_output_file, 'w') as f:
        f.write("""
#include <jni.h>

#include <cassert>

#include "com_revrobotics_jni_CANSparkMaxJNI.h"
#include "RevSimMocks/RevMockUtilities.h"
#include "rev/CANSParkMaxDriver.h"
#include "RevSimMocks/RevDeviceWrapper.h"

extern "C" {


c_SparkMax_handle ConvertToMotorControllerWrapper(jlong aHandle)
{
    return (c_SparkMax_handle) reinterpret_cast<SnobotSim::RevSimulator*>(aHandle);
}


""")

        for jni_def in jni_functions:
            f.write(jni_def.comment)
            f.write("JNIEXPORT %s JNICALL %s\n" %
                    (jni_def.rettype, jni_def.func_name))
            f.write(guess_jni_function(package_name, jni_def, cci_functions))

        f.write("\n}\n")


def dump_updated_cci(cci_output_file, cci_header, func_prefix_name, class_type,
                     conversion_func, cci_functions):

    prefix_name = func_prefix_name
    with open(cci_output_file, 'w') as f:
        f.write(cci_header)
        #         tmemp_funcs = []
        #         tmemp_funcs.append("c_SparkMax_SetFollow")
        #         tmemp_funcs.append("c_SparkMax_SetpointCommand")
        #         tmemp_funcs.append("c_SparkMax_SetFollow")
        #
        #         other_temp = []
        #         other_temp.append("c_SparkMax_GetAppliedOutput")

        for cci_def in cci_functions.values():
            f.write("%s %s(" % (cci_def.rettype, cci_def.func_name))
            f.write(", ".join("%s %s" % x for x in cci_def.args))
            f.write(")")
            call_name = cci_def.func_name[len(prefix_name):]

            if cci_def.func_name == "c_MotController_GetInverted":
                print("HEllo")

            is_getter = True
            for arg_type, _ in cci_def.sanitized_args():
                if "*" in arg_type and arg_type != "void*":
                    print("Not a getter because of ", arg_type)
                    is_getter = False
                    break

            if is_getter:

                f.write("\n{\n")
                f.write(
                    '    %s* wrapper = %s(handle);\n    wrapper->Send("%s"' %
                    (class_type, conversion_func, call_name))
                for _, arg_name in cci_def.sanitized_args():
                    if arg_name != "handle" and arg_name != "*handle" and arg_name != "timeoutMs":
                        f.write(", %s" % arg_name)
                f.write(");\n")
                if cci_def.rettype != "void":
                    f.write("    return (ctre::phoenix::ErrorCode)0;\n")
                f.write("}\n\n")
            elif True:  # cci_def.func_name in other_temp:
                f.write("\n{\n")
                rcv_helper = ""
                for arg_type, arg_name in cci_def.sanitized_args():
                    if arg_name != "handle" and arg_name != "timeoutMs":
                        if "*" in arg_type:
                            rcv_helper += ("sizeof(*%s) + " % arg_name)
                        else:
                            rcv_helper += ("sizeof(%s) + " % arg_name)
                f.write('    RECEIVE_HELPER("%s", %s);\n' %
                        (call_name, rcv_helper[:-3]))

                for arg_type, arg_name in cci_def.sanitized_args():
                    if arg_name != "handle":
                        if "*" in arg_type:
                            f.write(
                                '    PoplateReceiveResults(buffer, %s, buffer_pos);\n'
                                % (arg_name))
                        else:
                            f.write(
                                '    PoplateReceiveResults(buffer, &%s, buffer_pos);\n'
                                % (arg_name))


#                 f.write(rcv_helper + ");\n")
                f.write("    return (ctre::phoenix::ErrorCode)0;\n")
                f.write("}\n\n")
            else:
                f.write(";\n")


def main():

    motorcontrl_cci_header = """
#include "ctre/phoenix/cci/MotController_CCI.h"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "CtreSimMocks/CtreMotorControllerWrapper.h"
#include "CtreSimMocks/MockHooks.h"

typedef SnobotSim::CtreMotorControllerWrapper MotorControllerWrapper;

#define RECEIVE_HELPER(paramName, size)                                        \\
    MotorControllerWrapper* wrapper = ConvertToMotorControllerWrapper(handle); \\
    uint8_t buffer[size]; /* NOLINT */                                         \\
    std::memset(&buffer[0], 0, size);                                          \\
    wrapper->Receive(paramName, buffer, size);                                 \\
    uint32_t buffer_pos = 0;


MotorControllerWrapper* ConvertToMotorControllerWrapper(void* param)
{
    return reinterpret_cast<MotorControllerWrapper*>(param);
}

extern "C"{

"""

    pheonix_cci_header = """
#include "ctre/phoenix/cci/PigeonIMU_CCI.h"

#include <cstring>
#include <vector>

#include "CtreSimMocks/CtrePigeonImuWrapper.h"
#include "CtreSimMocks/MockHooks.h"

typedef SnobotSim::CtrePigeonImuWrapper PigeonImuSimulatorWrapper;

#define RECEIVE_HELPER(paramName, size)                                  \\
    PigeonImuSimulatorWrapper* wrapper = ConvertToPigeonWrapper(handle); \\
    uint8_t buffer[size]; /* NOLINT */                                   \\
    std::memset(&buffer[0], 0, size);                                    \\
    wrapper->Receive(paramName, buffer, size);                           \\
    uint32_t offset = 0;


PigeonImuSimulatorWrapper* ConvertToPigeonWrapper(void* param)
{
    return reinterpret_cast<PigeonImuSimulatorWrapper*>(param);
}

extern "C"{
"""
    canifier_cci_header = """#include "ctre/phoenix/cci/CANifier_CCI.h"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "CtreSimMocks/CtreCanifierWrapper.h"
#include "CtreSimMocks/MockHooks.h"

#define RECEIVE_HELPER(paramName, size)                                         \\
    SnobotSim::CtreCanifierWrapper* wrapper = ConvertToCanifierWrapper(handle); \\
    uint8_t buffer[size]; /* NOLINT */                                          \\
    std::memset(&buffer[0], 0, size);                                           \\
    wrapper->Receive(paramName, buffer, size);                                  \\
    uint32_t buffer_pos = 0;

SnobotSim::CtreCanifierWrapper* ConvertToCanifierWrapper(void* param)
{
    return reinterpret_cast<SnobotSim::CtreCanifierWrapper*>(param);
}

extern "C"{

"""

    print("Hello")
    #     cci_functions = parse_cci_file(os.path.join(base_dir, 'ctre_source/cci/native/include/ctre/phoenix/cci/MotController_CCI.h'))
    #     jni_functions = parse_jni_file(os.path.join(base_dir, 'ctre_source/cci/native/include/ctre/phoenix/jni/com_ctre_phoenix_motorcontrol_can_MotControllerJNI.h'))

    #     run_conversion("MotController_CCI.h", "com_ctre_phoenix_motorcontrol_can_MotControllerJNI.h", motorcontrl_cci_header, "c_MotController_", "MotorControllerWrapper", "ConvertToMotorControllerWrapper")
    #     run_conversion("PigeonIMU_CCI.h", "com_ctre_phoenix_sensors_PigeonImuJNI.h", pheonix_cci_header, "c_PigeonIMU_", "PigeonImuSimulatorWrapper", "ConvertToPigeonWrapper")
    run_conversion("CANifier_CCI.h", "com_ctre_phoenix_CANifierJNI.h",
                   canifier_cci_header, "c_CANifier_",
                   "SnobotSim::CtreCanifierWrapper", "ConvertToCanifierWrapper")


def run_conversion(cci_header, jni_header, cci_initial_dump, cci_prefix,
                   cci_wrapper_type, cci_conversion_func):

    base_dir = r'C:\Users\PJ\Documents\GitHub\SnobotSim\CtreSimulator'
    output_cci_file = os.path.join(base_dir,
                                   'com_revrobotics_jni_CANSparkMaxJNI.h')
    output_jni_file = os.path.join(base_dir,
                                   'com_revrobotics_jni_CANSparkMaxJNI.h')

    base_dir = r'C:\Users\PJ\Documents\GitHub\SnobotSim\CtreSimulator'
    cci_functions = parse_cci_file(
        os.path.join(
            base_dir,
            'ctre_source/cci/native/include/ctre/phoenix/cci/%s' % cci_header))
    jni_functions = parse_jni_file(
        os.path.join(
            base_dir,
            'ctre_source/cci/native/include/ctre/phoenix/jni/%s' % jni_header))
    #     dump_updated_jni(output_jni_file, "com_revrobotics_jni_CANSparkMaxJNI", cci_functions, jni_functions)
    dump_updated_cci(output_cci_file, cci_initial_dump, cci_prefix,
                     cci_wrapper_type, cci_conversion_func, cci_functions)


if __name__ == "__main__":
    main()
