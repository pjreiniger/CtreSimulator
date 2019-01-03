
import re
import collections


def check(source_file, parent_deff_pattern , child_def_pattern, max_diff=5, child_group_index=1):

    print "Checking file ", source_file

    parent_deff_pattern = re.compile(parent_deff_pattern)
    child_def_pattern = re.compile(child_def_pattern)

    parent_functions = collections.OrderedDict()
    child_functions = collections.OrderedDict()
    with open(source_file, 'r') as f:
        for i, line in enumerate(f.readlines()):
            parent_results = re.match(parent_deff_pattern, line)
            if parent_results:
                parent_functions[parent_results.group(1)] = i

            child_results = re.match(child_def_pattern, line)
            if child_results:
                child_functions[child_results.group(child_group_index)] = i

    for func in parent_functions:
        if func not in child_functions:
            print "  Child function %s not in list" % func
            continue

        parent_line = parent_functions[func]
        child_line = child_functions[func]

        line_diff = child_line - parent_line
        if  line_diff > max_diff or line_diff < 0:
            print "  Function %s is %s lines away" % (func, line_diff)


def search_files(header_file, source_file, header_pattern, source_pattern):


    header_functions = collections.OrderedDict()
    with open(header_file, 'r') as f:
        lines = f.readlines()
        lines = "".join(lines)
        for match in header_pattern.finditer(lines):
            header_functions[match.group(2)] = (match.group(1))

    source_functions = collections.OrderedDict()
    with open(source_file, 'r') as f:
        lines = f.readlines()
        lines = "".join(lines)
        for match in source_pattern.finditer(lines):
            source_functions[match.group(2)] = (match.group(1))

#     print "Headers..."
#     for func_name in header_functions:
#        print "  ", header_functions[func_name], func_name
# 
#     print "Source..."
#     for func_name in source_functions:
#        print "  ", source_functions[func_name], func_name

    source_function_set = set(source_functions.keys())
    header_function_set = set(header_functions.keys())
    source_only = source_function_set.difference(header_function_set)
    header_only = header_function_set.difference(source_function_set)

    success = len(source_only) == 0 and len(header_only) == 0
    if not success:
        print "Error in ", header_file

    if len(source_only) != 0:
        print "  Source Only:"
        print "    " + "\n  ".join(source_only)

    if len(header_only) != 0:
        print "  Header Only:"
        print "    " + "\n  ".join(header_only)


def search_jni():
    jni_header_base = r'ctre_source\cci\native\include\ctre\phoenix\jni'
    jni_source_base = r'src\main\native\cpp\ctre_jni_mocks'

    jni_header_pattern = re.compile(r'JNIEXPORT (.*) JNICALL (.*)\n +\((.*)\);', re.MULTILINE)
    jni_source_pattern = re.compile(r'JNIEXPORT (.*) JNICALL (.*)(\r?\n +)?\(', re.MULTILINE)

    search_files(jni_header_base + '/com_ctre_phoenix_CANifierJNI.h', jni_source_base + '/CANifierJNI.cpp', jni_header_pattern, jni_source_pattern)
    search_files(jni_header_base + '/com_ctre_phoenix_CTRLoggerJNI.h', jni_source_base + '/CTRLoggerJNI.cpp', jni_header_pattern, jni_source_pattern)
    search_files(jni_header_base + '/com_ctre_phoenix_MotorControl_CAN_MotControllerJNI.h', jni_source_base + '/MotControllerJNI.cpp', jni_header_pattern, jni_source_pattern)
    search_files(jni_header_base + '/com_ctre_phoenix_Sensors_PigeonImuJNI.h', jni_source_base + '/PigeonImuJNI.cpp', jni_header_pattern, jni_source_pattern)


def search_cci():
    cci_header_base = r'ctre_source\cci\native\include\ctre\phoenix\CCI'
    cci_source_base = r'src\main\native\cpp\ctre_cci_mocks'

    search_files(cci_header_base + '/CANifier_CCI.h',
                cci_source_base + '/CANifier_CCI.cpp',
                re.compile(r'\s(.*)(c_CANifier_.*)\(', re.MULTILINE),
                re.compile(r'(.*)(c_CANifier_.*)\(', re.MULTILINE))

    search_files(cci_header_base + '/Logger_CCI.h',
                cci_source_base + '/Logger_CCI.cpp',
                re.compile(r'\s(.*)(c_Logger_.*)\(', re.MULTILINE),
                re.compile(r'(.*)(c_Logger_.*)\(', re.MULTILINE))

    search_files(cci_header_base + '/MotController_CCI.h',
                cci_source_base + '/MotController_CCI.cpp',
                re.compile(r'\s(.*)(c_MotController_.*)\(', re.MULTILINE),
                re.compile(r'(.*)(c_MotController_.*)\(', re.MULTILINE))

    search_files(cci_header_base + '/PigeonIMU_CCI.h',
                 cci_source_base + '/PigeonIMU_CCI.cpp',
                 re.compile(r'\s(.*)(c_PigeonIMU_.*)\(', re.MULTILINE),
                 re.compile(r'(.*)(c_PigeonIMU_.*)\(', re.MULTILINE))


def check_jni_sources():
    cci_base = r'src\main\native\cpp\ctre_jni_mocks'

    check(cci_base + '/CANifierJNI.cpp',
          "JNIEXPORT .* JNICALL Java_com_ctre_phoenix_CANifierJNI_JNI_[0-9]?(.*)",
          ".*c_CANifier_(.*)\(.*")

    check(cci_base + '/CTRLoggerJNI.cpp',
          "JNIEXPORT .* JNICALL Java_com_ctre_phoenix_CTRLoggerJNI_JNI_[0-9]?(.*)",
          ".*c_Logger_(.*)\(.*")

    check(cci_base + '/MotControllerJNI.cpp',
          "JNIEXPORT .* JNICALL Java_com_ctre_phoenix_motorcontrol_can_MotControllerJNI_[0-9]?(.*)",
          ".*c_MotController_(.*)\(.*")

    check(cci_base + '/PigeonImuJNI.cpp',
          "JNIEXPORT .* JNICALL Java_com_ctre_phoenix_sensors_PigeonImuJNI_JNI_[0-9]?(.*)",
          ".*c_PigeonIMU_(.*)\(.*")



def check_cci_sources():
    cci_base = r'src\main\native\cpp\ctre_cci_mocks'

    child_pattern = '.*(wrapper->Send|RECEIVE_HELPER)\("(.*)".*\);'
    max_diff = 5

    check(cci_base + '/CANifier_CCI.cpp',
          ".*c_CANifier_(.*)\(.*",  child_pattern, max_diff, child_group_index=2)

#     check(cci_base + '/Logger_CCI.cpp',
#           ".*c_Logger_(.*)\(.*",  child_pattern, max_diff, child_group_index=2)

    check(cci_base + '/MotController_CCI.cpp',
          ".*c_MotController_(.*)\(.*", child_pattern, max_diff, child_group_index=2)

    check(cci_base + '/PigeonIMU_CCI.cpp',
          ".*c_PigeonIMU_(.*)\(.*",  child_pattern, max_diff, child_group_index=2)


def main():
    
    root_path = "."
    
    search_jni()
    search_cci()
#     check_jni_sources()
#     check_cci_sources()

main()
