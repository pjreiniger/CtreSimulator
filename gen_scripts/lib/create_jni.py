import CppHeaderParser
import os
from jinja2 import Environment, Template, PackageLoader, select_autoescape
from .cci_helpers import cci_sanitize_rettype, cci_sanitize_func_name, cci_get_output_arguments

DOCS_TEMPLATE = """
/*
 * Class:     {{ package_name }}
 * Method:    {{ method_name }}
 * Signature: ({{args_abbrv}}){{retrn_abbr}}
 */
{{ return_type }}
{{full_method_name}}
  (JNIEnv*, jclass{{ jni_func_args }})
{
{{ function_body }}
}

"""


class JniGenerator():

    def __init__(self, definition):
        self.jni_package_name = "Java_%s_" % definition.jni_package
        self.jni_prefix = "Java_%s_" % definition.jni_package
        self.jni_package = definition.jni_package
        self.cci_class_name = definition.cci_class_name
        self.cci_wrapper_type = definition.cci_wrapper_type
        self.include_jni_in_method_name = definition.include_jni_in_method_name
        self.doc_package_name = definition.jni_package
        self.full_jni_prefix = "Java_%s" % definition.full_jni_package
        self.cci_prefix = definition.cci_prefix
        self.getter_overrides = definition.cci_getter_overrides
        self.jni_sanitized_name = definition.jni_sanitized_name
        self.error_ret_code = definition.error_ret_code
        self.extra_jni_header = definition.extra_jni_header
        self.overriden_function_bodies = definition.cci_overriden_function_bodies
        
        self.default_value_lookup = {}
        self.default_value_lookup["uint8_t *"] = " = 0"
        self.default_value_lookup["uint16_t *"] = " = 0"
        self.default_value_lookup["uint32_t *"] = " = 0"
        self.default_value_lookup["float *"] = " = 0"
        self.default_value_lookup["bool *"] = " = false"
        self.default_value_lookup["const char *"] = ""
        self.default_value_lookup["int *"] = " = 0"
        self.default_value_lookup["double *"] = " = 0"
        self.default_value_lookup["size_t *"] = " = 0"
        self.default_value_lookup["const double *"] = " = 0"
        self.default_value_lookup["char *"] = " = 0"
        self.default_value_lookup.update(definition.default_value_lookup)

        self.enum_casts = definition.cpp_enum_types


        self.jni_type_to_abbr_lookup = {}
        self.jni_type_to_abbr_lookup['int'] = 'I'
        self.jni_type_to_abbr_lookup['jint'] = 'I'
        self.jni_type_to_abbr_lookup['jlong'] = 'J'
        self.jni_type_to_abbr_lookup['jfloat'] = 'F'
        self.jni_type_to_abbr_lookup['jstring'] = 'java/lang/String;'
        self.jni_type_to_abbr_lookup['jdouble'] = 'D'
        self.jni_type_to_abbr_lookup['jboolean'] = 'Z'
        self.jni_type_to_abbr_lookup['void'] = 'V'
        self.jni_type_to_abbr_lookup['jintArray'] = '[I'
        self.jni_type_to_abbr_lookup['jdoubleArray'] = '[D'
        self.jni_type_to_abbr_lookup['jshortArray'] = '[S'
        self.jni_type_to_abbr_lookup['jbooleanArray'] = '[Z'
        
    def __load_cci_functions(self, header_file):
        parsed_header = CppHeaderParser.CppHeader(header_file)

        jni_functions = {}
        for func in parsed_header.functions:
            jni_functions[func["name"][len(self.cci_prefix):]] = func

        return jni_functions

    def __get_setter_function_body(self, cci_func, jni_func):
        output = ""

        if self.error_ret_code in cci_func['rtnType'] and "void" not in jni_func['rtnType']:
            output += "    return (jint)"
        else:
            output += "    "

        output += cci_func['name'] + "(ConvertToWrapper(handle)"

        for arg in cci_func['parameters']:
            if arg['name'] != 'handle':
                output += ", "
                if arg['type'] in self.enum_casts:
                    output += "(%s) " % arg['type']
                output += arg['name']

        output += ");"

        return output

    def __get_getter_function_body(self, cci_func, output_arguments):
        output = ""

        for arg in cci_func['parameters']:
            if str(arg['name']) in output_arguments:
                output += "    " + arg['type'].replace(" *", "") + " " + arg['name'] + self.default_value_lookup[str(arg['type'])] + ";\n"

        output += "    "
        output += cci_func['name'] + "(ConvertToWrapper(handle)"

        for arg in cci_func['parameters']:
            if arg['name'] != 'handle':
                output += ", "
                if arg['type'] in self.enum_casts:
                    output += "(%s) " % arg['type']
                if "*" in arg['type'] and arg['type'] != "void *":
                    output += "&"

                output += arg['name']

        output += ");\n"

        for arg in cci_func['parameters']:
            if "*" in arg['type'] and arg['type'] != "void *":
                output += "    return " + arg['name'] + ";"
                break

        return output

    def __sanitize_jni_func_name(self, func):
        sanitized_func_name = func["name"][len(self.full_jni_prefix):]

        for sanitizer in self.jni_sanitized_name:
            sanitized_func_name = sanitized_func_name.replace(
                sanitizer[0], sanitizer[1])

        return sanitized_func_name

    def generate(self, jni_header_file, cci_header_file, template_file, output_file):

        output = ""

        cci_functions = self.__load_cci_functions(cci_header_file)
        print(cci_functions.keys())

        parsed_header = CppHeaderParser.CppHeader(jni_header_file)

        jni_functions = []
        for func in parsed_header.functions:
            sanitized_func_name = self.__sanitize_jni_func_name(func)
            #         print(func.keys())
            arg_abbrv = "".join(self.jni_type_to_abbr_lookup[arg['type']] for arg in func['parameters'] if arg['type'] != "JNIEnv *" and arg['type'] != "jclass")

            jni_args = [arg['type'] for arg in func['parameters'] if arg['type'] != "JNIEnv *" and arg['type'] != "jclass"]
            cci_func = cci_functions.get(sanitized_func_name, None)
            cci_args = cci_func['parameters'] if cci_func else []
            if not cci_func:
                print("Bouncing", sanitized_func_name)
                function_body = '    LOG_UNSUPPORTED_CAN_FUNC("");\n    return 0;'
                jni_func_args = jni_args
            else:
                output_arguments = cci_get_output_arguments(cci_func, self.getter_overrides)

                if str(func['name']) in self.overriden_function_bodies:
                    print("Dumping override for ", str(func['name']))
                    print(self.overriden_function_bodies[str(func['name'])])
                    function_body = Template(self.overriden_function_bodies[str(func['name'])]).render()
                elif output_arguments:
                    function_body = self.__get_getter_function_body(cci_func, output_arguments)
                else:
                    function_body = self.__get_setter_function_body(cci_func, func)

                jni_func_args = []
                jni_arg_ctr = 0

                for cci_ctr, cci_arg in enumerate(cci_args):
                    if str(cci_arg['name']) not in output_arguments:
                        try:
                            jni_func_args.append("%s %s" % (jni_args[jni_arg_ctr], cci_arg['name']))
                            jni_arg_ctr += 1
                        except:
                            print("UH OH")
                            pass

            output += Template(DOCS_TEMPLATE).render(
                package_name=self.doc_package_name,
                method_name=sanitized_func_name,
                full_method_name=func['name'],
                retrn_abbr=self.jni_type_to_abbr_lookup[func['rtnType'].split()[1]],
                args_abbrv=arg_abbrv,
                return_type=func['rtnType'],
                cci_args=cci_args,
                jni_args=jni_args,
                function_body=function_body,
                include_jni_in_method_name=self.include_jni_in_method_name,
                jni_func_args=", " + ", ".join(jni_func_args))

        with open(template_file) as f:
            template_text = f.read()

        print("Dumping to ", output_file)
        with open(output_file, 'w') as f:
            f.write(Template(template_text).render(functions=output,
                                                   extra_jni_header=self.extra_jni_header,
                jni_package=self.jni_package,
                cci_wrapper_type=self.cci_wrapper_type,
                cci_class_name=self.cci_class_name))
