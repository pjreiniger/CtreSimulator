import CppHeaderParser
from jinja2 import Environment, Template, PackageLoader, select_autoescape
import os
import copy
from .cci_helpers import cci_sanitize_rettype, cci_sanitize_func_name, cci_get_output_arguments


class CciGenerator():

    def __init__(self, definition):
        self.stripped_cci_class_name = definition.get_stripped_cci_class_name()
        self.wrapper_class_name = definition.get_wrapper_class_name()
        self.cci_prefix = definition.cci_prefix
        self.cci_class_name = definition.cci_class_name
        self.getter_overrides = definition.cci_getter_overrides
        self.overriden_function_bodies = definition.cci_overriden_function_bodies
        self.wrapper_type = definition.cci_wrapper_type
        self.error_ret_code = definition.error_ret_code

    def __dump_func_contents(self, func):
        is_getter = cci_get_output_arguments(func, self.getter_overrides)
        template = """{
    ConvertToWrapper(handle)->{{ callback_name }}({% for arg in args if arg.name != "handle" and arg.name != "timeoutMs" and arg.name != "timoutMs" %}{{arg.name}}{{ ", " if not loop.last }}{% endfor %});{% if return_type == error_code %}
    return ({{error_code}})0;{% endif %}
}"""

        return Template(template).render(
            callback_name=cci_sanitize_func_name(self.cci_prefix, func['name']),
            return_type=cci_sanitize_rettype(func['rtnType']),
            stripped_cci_class_name=self.stripped_cci_class_name,
            wrapper_class_name=self.wrapper_class_name,
            wrapper_type=self.wrapper_type,
            error_code=self.error_ret_code,
            args=func['parameters'])

    def generate(self, cci_header_file, output_file, template_file):

        with open(template_file) as f:
            template_text = f.read()

        parsed_header = CppHeaderParser.CppHeader(cci_header_file)

        output = ""

        jni_functions = []
        for func in parsed_header.functions:
            output += cci_sanitize_rettype(func['rtnType']) + " " + func["name"] + "("

            def get_arg_str(arg):
                if arg["type"].endswith("*"):
                    arg_str = arg["type"][:-2] + "* "
                else:
                    arg_str = arg["type"] + " "

                return arg_str

            output += ", ".join(get_arg_str(arg) + arg["name"] + ("[%s]" % arg['array_size'] if "array_size" in arg else "") for arg in func["parameters"])

            output += ")\n"
            #             print(func['name'], self.overriden_function_bodies)
            if str(func['name']) in self.overriden_function_bodies:
                print("Dumping override for ", str(func['name']))
                output += Template(self.overriden_function_bodies[str(
                    func['name'])]).render(wrapper_type=self.wrapper_type)
            else:
                output += self.__dump_func_contents(func)
            output += "\n\n"

        with open(output_file, 'w') as f:
            f.write(
                Template(template_text).render(
                    functions=output,
                    stripped_cci_class_name=self.stripped_cci_class_name,
                    wrapper_class_name=self.wrapper_class_name,
                    cci_class_name=self.cci_class_name))
