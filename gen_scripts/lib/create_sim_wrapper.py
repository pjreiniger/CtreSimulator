import CppHeaderParser
from jinja2 import Environment, Template, PackageLoader, select_autoescape
import os
import copy
from .cci_helpers import cci_sanitize_rettype, cci_sanitize_func_name, cci_get_output_arguments, cci_seperate_arguments


WRAPPER_SETTER_TEMPLATE = """{
    RECEIVE_HELPER("{{ callback_name }}", {% for arg in args if arg.name != "handle" and arg.name != "timeoutMs" %}sizeof({{ "*" if "*" in arg.type }}{{arg.name}}){{ " + " if not loop.last }}{% endfor %});{% for arg in args if arg.name != "handle" and arg.name != "timeoutMs" %}
    PoplateReceiveResults(buffer, {{ "&" if "*" not in arg.type }}{{arg.name}}, buffer_pos);{% endfor %}
}"""

WRAPPER_GETTER_TEMPLATE = """{
    Send("{{ callback_name }}"{% for arg in args if arg.name != "handle" and arg.name != "timeoutMs" %}, {{arg.name}}{% endfor %});
}"""

class SimWrapperGenerator():

    def __init__(self, definition):
        self.stripped_cci_class_name = definition.get_stripped_cci_class_name()
        self.wrapper_class_name = definition.get_wrapper_class_name()
        self.cci_prefix = definition.cci_prefix
        self.cci_class_name = definition.cci_class_name
        self.getter_overrides = definition.cci_getter_overrides
        self.overriden_function_bodies = definition.cci_overriden_function_bodies
        self.has_device_id = self.wrapper_class_name != "CtreBuffTrajPointStreamWrapper"
        #         self.setter_template = definition.cci_setter_template
        #         self.getter_template = definition.cci_getter_template

        self.setter_template = WRAPPER_GETTER_TEMPLATE
        self.getter_template = WRAPPER_SETTER_TEMPLATE

        self.setter_keys = ['Config', 'Set']
        self.getter_keys = ['Get']


    def __explode_arrays(self, args):
        new_args = []

        for arg in args:
            if "array_size" in arg:
                for i in range(int(arg['array_size'])):
                    array_arg = {}
                    array_arg['name'] = arg['name'] + '[%s]' % i
                    array_arg['type'] = arg['type']
                    new_args.append(array_arg)
            else:
                new_args.append(arg)

        return new_args

    def __dump_func_contents(self, func):
        getter_arguments = cci_get_output_arguments(func, self.getter_overrides)
        template = self.getter_template if getter_arguments else self.setter_template

        return Template(template).render(
            callback_name=cci_sanitize_func_name(self.cci_prefix, func['name']),
            return_type=cci_sanitize_rettype(func['rtnType']),
            stripped_cci_class_name=self.stripped_cci_class_name,
            wrapper_class_name=self.wrapper_class_name,
            args=self.__explode_arrays(func['parameters']))

    def __gen_function_header(self, func, include_class_name):
        output = ""
        output += "void "
        if include_class_name:
            output += "SnobotSim::%s::" % self.wrapper_class_name
        output += func['name'][len(self.cci_prefix):] + "("
        output += ", ".join(
            arg["type"] + ("" if arg["type"].endswith("*") else " ") +
            arg["name"] +
            ("[%s]" % arg['array_size'] if "array_size" in arg else "")
            for arg in func["parameters"]
            if arg['name'] != "handle" and arg['name'] != "timeoutMs" and arg['name'] != "timoutMs")

        output += ")"
        return output


    def __get_constructor_body(self, variables_lookup):

        template = """
    Send("Create");
"""

        return Template(template).render()

    def generate(self, cci_header_file, cpp_output_file, header_output_file,
                 cpp_template_file, header_template_file):

        parsed_header = CppHeaderParser.CppHeader(cci_header_file)

        cpp_output = ""
        header_output = ""
        member_variables = []

#         variable_lookup = self.__load_variable_mapping(parsed_header.functions)

        for func in parsed_header.functions:
            #             print(func['name'], self.cci_prefix + "Create")
            if str(func['name']) in self.overriden_function_bodies:
                continue
            elif self.cci_prefix + "Create" in str(func['name']):
                continue
            elif self.cci_prefix + "Destroy" in str(func['name']):
                continue
            else:
                header_output += "    " + self.__gen_function_header(
                    func, False) + ";\n"
                cpp_output += self.__gen_function_header(func, True) + "\n"
                cpp_output += self.__dump_func_contents(func)

            cpp_output += "\n\n"
            
#         print(cpp_template_file)
#         print(header_template_file)
#         print(cpp_output_file)
#         print(header_output_file)
#         raise

        with open(cpp_template_file) as f:
            cpp_template_text = f.read()

        with open(header_template_file) as f:
            header_template_text = f.read()

        with open(cpp_output_file, 'w') as f:
            f.write(
                Template(cpp_template_text).render(
                    cpp_functions=cpp_output,
                    wrapper_class_name=self.wrapper_class_name,
                    stripped_cci_class_name=self.stripped_cci_class_name,
                    has_device_id=self.has_device_id,
                ))

        with open(header_output_file, 'w') as f:
            f.write(
                Template(header_template_text).render(
                    header_functions=header_output,
                    wrapper_class_name=self.wrapper_class_name,
                    has_device_id=self.has_device_id,
                    cci_class_name=self.cci_class_name,
                ))
