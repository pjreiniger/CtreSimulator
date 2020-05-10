def cci_sanitize_rettype(return_type):
    if return_type.startswith("CCIEXPORT"):
        return_type = return_type[len("CCIEXPORT "):]

    return return_type


def cci_sanitize_func_name(cci_prefix, func_name):
    if func_name.startswith(cci_prefix):
        func_name = func_name[len(cci_prefix):]

    return func_name


def cci_seperate_arguments(func, getter_overrides):
    input_arguments = []
    output_arguments = []

    if func['name'] in getter_overrides:
        output_arguments = getter_overrides[func['name']]
#         print("{} is in overrides ({})".format(func['name'], output_arguments))
    else:
        for arg in func['parameters']:
            if "*" in arg["type"] and arg["type"] != "void *":
                output_arguments.append(arg)
            else:
                input_arguments.append(arg)

    return input_arguments, output_arguments


def cci_get_output_arguments(func, getter_overrides):
    _, output_arguments = cci_seperate_arguments(func, getter_overrides)

    return [arg['name'] for arg in output_arguments]
