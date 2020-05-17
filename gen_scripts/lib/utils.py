


def realign_pointer_position(arg):
    if arg["type"].endswith("*"):
        arg_str = arg["type"][:-2] + "* "
    else:
        arg_str = arg["type"] + " "
    
    return arg_str