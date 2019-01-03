import subprocess
import os
import re



def create_tests(jar_path):
    output_dir = "build/tempCreateTests"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    os.chdir(output_dir)
        
        
    unzip_args = []
    unzip_args.append(r'C:\Program Files\Java\jdk1.8.0_191\bin\jar')
    unzip_args.append("xf")
    unzip_args.append(jar_path)
    unzip_args.append("com/ctre/phoenix/motorcontrol/can/TalonSRX.class")
    unzip_args.append("com/ctre/phoenix/motorcontrol/can/BaseMotorController.class")
    unzip_args.append("com/ctre/phoenix/sensors/PigeonIMU.class")
    unzip_args.append("com/ctre/phoenix/CANifier.class")
     
    print " ".join(unzip_args)
    print subprocess.call(unzip_args)

    
    run_javap("talon", r'com/ctre/phoenix/motorcontrol/can/TalonSRX.class')
    run_javap("talon", r'com/ctre/phoenix/motorcontrol/can/BaseMotorController.class')
    run_javap("imu", r'com/ctre/phoenix/sensors/PigeonIMU.class')
    run_javap("canifier", r'com/ctre/phoenix/CANifier.class')
    
    
def prepare_variable_name(in_name):
    
    outname = in_name.replace("LED", "led")
    outname = outname.replace("CAN", "can")
    
    lower_first_char = lambda s: s[:1].lower() + s[1:] if s else ''
    outname = lower_first_char(outname)
    
    return outname
    
    

def run_javap(objName, class_file):
    
    print "File %s ---------------------" % class_file

    javap_args = []
    javap_args.append(r'C:\Program Files\Java\jdk1.8.0_191\bin\javap')
    javap_args.append(class_file)
    p = subprocess.Popen(javap_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    stdout, _ = p.communicate()
    
    tests = ""
    
    for line in stdout.split("\n"):
        matches = re.findall("  public .* (.*)\((.*)\)", line)
        if len(matches) != 0:
            matches = matches[0]
            func = matches[0]
            args = matches[1].split(",")
            arg_replacement = []
            
            enumerations = []
            
            if type(args) == list:
                for a in args:
                    a = a.strip()
                    if a == "int" or a == "double":
                        arg_replacement.append("0")
                    elif a == "boolean":
                        arg_replacement.append("false")
                    elif a != "":
                        stripped_name = a[a.rfind(".") + 1:]
                        if "$" in stripped_name:
                            stripped_name = stripped_name[stripped_name.rfind("$") + 1:]
                        
                        enumerations.append(stripped_name)
                        arg_replacement.append(prepare_variable_name(stripped_name))
                        
            if len(enumerations) == 0:
                tests += "    %s.%s(%s);\n" % (objName, func, ", ".join(arg_replacement))
            else:
                tests += "    for({0} {1} : {0}.values())".format(enumerations[0], prepare_variable_name(enumerations[0])) + "\n    {\n"
                tests += "        %s.%s(%s);\n" % (objName, func, ", ".join(arg_replacement))
                tests += "    }\n"
                
            
    print tests
    
#     print stdout


create_tests(r'C:\Users\PJ\.gradle\caches\modules-2\files-2.1\com.ctre.phoenix\api-java\5.9.2\64e79e598e81bcb62fd084bfb70c8abffad1524e/api-java-5.9.2.jar')