import subprocess
import os
import re
import collections
import shutil



def create_tests(jar_path):
    print(r'C:\Users\PJ\.gradle\caches\modules-2\files-2.1\com.ctre.phoenix\wpiapi-java\5.17.2')
    print(jar_path)
    output_dir = "build/tempCreateTests"
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
        
    os.makedirs(output_dir)
    os.chdir(output_dir)
        
    classes = collections.defaultdict(list)
    classes["talon"].append(("com/ctre/phoenix/motorcontrol/can/TalonSRX.class", TALON_TEMPLATE_HEADER))
    classes["talon"].append(("com/ctre/phoenix/motorcontrol/can/BaseMotorController.class", BASE_MOTOR_CONTROLLER_TEMPLATE_HEADER))
    classes["imu"].append(("com/ctre/phoenix/sensors/PigeonIMU.class", PIGEON_TEMPLATE_HEADER))
    classes["canifier"].append(("com/ctre/phoenix/CANifier.class", CANNIFIER_TEMPLATE_HEADER))
     
        
    unzip_args = []
    unzip_args.append(JAVA_PATH + r'\bin\jar')
    unzip_args.append("xf")
    unzip_args.append(jar_path)

    for class_list in classes.values():
        for clazz, _ in class_list:
            unzip_args.append(clazz)
    
    print(subprocess.call(unzip_args))

    known_classes = ["IMotorController", "SlotConfiguration", "FilterConfiguration", "TrajectoryPoint", "BufferedTrajectoryPointStream", "MotionProfileStatus", "Faults", "StickyFaults",
                     "PinValues", "PWMChannel", "CANifierFaults", "CANifierStickyFaults", "CANifierConfiguration", 
                     "PigeonIMU_Faults", "PigeonIMU_StickyFaults", "PigeonIMUConfiguration", "GeneralStatus"]
    
    

    for key in classes:
        for clazz, header_template in classes[key]:
            run_javap(key, clazz, known_classes, header_template)
    
    
def prepare_variable_name(in_name):
    
    outname = in_name.replace("LED", "led")
    outname = outname.replace("CAN", "can")
    
    lower_first_char = lambda s: s[:1].lower() + s[1:] if s else ''
    outname = lower_first_char(outname)
    
    return outname
    
    

def run_javap(objName, class_file, known_classes, header_template):
    
    print("File %s ---------------------" % class_file)
    output_file = "Test%sFunctions" % class_file.split('/')[-1][:-6]
    output_file = os.path.join("%s/src/test/java/com/snobot/simulator/ctre/%s.java" % (os.path.dirname(os.path.realpath(__file__)), output_file))
    print(class_file)
    print(output_file)

    javap_args = []
    javap_args.append(JAVA_PATH + r'\bin\javap')
    javap_args.append(class_file)
    p = subprocess.Popen(javap_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    stdout, stderr = p.communicate()
    stdout = stdout.decode("utf-8")
    print(stderr)
    print("Returned with %s" % p.returncode)
    
    tests = ""
    
    for line in stdout.split("\n"):
#         print(line)
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
                        
                        if stripped_name in known_classes:
                            arg_replacement.append("new %s()" % stripped_name)
                        else:
                            enumerations.append(stripped_name)
                            arg_replacement.append(prepare_variable_name(stripped_name))
                        
            if len(enumerations) == 0:
                tests += "        %s.%s(%s);\n" % (objName, func, ", ".join(arg_replacement))
            else:
                indent = "        "
                tests += dump_enemerate(iter(enumerations), indent, ", ".join(arg_replacement), objName, func, arg_replacement)
                
    with(open(output_file, 'w')) as f:
        f.write(header_template)
        f.write(tests) 
        f.write("\n}\n")
#     print tests

def dump_enemerate(enumeration_iter, indent, args, objName, func, arg_replacement):
    
    def safe_get_next(enumeration_iter):
        try:
            n = next(enumeration_iter)
            return True, n
        except StopIteration:
            return False, None
    
    tests = ""
    valid, enum = safe_get_next(enumeration_iter)
    if valid:
        print(indent, "DUMPING ENUM ", enum, args)
        var_name = prepare_variable_name(enum)
        tests += "{indent}for ({enum} {var_name} : {enum}.values())".format(**locals()) 
        tests += "\n%s{\n" % indent
        tests += dump_enemerate(enumeration_iter, indent + "    ", args, objName, func, arg_replacement)
        tests += "{indent}}}\n".format(**locals())
    else:
        tests += "%s%s.%s(%s);\n" % (indent, objName, func, ", ".join(arg_replacement))
        
    return tests

def dump_single_test(indent, objName, func, arg_replacement, enumerations):
    tests = ""
    args=", ".join(arg_replacement)
    if len(enumerations) == 0:
        tests += "        {objName}.{func}({args});\n".format(**locals())
    else:
        
        enumeration_iter = iter(enumerations)
        dump_enemerate(enumeration_iter, indent, args, objName, func, arg_replacement)
        
    return tests
    

BASE_MOTOR_CONTROLLER_TEMPLATE_HEADER = """package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

public class TestBaseMotorControllerFunctions
{
    private CtreCallback mTestCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting callback " + aName);
        }
    };
    private CtreCallback mTestBuffTrajPointStreamCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting TrajPoint callback " + aName);
        }
    };

    @Test
    public void testAllFunctions()
    {
        CtreJni.registerCanBuffTrajPointStreamCallback(mTestBuffTrajPointStreamCallback);

        TalonSRX talon = new TalonSRX(0);
        TalonSRX followTalon = new TalonSRX(1);
        CtreJni.registerCanMotorCallback(mTestCallback);

        TrajectoryPoint trajectoryPoint = new TrajectoryPoint();
        trajectoryPoint.timeDur = 10;


"""

TALON_TEMPLATE_HEADER = """package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

public class TestTalonSRXFunctions
{
    private CtreCallback mTestCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting callback " + aName);
        }
    };
    private CtreCallback mTestBuffTrajPointStreamCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting TrajPoint callback " + aName);
        }
    };

    @Test
    public void testAllFunctions()
    {
        CtreJni.registerCanBuffTrajPointStreamCallback(mTestBuffTrajPointStreamCallback);

        TalonSRX talon = new TalonSRX(0);
        CtreJni.registerCanMotorCallback(mTestCallback);

        TrajectoryPoint trajectoryPoint = new TrajectoryPoint();
        trajectoryPoint.timeDur = 10;

        //////////////////////////////////////////////

"""

PIGEON_TEMPLATE_HEADER = """package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;
import com.ctre.phoenix.sensors.PigeonIMU.GeneralStatus;
import com.ctre.phoenix.sensors.PigeonIMUConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;
import com.ctre.phoenix.sensors.PigeonIMU_Faults;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU_StickyFaults;

@Tag("CTRE")
public class TestPigeonIMUFunctions
{
    private CtreCallback mTestCallback = new CtreCallback()
    {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount)
        {
            System.out.println("Getting callback " + aName);
        }
    };

    @Test
    public void testAllFunctions()
    {
        PigeonIMU imu = new PigeonIMU(0);
        CtreJni.registerCanPigeonImuCallback(mTestCallback);

"""

CANNIFIER_TEMPLATE_HEADER = """package com.snobot.simulator.ctre;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.CANifier.PinValues;
import com.ctre.phoenix.CANifierConfiguration;
import com.ctre.phoenix.CANifierControlFrame;
import com.ctre.phoenix.CANifierFaults;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.CANifierStickyFaults;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.VelocityPeriod;

public class TestCanifierFunctions {

    private CtreCallback mTestCallback = new CtreCallback() {

        @Override
        public void callback(String aName, int aDeviceId, ByteBuffer aBuffer, int aCount) {
            System.out.println("Getting callback " + aName);
        }
    };

    @Test
    public void testAllFunctions()
    {
        CANifier canifier = new CANifier(0);
        CtreJni.registerCanCanifierCallback(mTestCallback);

"""

JAVA_PATH = r'C:\Users\Public\wpilib\2020\jdk'
LIB_VERSION = "5.17.2"
LIB_HASH = "ed192c71cffd6eb083149d27e3386acd4f1225b0"
ARTIFACT_PACKAGE = "com.ctre.phoenix"
ARTIFACT_NAME = "api-java"
create_tests(r'C:\Users\PJ\.gradle\caches\modules-2\files-2.1\{artifact_package}\{artifact_name}\{lib_version}\{lib_hash}/{artifact_name}-{lib_version}.jar'.format(
        lib_hash=LIB_HASH,
        lib_version=LIB_VERSION,
        artifact_package=ARTIFACT_PACKAGE,
        artifact_name=ARTIFACT_NAME))
