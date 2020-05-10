import os
import requests
from zipfile import ZipFile

URL_BASE = "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/"

def extract_source_files(version, original_source_dir, redownload=False):
    __extract_cci_files(version, original_source_dir, redownload)
    __extract_core_files(version, original_source_dir, redownload)
    __extract_java_files(version, original_source_dir, redownload)
    __extract_java_test_files(version, original_source_dir, redownload)


def __download_file(url, output_file, redownload):
    myfile = requests.get(url)
    if not os.path.exists(output_file) or redownload:
        print("Downloading {} to {}".format(url, output_file))
        open(output_file, 'wb').write(myfile.content)

    return output_file


def __extract_java_files(version, original_source_dir, redownload):

    print("Extracting java...")
    local_zip_file = __download_file(URL_BASE + "api-java/%s/api-java-%s-sources.jar" % (version, version), "../build/java_zip.jar", redownload)

    files_of_interest = []
    files_of_interest.append('com/ctre/phoenix/CTREJNIWrapper.java')

    with ZipFile(local_zip_file, 'r') as zib_obj:
        listOfFileNames = zib_obj.namelist()
        for file_in_zip in listOfFileNames:
            should_unzip = False
            if file_in_zip.endswith("JNI.java"):
                should_unzip = True
            elif file_in_zip in files_of_interest:
                should_unzip = True

            if should_unzip:
                zib_obj.extract(file_in_zip, os.path.join(original_source_dir, "api-java/java/"))


def __extract_java_test_files(version, original_source_dir, redownload):

    print("Extracting test java...")
    local_zip_file = __download_file(URL_BASE + "api-java/%s/api-java-%s-sources.jar" % (version, version), "../build/java_zip.jar", redownload)

    files_of_interest = []
    files_of_interest.append('com/ctre/phoenix/motorcontrol/can/BaseMotorController.java')
    files_of_interest.append('com/ctre/phoenix/motorcontrol/can/BaseTalon.java')
    files_of_interest.append('com/ctre/phoenix/motorcontrol/can/TalonFX.java')
    files_of_interest.append('com/ctre/phoenix/motorcontrol/can/TalonSRX.java')
    files_of_interest.append('com/ctre/phoenix/motorcontrol/can/VictorSPX.java')
    files_of_interest.append('com/ctre/phoenix/sensors/CANCoder.java')
    files_of_interest.append('com/ctre/phoenix/sensors/PigeonIMU.java')
    files_of_interest.append('com/ctre/phoenix/CANifier.java')

    with ZipFile(local_zip_file, 'r') as zib_obj:
        listOfFileNames = zib_obj.namelist()
        for file_in_zip in listOfFileNames:
            should_unzip = False
            if file_in_zip in files_of_interest:
                should_unzip = True
                print("  Found one in white list", file_in_zip)
            else:
                print("  Ignoring\t\t\t", file_in_zip)

            if should_unzip:
                zib_obj.extract(
                    file_in_zip,
                    os.path.join(original_source_dir, "../build/tmp/test_files"))


def __extract_cci_files(version, original_source_dir, redownload=False):

    print("Extracting CCI files")
    local_zip_file = __download_file(URL_BASE + "cci/%s/cci-%s-headers.zip" % (version, version), "../build/cci_zip.jar", redownload)

    files_of_interest = []
    files_of_interest.append('ctre/phoenix/ErrorCode.h')
    files_of_interest.append('ctre/phoenix/cci/CCI.h')
    files_of_interest.append('ctre/phoenix/core/GadgeteerUartClient.h')
    files_of_interest.append('ctre/phoenix/motorcontrol/ControlMode.h')
    files_of_interest.append('ctre/phoenix/motorcontrol/MotorCommutation.h')
    files_of_interest.append('ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h')
    files_of_interest.append('ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h')
    files_of_interest.append('ctre/phoenix/sensors/AbsoluteSensorRange.h')
    files_of_interest.append('ctre/phoenix/sensors/SensorInitializationStrategy.h')
    files_of_interest.append('ctre/phoenix/sensors/MagnetFieldStrength.h')
    files_of_interest.append('ctre/phoenix/sensors/SensorTimeBase.h')
    files_of_interest.append('ctre/phoenix/sensors/SensorVelocityMeasPeriod.h')

    with ZipFile(local_zip_file, 'r') as zib_obj:
        listOfFileNames = zib_obj.namelist()
        for file_in_zip in listOfFileNames:
            should_unzip = False
            if file_in_zip.endswith("CCI.h"):
                should_unzip = True
                print("  Found a CCI file\t", file_in_zip)
            elif file_in_zip.endswith("JNI.h"):
                should_unzip = True
                print("  Found a JNI file\t", file_in_zip)
            elif file_in_zip in files_of_interest:
                should_unzip = True
                print("  Found one in white list\t", file_in_zip)
            else:
                print("  Ignoring\t\t\t", file_in_zip)

            if should_unzip:
                zib_obj.extract(
                    file_in_zip,
                    os.path.join(original_source_dir, "cci/native/include/"))


def __extract_core_files(version, original_source_dir, redownload=False):

    print("Extracting Core files")
    local_zip_file = __download_file(
        URL_BASE + "core/%s/core-%s-headers.zip"
        % (version, version), "../build/core_zip.jar", redownload)

    files_of_interest = []
    files_of_interest.append('ctre/phoenix/ErrorCode.h')
    files_of_interest.append('ctre/phoenix/cci/CCI.h')
    files_of_interest.append('ctre/phoenix/string_util/string_util.h')

    with ZipFile(local_zip_file, 'r') as zib_obj:
        listOfFileNames = zib_obj.namelist()
        for file_in_zip in listOfFileNames:
            should_unzip = False
            if file_in_zip in files_of_interest:
                should_unzip = True
                print("  Found one in white list\t", file_in_zip)
            else:
                print("  Ignoring\t\t\t", file_in_zip)

            if should_unzip:
                zib_obj.extract(
                    file_in_zip,
                    os.path.join(original_source_dir, "core/native/include/"))
