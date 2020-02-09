
import os
import requests
from zipfile import ZipFile

def main():
    version = "5.17.2"

    download_url = "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/cci/%s/cci-%s-headers.zip" % (version, version)
    myfile = requests.get(download_url)
    local_zip_file = 'build/cci_zip.zip'
    # open(local_zip_file, 'wb').write(myfile.content)

    files_of_interest = []
    files_of_interest.append('ctre/phoenix/cci/BuffTrajPointStream_CCI.h')
    files_of_interest.append('ctre/phoenix/cci/CANifier_CCI.h')
    files_of_interest.append('ctre/phoenix/cci/CCI.h')
    files_of_interest.append('ctre/phoenix/cci/Logger_CCI.h')
    files_of_interest.append('ctre/phoenix/cci/MotController_CCI.h')
    files_of_interest.append('ctre/phoenix/cci/PigeonIMU_CCI.h')
    files_of_interest.append('ctre/phoenix/core/GadgeteerUartClient.h')
    files_of_interest.append('ctre/phoenix/jni/com_ctre_phoenix_CANifierJNI.h')
    files_of_interest.append('ctre/phoenix/jni/com_ctre_phoenix_CTRLoggerJNI.h')
    files_of_interest.append('ctre/phoenix/jni/com_ctre_phoenix_motion_BuffTrajPointStreamJNI.h')
    files_of_interest.append('ctre/phoenix/jni/com_ctre_phoenix_motorcontrol_can_MotControllerJNI.h')
    files_of_interest.append('ctre/phoenix/jni/com_ctre_phoenix_platform_can_PlatformCANJNI.h')
    files_of_interest.append('ctre/phoenix/jni/com_ctre_phoenix_platform_PlatformJNI.h')
    files_of_interest.append('ctre/phoenix/jni/com_ctre_phoenix_sensors_CANCoderJNI.h')
    files_of_interest.append('ctre/phoenix/jni/com_ctre_phoenix_sensors_PigeonImuJNI.h')
    files_of_interest.append('ctre/phoenix/jni/com_ctre_phoenix_unmanaged_UnmanagedJNI.h')
    files_of_interest.append('ctre/phoenix/motorcontrol/ControlMode.h')
    files_of_interest.append('ctre/phoenix/motorcontrol/MotorCommutation.h')
    files_of_interest.append('ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h')
    files_of_interest.append('ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h')
    files_of_interest.append('ctre/phoenix/sensors/AbsoluteSensorRange.h')
    files_of_interest.append('ctre/phoenix/sensors/SensorInitializationStrategy.h')

    with ZipFile(local_zip_file, 'r') as zib_obj:
        listOfFileNames = zib_obj.namelist()
        for file_in_zip in listOfFileNames:
            if file_in_zip in files_of_interest:
                output_path = os.path.abspath(os.path.join("ctre_source/cci/native/include/", file_in_zip))
                print("Got one", file_in_zip)
                print("output path", output_path)
                zib_obj.extract(file_in_zip, "ctre_source/cci/native/include/")


if __name__ == "__main__":
    main()