'''
* @file ros_interface.py
* @author Gustavo Diaz H.
* @date 08 Aug 2020
* @brief serial-uart script to performe a test input signal to
*        DRV10987 BLDC driver Arduino Firmware
'''

from timeit import default_timer as timer
from serial_interface import serialInterface
from file_handler import fileHandler

class DRV10987Controller:
    def __init__(self, port, baud, folder, test_name, test_params, test_specs):
        # Only argument stuff
        self.test_specs = test_specs
        self.serial_trx = serialInterface(port, baud, debug = False)
        self.file_handler = fileHandler(folder, test_name)
        self.cmd_times = test_params['cmd_times']   #[s]
        self.cmd_values = test_params['cmd_values'] #[cmd]

    def initialize(self):
        # Get params and allocate msgs
        self.serial_trx.initialize()
        self.file_handler.init()

    def stop(self):
        print("Saving data ...")
        self.file_handler.save(self.test_specs)

    def step(self):
        initial_time = timer()              #[s]
        time = timer() - initial_time       #[s]
        step_stoped = False
        CMD1_OFF = True
        CMD2_OFF = True
        CMD3_OFF = True
        CMD4_OFF = True
        CMD5_OFF = True
        while True:
            # Send speed cmd's according to specified times
            if self.cmd_times[0] <= time < self.cmd_times[1] and CMD1_OFF:
                print(chr(27)+"[1;31m"+"CMD: "+str(self.cmd_values[0])+chr(27)+"[0m")
                self.serial_trx.write(1, int(self.cmd_values[0]))
                CMD1_OFF = False
            elif self.cmd_times[1] <= time < self.cmd_times[2] and CMD2_OFF:
                print(chr(27)+"[1;31m"+"CMD: "+str(self.cmd_values[1])+chr(27)+"[0m")
                self.serial_trx.write(1, int(self.cmd_values[1]))
                CMD2_OFF = False
            elif self.cmd_times[2] <= time < self.cmd_times[3] and CMD3_OFF:
                print(chr(27)+"[1;31m"+"CMD: "+str(self.cmd_values[2])+chr(27)+"[0m")
                self.serial_trx.write(1, int(self.cmd_values[2]))
                CMD3_OFF = False
            elif self.cmd_times[3] <= time < self.cmd_times[4] and CMD4_OFF:
                print(chr(27)+"[1;31m"+"CMD: "+str(self.cmd_values[3])+chr(27)+"[0m")
                self.serial_trx.write(1, int(self.cmd_values[3]))
                CMD4_OFF = False
            elif self.cmd_times[4] <= time < self.cmd_times[5] and CMD5_OFF:
                print(chr(27)+"[1;31m"+"Turning off"+chr(27)+"[0m")
                self.serial_trx.write(1, 0)
                CMD5_OFF = False
            elif self.cmd_times[5] < time:
                # Test Finished
                print(chr(27)+"[1;31m"+"Closing"+chr(27)+"[0m")
                break
            # Read Data
            if(self.serial_trx.read()):
                # read data
                data = self.serial_trx.struct 
                current_ma_1 = 1.46484375*(data[0]-1023)
                speed_rpm_1 = data[1]
                speed_cmd_1 = data[2]
                # ignore outliers
                # if current_ma_1>=200 or current_ma_1<=-200: current_ma_1 = -1
                # get time
                time = timer() - initial_time    #[s]
                # save data
                self.file_handler.historical_time.append(time)
                self.file_handler.historical_current_ma.append(current_ma_1)
                self.file_handler.historical_speed_rpm.append(speed_rpm_1)
                self.file_handler.historical_speed_cmd.append(speed_cmd_1)
        self.serial_trx.stop()
        print(chr(27)+"[1;31m"+"Test Finished"+chr(27)+"[0m")

if __name__ == '__main__':
    # Sirial comunication parameters
    port = '/dev/ttyACM0'
    baud = 115200
    # Test Information
    test_name = input("test_name: ")
    if test_name=="dummy":
        folder = "data/dummy/"
        test_params = {'cmd_times': [0, 10, 20, 30, 40, 42], 'cmd_values': [123, 234, 287, 182, 0]}
        test_specs = None
    else:
        print("Specify Test conditions")
        default_conf = input(chr(27)+"[0;33m"+"Used default configuration (used only if you are sure) y/n?: "+chr(27)+"[0m")
        if default_conf == "y":
            folder = "data/signalm1/"
            brief = "Brief: Speed input signal to drv through uart"
            motor_type = "Motor type (dvd/HDD/Other): dvd"
            flywheel_type = "Flywheel type (medium/rods/small/No): No"
            motor_alignment = "Motor Rot. Axis alignment (vertical/horizontal): horizontal"
        else:
            folder = "data/"+input("folder: data/") + "/"
            brief = "Brief: " + input("Brief: ")
            motor_type = "Motor type (dvd/HDD/Other): " + input("Motor type (dvd/HDD/Other): ")
            flywheel_type = "Flywheel type (medium/rods/small/No): " + input("Flywheel type (medium/rods/small/No): ")
            motor_alignment = "Motor Rot. Axis alignment (vertical/horizontal): " + input("Motor Rot. Axis alignment (vertical/horizontal): ")
        test_params = {'cmd_times': [0, 10, 20, 30, 40, 42], 'cmd_values': [123, 234, 287, 182, 0]}
        test_specs = [brief,
                      motor_type,
                      flywheel_type,
                      motor_alignment,
                      "cmd_times[s]: " + str(test_params['cmd_times']),
                      "cmd_values[cmd]: " + str(test_params['cmd_values'])]
        if default_conf == "y":
            test_specs.append("Voltage[V]: 12.0")
        else:
            ans = input("Add specification? y/n: ")
            while ans == "y":
                print("type: Name[units]: Value\n")
                param = input()
                test_specs.append(param)
                ans = input("Add specification? y/n: ")
    # Start test
    print(chr(27)+"[0;34m"+"Running Speed-Step Input DRV10987 Test ..."+chr(27)+"[0m")
    controller = DRV10987Controller(port, baud, folder, test_name, test_params, test_specs)
    controller.initialize()
    controller.step()
    controller.stop()