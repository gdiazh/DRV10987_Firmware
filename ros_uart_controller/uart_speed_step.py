'''
* @file ros_interface.py
* @author Gustavo Diaz H.
* @date 08 Aug 2020
* @brief serial-uart script to performe a Speed Step input to
*        DRV10987 BLDC driver Arduino Firmware
'''

from timeit import default_timer as timer
from serial_interface import serialInterface
from file_handler import fileHandler

class DRV10987Controller:
    def __init__(self, port, baud, folder, test_name, test_params, test_specs):
        # Only argument stuff
        self.test_specs = test_specs
        self.serial_trx = serialInterface(port, baud, debug = True)
        self.file_handler = fileHandler(folder, test_name)
        self.cmd_code = 0                    #[cmd]
        self.speed = 0                       #[rpm]
        self.stop_time = test_params[0]      #[s]
        self.step_value = test_params[1]     #[cmd]
        self.coasting_time = test_params[2]  #[s]

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
        # Send speed cmd
        self.serial_trx.write(1, int(self.step_value))
        while time < self.stop_time + self.coasting_time:
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
                if time > self.stop_time and not(step_stoped):
                    self.serial_trx.write(1, 0)
                    step_stoped = True
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
        test_params = [8, 100, 2]
        test_specs = None
    else:
        print("Specify Test conditions")
        default_conf = input(chr(27)+"[0;33m"+"Used default configuration (used only if you are sure) y/n?: "+chr(27)+"[0m")
        if default_conf == "y":
            folder = "data/steps/"
            brief = "Brief: Speed step input to drv through uart"
            motor_type = "Motor type (dvd/HDD/Other): dvd"
            flywheel_type = "Flywheel type (medium/rods/small/No): No"
            motor_alignment = "Motor Rot. Axis alignment (vertical/horizontal): horizontal"
            stop_time = "8"
            step_value = input("Step value[cmd](Sugested > 60): ")
            coasting_time = "2"
        else:
            folder = "data/"+input("folder: data/") + "/"
            brief = "Brief: " + input("Brief: ")
            motor_type = "Motor type (dvd/HDD/Other): " + input("Motor type (dvd/HDD/Other): ")
            flywheel_type = "Flywheel type (medium/rods/small/No): " + input("Flywheel type (medium/rods/small/No): ")
            motor_alignment = "Motor Rot. Axis alignment (vertical/horizontal): " + input("Motor Rot. Axis alignment (vertical/horizontal): ")
            stop_time = input("Stop time[s](Sugested > 7): ")
            step_value = input("Step value[cmd](Sugested > 60): ")
            coasting_time = input("Coasting time[s](Sugested > 1): ")
        test_params = [float(stop_time), float(step_value), float(coasting_time)]
        test_specs = [brief,
                      motor_type,
                      flywheel_type,
                      motor_alignment,
                      "Stop time[s]: " + stop_time,
                      "Step value[cmd]: " + step_value,
                      "Coasting time[s]: " + coasting_time]
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