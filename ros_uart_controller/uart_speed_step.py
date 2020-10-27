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
        self.serial_trx = serialInterface(port, baud, debug = False)
        self.file_handler = fileHandler(folder, test_name)
        self.cmd_code = 0                                   #[cmd]
        self.speed = 0                                      #[rpm]
        self.steady_time = 5                                #[s]
        self.finalcoasting_time = 3                         #[s]
        self.stop_time = test_params[0]+self.steady_time    #[s]
        self.initial_value = test_params[1]                 #[cmd]
        self.step_value = test_params[2]                    #[cmd]
        self.coasting_time = test_params[3]                 #[s]

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
        ST1 = True
        ST2 = True
        ST3 = True
        while time < self.stop_time + self.finalcoasting_time + self.coasting_time:
            if 0 <= time and ST1:   # Send initial speed cmd value
                print(chr(27)+"[1;31m"+"Initial Speed CMD: "+str(self.initial_value)+chr(27)+"[0m")
                self.serial_trx.write(1, int(self.initial_value))
                ST1 = False
            elif self.steady_time <= time and ST2: # Send step speed cmd value
                print(chr(27)+"[1;31m"+"Step Value CMD: "+str(self.step_value)+chr(27)+"[0m")
                self.serial_trx.write(1, int(self.step_value))
                ST2 = False
            elif self.stop_time <= time and ST3: # Send final step speed cmd value
                print(chr(27)+"[1;31m"+"Final Step Value CMD: "+str(self.initial_value)+chr(27)+"[0m")
                self.serial_trx.write(1, int(self.initial_value))
                ST3 = False
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
            if time > self.stop_time + self.finalcoasting_time and not(step_stoped):
                print(chr(27)+"[1;31m"+"Stopping CMD: "+str(0)+chr(27)+"[0m")
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
        test_params = [8, 0, 100, 2]
        test_specs = None
    else:
        print("Specify Test conditions")
        default_conf = input(chr(27)+"[0;33m"+"Used default configuration (used only if you are sure) y/n?: "+chr(27)+"[0m")
        if default_conf == "y":
            folder = "data/inisteps/"
            brief = "Brief: Speed step input to drv through uart"
            motor_type = "Motor type (dvd/HDD/Other): dvd"
            flywheel_type = "Flywheel type (medium/rods/small/No): No"
            motor_alignment = "Motor Rot. Axis alignment (vertical/horizontal): horizontal"
            stop_time = "7"
            initial_value = "100"
            step_value = input("Step value[cmd](Sugested > 60): ")
            coasting_time = "2"
        else:
            folder = "data/"+input("folder: data/") + "/"
            brief = "Brief: " + input("Brief: ")
            motor_type = "Motor type (dvd/HDD/Other): " + input("Motor type (dvd/HDD/Other): ")
            flywheel_type = "Flywheel type (medium/rods/small/No): " + input("Flywheel type (medium/rods/small/No): ")
            motor_alignment = "Motor Rot. Axis alignment (vertical/horizontal): " + input("Motor Rot. Axis alignment (vertical/horizontal): ")
            stop_time = input("Stop time[s](Sugested > 7): ")
            initial_value = input("Initial step value[cmd](Sugested > 60): ")
            step_value = input("Step value[cmd](Sugested > 60): ")
            coasting_time = input("Coasting time[s](Sugested > 1): ")
        test_params = [float(stop_time), float(initial_value), float(step_value), float(coasting_time)]
        test_specs = [brief,
                      motor_type,
                      flywheel_type,
                      motor_alignment,
                      "Stop time[s]: " + stop_time,
                      "Initial step value[cmd]: " + initial_value,
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