'''
* @file ros_interface.py
* @author Gustavo Diaz H.
* @date 07 Aug 2020
* @brief ROS node to performe a Step input to
*        DRV10987 BLDC driver Arduino Firmware
'''

import rospy
import sys
from threading import Thread
from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from std_msgs.msg import UInt16MultiArray
from timeit import default_timer as timer
from serial_interface import serialInterface
from file_handler import fileHandler

class DRV10987Controller:
    def __init__(self, port, baud, folder, test_name, test_params, test_specs):
        # Only argument stuff
        self.running = False
        self.test_specs = test_specs
        self.serial_trx = serialInterface(port, baud, debug = True)
        self.file_handler = fileHandler(folder, test_name)
        self.cmd_code = 0                   #[cmd]
        self.speed = 0                      #[rpm]
        self.step_time = test_params[0]      #[s]
        self.step_value = test_params[1]     #[cmd]
        self.coasting_time = test_params[2]  #[s]

    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param('/rate', 1000)
        self.serial_trx.initialize()
        self.file_handler.init()

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        #publishers
        self.data1_pub = rospy.Publisher('/current_ma', Float32, queue_size=70)
        self.data2_pub = rospy.Publisher('/speed_rpm', UInt16, queue_size=70)
        self.data3_pub = rospy.Publisher('/speed_cmd', UInt16, queue_size=70)
        Thread(target=self.step).start()
        #subscribers
        self.cmd_sub = rospy.Subscriber('/cmd', UInt16MultiArray, self.cmd)

    def stop(self):
        self.running = False
        self.data1_pub.unregister()
        self.data2_pub.unregister()
        self.data3_pub.unregister()
        self.cmd_sub.unregister()
        print("Saving data ...")
        self.file_handler.save(self.test_specs)

    def cmd(self, msg):
        # get input from /cmd topic and send it by uart via serialInterface
        self.cmd_code = msg.data[0] #motor id
        self.speed = msg.data[1] #motor speed
        self.serial_trx.write(self.cmd_code, self.speed)

    def step(self):
        # read data from uart and publish it on respective topic
        rate = rospy.Rate(self.state_update_rate)
        initial_time = timer()
        time = timer() - initial_time    #[s]
        step_stoped = False
        # Send speed cmd
        self.serial_trx.write(1, int(self.step_value))
        while time < self.step_time + self.coasting_time:
            if(self.serial_trx.read()):
                # read data
                data = self.serial_trx.struct 
                current_ma_1 = 1.46484375*(data[0]-1023)
                speed_rpm_1 = data[1]
                speed_cmd_1 = data[2]
                # ignore outliers
                # if current_ma_1>=200 or current_ma_1<=-200: current_ma_1 = -1
                # publish data
                self.data1_pub.publish(current_ma_1)
                self.data2_pub.publish(speed_rpm_1)
                self.data3_pub.publish(speed_cmd_1)
                # get time
                time = timer() - initial_time    #[s]
                # save data
                self.file_handler.historical_time.append(time)
                self.file_handler.historical_current_ma.append(current_ma_1)
                self.file_handler.historical_speed_rpm.append(speed_rpm_1)
                self.file_handler.historical_speed_cmd.append(speed_cmd_1)
                if time > self.step_time and not(step_stoped):
                    self.serial_trx.write(1, 0)
                    step_stoped = True
            rate.sleep()
        self.serial_trx.stop()
        print(chr(27)+"[1;31m"+"Test Finished"+chr(27)+"[0m")
        rospy.signal_shutdown("Shutdown Node")

if __name__ == '__main__':
    # Sirial comunication parameters
    port = '/dev/ttyACM0'
    baud = 115200
    # Test Information
    rospy.init_node('ros_uart_interface')
    test_name = input("test_name: ")
    if test_name=="dummy":
        folder = "data/steps/"
        test_params = [8, 100, 2]
        test_specs = None
    else:
        print("Specify Test conditions")
        default_conf = input("Used default configuration (used only if you are sure) y/n?: ")
        if default_conf == "y":
            folder = "data/ros-steps/"
            brief = "Brief: Speed step input to drv through ros-uart"
            motor_type = "Motor type (dvd/HDD/Other): dvd"
            flywheel_type = "Flywheel type (medium/rods/small/No): No"
            stop_time = "8"
            step_value = input("Step value[cmd](Sugested > 60): ")
            coasting_time = "2"
        else:
            folder = "data/"+input("folder: data/") + "/"
            brief = "Brief: " + input("Brief: ")
            motor_type = "Motor type (dvd/HDD/Other): " + input("Motor type (dvd/HDD/Other): ")
            flywheel_type = "Flywheel type (medium/rods/small/No): " + input("Flywheel type (medium/rods/small/No): ")
            stop_time = input("Stop time[s](Sugested > 7): ")
            step_value = input("Step value[cmd](Sugested > 60): ")
            coasting_time = input("Coasting time[s](Sugested > 1): ")
        test_params = [float(stop_time), float(step_value), float(coasting_time)]
        test_specs = [brief,
                      motor_type,
                      flywheel_type,
                      "Stop time[s]: " + stop_time,
                      "Step value[cmd]: " + step_value,
                      "Coasting time[s]: " + coasting_time]
        if default_conf == "y":
            test_specs.append("Voltage[V]: 12.0")
        else:
            ans = input("Add specification? y/n: ")
            while ans == "y":
                print("type [Name: Value]\n")
                param = input()
                test_specs.append(param)
                ans = input("Add specification? y/n: ")
    # Start test
    print(chr(27)+"[0;34m"+"Starting Speed-Step Input DRV10987 Test"+chr(27)+"[0m")
    controller = DRV10987Controller(port, baud, folder, test_name, test_params, test_specs)
    controller.initialize()
    controller.start()
    rospy.spin()
    controller.stop()