'''
* @file ros_interface.py
* @author Gustavo Diaz H.
* @date 04 Aug 2020
* @brief ROS node interface for communication with
*        DRV10987 BLDC driver Arduino Firmware
'''

import rospy
from threading import Thread
from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from std_msgs.msg import UInt16MultiArray
from timeit import default_timer as timer
from serial_interface import serialInterface
from file_handler import fileHandler

class DRV10987Interface:
    def __init__(self, port, baud, folder, test_name, test_specs):
        # Only argument stuff
        self.running = False
        self.test_specs = test_specs
        self.serial_trx = serialInterface(port, baud, debug = True)
        self.file_handler = fileHandler(folder, test_name)
        self.cmd_code = 0
        self.speed = 0

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
        Thread(target=self.update_state).start()
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

    def update_state(self):
        # read data from uart and publish it on respective topic
        rate = rospy.Rate(self.state_update_rate)
        initial_time = timer()
        while self.running and not rospy.is_shutdown():
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
                time= timer() - initial_time    #[s]
                # save data
                self.file_handler.historical_time.append(time)
                self.file_handler.historical_current_ma.append(current_ma_1)
                self.file_handler.historical_speed_rpm.append(speed_rpm_1)
                self.file_handler.historical_speed_cmd.append(speed_cmd_1)
            rate.sleep()

if __name__ == '__main__':
    # Sirial comunication parameters
    port = '/dev/ttyACM0'
    baud = 115200
    # Test Information
    rospy.init_node('ros_uart_interface')
    test_name = input("test_name: ")
    if test_name=="dummy":
        folder = "dummy_data/"
        test_specs = None
    else:
        print("Specify Test conditions")
        folder = input("folder: ")
        brief = "Brief: " + input("Brief: ")
        test_specs = [brief]
        ans = input("Add specification? y/n")
        while ans == "y":
            print("type [Name: Value]\n")
            param = input()
            test_specs.append(param)
            ans = input("Add specification? y/n")
    # Start test
    print("Starting DRV10987 ROS-UART Controller")
    controller = DRV10987Interface(port, baud, folder, test_name, test_specs)
    controller.initialize()
    controller.start()
    rospy.spin()
    controller.stop()