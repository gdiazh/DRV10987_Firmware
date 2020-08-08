'''
* @file serial_interface.py
* @author Gustavo Diaz H.
* @date 04 Aug 2020
* @brief Communication interface between PC-Serial and microcontroller-Uart
*        for DRV10987 BLDC driver Arduino Firmware
'''

import serial 
import struct

SET_SPEED_CODE_MOTOR1  = 23

class serialInterface(object):
    def __init__(self, port, baud = 115200, debug = False):
        self.arduino = serial.Serial(port, baud)
        self.size = 10
        self.packet = [0]*self.size
        self.debug = debug
        self.struct = None

    def initialize(self):
        # TO DO: check serial port
        print("Conection succed! starting comunication ...")

    def stop(self):
        print("Conection Finish. Closing ports ...")
        self.arduino.close()

    def reset(self):
        self.packet = [0]*self.size

    def DEBUG_PRINT(self, msg_type, msg):

        if not(self.debug): return
        if msg_type == "info":
            print(chr(27)+"[0;32m"+"[INFO]: "+chr(27)+"[0m" + msg)
        elif msg_type == "warn":
            print(chr(27)+"[0;33m"+"[WARN]: "+chr(27)+"[0m" + msg)
        elif msg_type == "error":
            print(chr(27)+"[0;31m"+"[ERROR]: "+chr(27)+"[0m" + msg)
        elif msg_type == "alert":
            print(chr(27)+"[0;34m"+"[ALERT]: "+chr(27)+"[0m" + msg)
        elif msg_type == "firm_error":
            print(chr(27)+"[1;31m"+"[FIRMWARE ERROR]: "+chr(27)+"[0m" + msg)
        else:
            print("NON implemented Debug print type")

    def checksum(self, packet, sz):
        sum = 0
        for j in range(0,sz-1): sum += packet[j]
        return sum

    def readTest(self):
        data = self.arduino.readline()
        print("data: ", data)

    def read(self):
        i = 0
        k = 0
        self.reset()
        while (k < 2*self.size):
            byte = self.arduino.read(1)
            self.packet[i] = ord(byte)
            i+=1
            if (i==self.size):
                chksm = self.checksum(self.packet, self.size) & 0x00FF #Low byte of data checksum
                if (chksm == self.packet[self.size-1] and chksm !=0 and self.packet[self.size-2] == 253 and self.packet[self.size-3] == 254 and self.packet[self.size-4] == 255):
                    self.DEBUG_PRINT("info", "frame received = "+str(self.packet))
                    p = struct.pack('B'*self.size, self.packet[0],self.packet[1],self.packet[2],self.packet[3],self.packet[4],self.packet[5],self.packet[6],self.packet[7],self.packet[8],self.packet[9])
                    self.packet = struct.unpack("HHHBBBB", p)
                    self.struct = struct.unpack("HHHBBBB", p)
                    self.DEBUG_PRINT("info", "parsed struct = "+str(self.struct))
                    return True
                else:
                    for j in range(0,self.size-1): self.packet[j] = self.packet[j+1] #Shift Left packet
                    self.packet[self.size-1] = 0 #Clean last byte to receive other packet
                    i = self.size-1
                    self.DEBUG_PRINT("warn", "Bad checksum = "+str(chksm))
            k+=1
        # Packet not received Correctly
        self.DEBUG_PRINT("error", "Frame lost")
        self.reset()
        return False

    def write(self, cmd_code, speed):
        if cmd_code == 1:
            cmd = SET_SPEED_CODE_MOTOR1
        else:
            self.DEBUG_PRINT("error", "bad motor ID")
            return
        data = [cmd, speed, 255]
        p = struct.pack('HHB', cmd, speed, 255)
        frame = list(struct.unpack('BBBBB', p))
        cks = self.checksum(frame, len(frame)+1) & 0x00FF #Low byte of data checksum
        frame.append(cks)
        frame_bytes_exa = struct.pack('B'*6, frame[0], frame[1], frame[2], frame[3], frame[4], frame[5])
        self.DEBUG_PRINT("info", "writing to device ...")
        self.DEBUG_PRINT("alert", "cmd = "+str(cmd)+" speed = "+str(speed))
        # self.DEBUG_PRINT("alert", "data = "+str(data))
        # self.DEBUG_PRINT("alert", "frame = "+str(frame))
        # self.DEBUG_PRINT("alert", "frame_bytes_exa = "+str(frame_bytes_exa))
        self.arduino.write(frame_bytes_exa)

if __name__ == '__main__':
    port = '/dev/ttyACM0'
    baud = 115200
    serial_trx = serialInterface(port, baud, debug = True)
    serial_trx.initialize()
    serial_trx.write(1, 100)
    # test read
    while True:
        serial_trx.readTest()
    serial_trx.stop()