/*
* @file drv10987_firmware.ino
* @author Gustavo Diaz H.
* @date 01 Aug 2020
* @brief Reaction wheel's Firmware using three DRV10987 BLDC drivers
*/

#include "drv10987_firmware.h"

#define SerialPort SerialUSB

DRV10987Firmware drv;

void setup()
{
    drv.begin();
    // Wire.onReceive(readMasterWrite);
    // Wire.onRequest(responseToMasterRead);
    drv.setDefault(0);
}

void loop()
{ 
    drv.runUartControllerCommand();
    // drv.runI2cControllerCommand();
    drv.readStatesDRVs();
    drv.sendDataUart();
}

void readMasterWrite(int n)
{
    if (Wire.available())
    {
        memset(drv.i2c_rx_frame, 0, I2C_CMD_SZ);
        drv.i2c_rx_frame[0] = Wire.read();
        drv.i2c_rx_frame[1] = 0;
        drv.i2c_rx_frame[2] = Wire.read();
        drv.i2c_rx_frame[3] = Wire.read();
        memcpy(&drv.i2c_cmd, drv.i2c_rx_frame, sizeof(drv.i2c_cmd));
        drv.i2c_cmd_result = 1;
    }
}

void responseToMasterRead()
{
    if (drv.i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR1)
    {
        drv.write2b(drv.motor1_data.raw_speed[0], drv.motor1_data.raw_speed[1]);
    }
    else if (drv.i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR1)
    {
        drv.write2b(drv.motor1_data.raw_current[0], drv.motor1_data.raw_current[1]);
    }
}