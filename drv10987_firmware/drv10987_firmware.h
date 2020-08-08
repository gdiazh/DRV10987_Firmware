/*
* @file drv10987_firmware.h
* @author Gustavo Diaz H.
* @date 01 Aug 2020
* @brief Reaction wheel's Firmware using three DRV10987 BLDC drivers
*/

#include <Wire.h>
#include "wiring_private.h"
#include <Arduino.h>
// Device i2c address
#define SAMD21GD_ADDR 0x11
#define DRV10987_ADDR 0x52
#define DRV10987_DEVID_ADDR 0X08
#define DRV10987_CONFIG1_ADDR 0X90
#define DRV10987_CONFIG2_ADDR 0X91
#define DRV10987_CONFIG3_ADDR 0X92
#define DRV10987_CONFIG4_ADDR 0X93
#define DRV10987_CONFIG5_ADDR 0X94
#define DRV10987_CONFIG6_ADDR 0X95
#define DRV10987_CONFIG7_ADDR 0X96
#define DRV10987_EECTRL_ADDR 0x60
#define DRV10987_EEPROM_PROG1_ADDR 0x31
#define DRV10987_EEPROM_PROG2_ADDR 0x32
#define DRV10987_EEPROM_PROG5_ADDR 0x35
#define DRV10987_MOTORSPEED_STATUS_ADDR 0x01
#define DRV10987_MOTORCURRENT_STATUS_ADDR 0x04
#define DRV10987_MOTORSPEED_ADDR 0x30
// Firmware commands
#define SAMPLE_SPEED_CODE_MOTOR1 21
#define SAMPLE_CURRENT_CODE_MOTOR1 22
#define SET_SPEED_CODE_MOTOR1 23
#define MOTOR1_ID 1
// Motor Parameters
#define CONSTANT_RM 0x3C //RPH_CT = 1.8624[Ohm], the closest to 1.9[Ohm] measured
#define CONSTANT_KT 0x19 //Kt = 16.56[mV/Hz], the closest to 16.2504 [mV/Hz] measured
// Pin Assignments
#define DEBUG_LED_1 9
#ifndef SerialPort
    #define SerialPort SerialUSB
#endif
// Data structures
#define MOTOR_DT_SZ 12
typedef struct{
    float speed;
    float current;
    uint8_t raw_speed[2];
    uint8_t raw_current[2];
}motor_data_t;

#define I2C_CMD_SZ 4
typedef struct{
    uint16_t code;
    uint16_t speed;
}i2c_command_t;

#define UART_CMD_SZ 6
typedef struct{
    uint16_t code;
    uint16_t speed;
    uint8_t tail;
    uint8_t checksum;
}uart_command_t;

#define UART_PCKT_SZ 10
typedef struct{
    uint16_t raw_current;
    uint16_t raw_speed;
    uint16_t cmd_speed;
    uint8_t tail1;
    uint8_t tail2;
    uint8_t tail3;
    uint8_t checksum;
}uart_packet_t;

class DRV10987Firmware
{
    // Private Members
    // DRV10987 Configuration registers states
    uint8_t eeReadyStatus_[2] = {0, 0};
    uint8_t config1Status_[2] = {0, 0};
    uint8_t config2Status_[2] = {0, 0};
    uint8_t config3Status_[2] = {0, 0};
    uint8_t config4Status_[2] = {0, 0};
    uint8_t config5Status_[2] = {0, 0};
    uint8_t config6Status_[2] = {0, 0};
    uint8_t config7Status_[2] = {0, 0};
    uint8_t devId_[2] = {0, 0};
    // motors data to send through uart interface
    uart_packet_t uart_packet_;
public:
    // Public Members
    // Command handlers
    i2c_command_t i2c_cmd;
    uart_command_t uart_cmd;
    uint8_t i2c_cmd_result;
    uint8_t i2c_rx_frame[I2C_CMD_SZ];
    // Motor State handlers
    motor_data_t motor1_data;
    // Constructor
    DRV10987Firmware()
    {}
    // methods
    void begin(void);
    void setDefault(uint16_t m1_spd);
    uint8_t readCommandUart(void);
    void sendDataUart(void);
    void runI2cControllerCommand(void);
    void runUartControllerCommand(void);
    void readStatesDRVs(void);
    void setMotorSpeed(uint8_t motor_id, float speed);
    void drvRead(uint8_t motor_id, uint8_t reg_addr, uint8_t *data);
    void drvWrite(uint8_t motor_id, uint8_t reg_addr, uint8_t data1, uint8_t data2);
    void write2b(uint8_t data1, uint8_t data2);
    uint8_t checksum(uint8_t *packet, uint8_t n);
};