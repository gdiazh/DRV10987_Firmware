/*
* @file drv10987_firmware.cpp
* @author Gustavo Diaz H.
* @date 01 Aug 2020
* @brief Reaction wheel's Firmware using three DRV10987 BLDC drivers
*/

#include "drv10987_firmware.h"

void DRV10987Firmware::begin(void)
{
    // I2C Interface
    Wire.begin(SAMD21GD_ADDR);
    // Wire.setClock(200000);
    // Debug 
    pinMode(DEBUG_LED_1, OUTPUT);
    digitalWrite(DEBUG_LED_1, LOW);
    SerialPort.begin(115200);
    SerialPort.println("DRV10987 Firmware Interface");
}

void DRV10987Firmware::setDefault(uint16_t m1_spd){
    /* Set default values of variables, and motor speed
    *
    * @param m1_spd uint16_t dafault speed motor 1
    */
    // I2C Interface Commands
    i2c_cmd.code = 0;
    i2c_cmd.speed = 0;
    i2c_cmd_result = 0;
    // Motor Data States
    motor1_data.speed = 0;
    motor1_data.current = 0;
    motor1_data.raw_speed[0] = 0;
    motor1_data.raw_speed[1] = 0;
    motor1_data.raw_current[0] = 0;
    motor1_data.raw_current[1] = 0;
    // Uart packet struct
    uart_packet_.raw_current = 0;
    uart_packet_.raw_speed = 0;
    uart_packet_.cmd_speed = 0;
    uart_packet_.tail1 = 255;
    uart_packet_.tail2 = 254;
    uart_packet_.tail3 = 253;
    uart_packet_.checksum = checksum((uint8_t*)&uart_packet_, sizeof(uart_packet_));
    // Set Initial Motor Speed
    setMotorSpeed(MOTOR1_ID, m1_spd);
    // Debug Led State
    digitalWrite(DEBUG_LED_1, LOW);
    delay(100); // wait for the motor to reach initial speed
}

uint8_t DRV10987Firmware::readCommandUart(void)
{
    /* Read data frame containing cmds from uart port
    *
    * @update uart_cmd uart_command_t data structure to save cmds
    * @return sum uint8_t code result, 1 if received ok and 0 i fail
    */
    uint8_t uart_rx_frame[UART_CMD_SZ];
    memset(uart_rx_frame, 0, UART_CMD_SZ);
    uint8_t readed_bytes = 0;
    if (SerialPort.available())
        readed_bytes = SerialPort.readBytes((char*)&uart_rx_frame, UART_CMD_SZ);
    if (readed_bytes == UART_CMD_SZ)
    {
        // fill sttruct
        uint8_t cks = checksum(uart_rx_frame, UART_CMD_SZ);
        if (uart_rx_frame[UART_CMD_SZ-1] == cks && uart_rx_frame[UART_CMD_SZ-2] == 255)
        {
            memcpy(&uart_cmd, uart_rx_frame, sizeof(uart_cmd));
            return 1;
        }
    }
    return 0; //fail rx
}

void DRV10987Firmware::sendDataUart(void)
{
    /* Send motor state through uart port
    */
    uart_packet_.raw_current = (motor1_data.raw_current[0]<<8) | motor1_data.raw_current[1]; //[MSB LSB]
    uart_packet_.raw_speed = (motor1_data.raw_speed[0]<<8) | motor1_data.raw_speed[1]; //[MSB LSB]
    // uart_packet_.time_arduino = millis();
    uart_packet_.tail1 = 255;
    uart_packet_.tail2 = 254;
    uart_packet_.tail3 = 253;
    uart_packet_.checksum = checksum((uint8_t*)&uart_packet_, sizeof(uart_packet_));
    SerialPort.write((uint8_t*)&uart_packet_, sizeof(uart_packet_));
}

void DRV10987Firmware::runI2cControllerCommand(void)
{
    /* Run Master-Controller commands, reading or
    *  writing to the DRV10987 driver
    *  - get_speed
    *  - get_current
    *  - set_speed
    * @update motor1_data motor_data_t speed and current
    */
    if (i2c_cmd_result) // check if there is a valid cmd from the Master-Controller
    {
        if (i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR1)
        {
            drvRead(MOTOR1_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor1_data.raw_speed);
            //TODO:check poles coeficient
            motor1_data.speed = (motor1_data.raw_speed[0]<<8) | motor1_data.raw_speed[1]; //[RPM]
        }
        else if (i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR1)
        {
            drvRead(MOTOR1_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor1_data.raw_current);
            uint16_t current_aux = (motor1_data.raw_current[0]&0x07)<<8 | motor1_data.raw_current[1];
            motor1_data.current = 1.46484375*(current_aux-1023);       //[mA]
        }
        else if (i2c_cmd.code == SET_SPEED_CODE_MOTOR1)
        {
            setMotorSpeed(MOTOR1_ID, i2c_cmd.speed);
            uart_packet_.cmd_speed = i2c_cmd.speed;
        }
        i2c_cmd_result = 0;
    }
}

void DRV10987Firmware::runUartControllerCommand(void)
{
    /* Run Uart-Controller commands,
    *  writing to the DRV10987 driver
    *  - set_speed
    * @update uart_packet_ uart_packet_t data structure to save states
    */
    uint8_t uart_cmd_result = readCommandUart();
    if (uart_cmd_result)
    {
        if (uart_cmd.code == SET_SPEED_CODE_MOTOR1)
        {
            setMotorSpeed(MOTOR1_ID, uart_cmd.speed);
            uart_packet_.cmd_speed = uart_cmd.speed;
        }
    }
}

void DRV10987Firmware::readStatesDRVs(void)
{
    /* Read all motor states from DRV10987 driver
    * through the I2C interface and update data structs
    *
    * @update motor1_data motor_data_t speed(uint16_t) and current(float)
    */
    // Read motor 1 state
    drvRead(MOTOR1_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor1_data.raw_speed);
    motor1_data.speed = (motor1_data.raw_speed[0]<<8) | motor1_data.raw_speed[1];     //[RPM]
    // delay(5);   //TODO: check if/why this delay is necessary
    drvRead(MOTOR1_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor1_data.raw_current);
    uint16_t current_aux1 = (motor1_data.raw_current[0]&0x07)<<8 | motor1_data.raw_current[1];
    motor1_data.current = 1.46484375*(current_aux1-1023);     //[mA]
}

void DRV10987Firmware::setMotorSpeed(uint8_t motor_id, float speed)
{
    /* Write speed value to the corresponding register on DRV10987 driver
    * through the respective I2C interface
    *
    * @param motor_id uint8_t motor id number (valid values under <Firmware commands> include)
    * @param speed float speed value should be in the range [0, 511]
    */
    uint16_t speed_aux = (uint16_t) speed;
    if (speed_aux>511) speed_aux = 511;
    uint8_t speed_H = speed_aux >> 8;       //MSB
    uint8_t speed_L = speed_aux & 0xff;     //LSB
    speed_H = speed_H | 1<<7;               // Enable bit for i2c setting mode

    if (motor_id == MOTOR1_ID)
    {
        drvWrite(MOTOR1_ID, DRV10987_MOTORSPEED_ADDR, speed_H, speed_L);
    }
}

void DRV10987Firmware::drvRead(uint8_t motor_id, uint8_t reg_addr, uint8_t *data)
{
    /* Read 2 bytes of data from given motor id (and therefore respective drv10987 driver)
    * through I2C interface
    *
    * @param motor_id uint8_t motor id number (valid values under <Firmware commands> include)
    * @param reg_addr uint8_t drv10987 register address to read the data (valid values under <device i2c address> include)
    * @param data uint8_t[2] array to save the readed data
    */
    if (motor_id == MOTOR1_ID)
    {   // The motor 1 share the default TWI with an external control device
        Wire.setClock(200000);                          // set shared interface as master temporarily
        Wire.beginTransmission(DRV10987_ADDR);
        uint8_t bytesSent = Wire.write(reg_addr);       // send reg addr to be readed
        uint8_t errcode = Wire.endTransmission(false);  // stop transmitting
        Wire.requestFrom(DRV10987_ADDR, 2);               // request data from given addr
        if (Wire.available())
        {   //TODO: slave may send less than requested, check condition
            data[0] = Wire.read();                      // read MSB of data
            data[1] = Wire.read();                      // read LSB of data
        }
        Wire.begin(SAMD21GD_ADDR);                      // leave shared interface as slave
    }
}

void DRV10987Firmware::drvWrite(uint8_t motor_id, uint8_t reg_addr, uint8_t data1, uint8_t data2)
{
    /* Write 2 bytes of data to given motor id (and therefore respective drv10987 driver)
    * through I2C interface
    *
    * @param motor_id uint8_t motor id number (valid values under <Firmware commands> include)
    * @param reg_addr uint8_t drv10987 register address to write the data (valid values under <device i2c address> include)
    * @param data1 uint8_t data MSB
    * @param data2 uint8_t data LSB
    */
    if (motor_id == MOTOR1_ID)
    {   // The motor 1 share the default TWI with an external control device
        Wire.setClock(200000);                // set shared interface as master temporarily
        Wire.beginTransmission(DRV10987_ADDR);
        Wire.write(reg_addr);                 // send reg addr
        Wire.write(data1);                    // send MSB first
        Wire.write(data2);                    // send LSB second
        Wire.endTransmission(true);           // stop transmitting
        Wire.begin(SAMD21GD_ADDR);            // leave shared interface as slave
    }
}

void DRV10987Firmware::write2b(uint8_t data1, uint8_t data2)
{
    /* Write 2 bytes of data as a response to a read from a Master device
    * through the default I2C interface
    *
    * @param data1 uint8_t data MSB
    * @param data2 uint8_t data LSB
    */
    Wire.write(data1);                    // send MSB first
    Wire.write(data2);                    // send LSB second
}

uint8_t DRV10987Firmware::checksum(uint8_t *packet, uint8_t n)
{
    /* Check transmition errors of data structure
    *
    * @param packet uint8_t[UART_PCKT_SZ] data structure to verify
    * @param n uint8_t number of bytes on <packet>
    * @return sum uint8_t calculated checksum value
    */
    // TODO: implement better data authenticity algorith
    uint32_t sum = 0;
    for (int j=0;j<n-1;j++) sum += packet[j];
    return sum & 0x00FF;
}