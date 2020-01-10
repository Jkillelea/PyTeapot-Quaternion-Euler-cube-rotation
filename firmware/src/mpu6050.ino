#include <Arduino.h>
#include <Wire.h>
// #include <assert.h>
#include <HardwareSerial.h>
#include "mpu6050_registers.h"

HardwareSerial Serial3(USART3);

inline void write8(uint8_t reg, uint8_t val) {
    uint8_t buffer[2] = {reg, val};
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(buffer, 2);
    Wire.endTransmission();
}

void setup() {
    delay(500);
    Wire.begin();
    Serial3.begin(115200);
    Serial3.println("SUUPPPPPPPPPPPPPPPPPPpp");

    // start device, pll on z axis reference
    write8(RegPowerManagment1, (3 << 0) | (0 << 6));

    /* +/- 2g */
    write8(RegAccelConfig, 0 << RegAccelConfigScale);
    /* +/- 250 deg/s */
    write8(RegGyroConfig,  0 << RegGyroConfigScale);
}

void loop () {

    uint8_t accel_regs[] = {RegAccelX, RegAccelY, RegAccelZ};
    uint8_t gyro_regs[]  = {RegGyroX, RegGyroY, RegGyroZ};
    int16_t accel_data[] = {0, 0, 0};
    int16_t gyro_data[]  = {0, 0, 0};
    float accel_scale    = 2*9.81;
    float gyro_scale     = 250;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(RegAccelX);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.requestFrom(MPU_ADDR, 3*2);
    // while(!Wire.available());
    for (int i = 0; i < 3; i++) {
        accel_data[i] = (int16_t) ((Wire.read() << 8) | Wire.read());
    }
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(RegGyroX);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.requestFrom(MPU_ADDR, 3*2);
    // while(!Wire.available());
    for (int i = 0; i < 3; i++) {
        gyro_data[i] = (int16_t) ((Wire.read() << 8) | Wire.read());
    }
    Wire.endTransmission();


    // for (int i = 0; i < 3; i++) {
    //     accel_data[i] = (int16_t) read16(accel_regs[i]);
    // }
    // for (int i = 0; i < 3; i++) {
    //     gyro_data[i] = (int16_t) read16(gyro_regs[i]);
    // }

    for (int i = 0; i < 3; i++) {
        Serial3.print(accel_scale * ((float) accel_data[i]) / INT16_MAX);
        Serial3.print(" ");
    }

    for (int i = 0; i < 3; i++) {
        Serial3.print(gyro_scale * ((float) gyro_data[i]) / INT16_MAX);
        Serial3.print(" ");
    }

    Serial3.println("");

    delay(1); // convert to ms
}
