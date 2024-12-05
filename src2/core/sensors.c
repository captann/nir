#include "../include/hardware.h"
#include "../include/constants.h"
#include "../include/structures.h"
#include "driverlib/i2c.h"
#include <math.h>

// Регистры MPU6050
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACCEL_XOUT_H  0x3B

static SensorData sensor_data = {0};

// Вспомогательные функции для работы с I2C
static void I2C_WriteByte(uint8_t reg, uint8_t data) {
    I2CMasterSlaveAddrSet(MPU6050_I2C_BASE, MPU6050_I2C_ADDR, false);
    I2CMasterDataPut(MPU6050_I2C_BASE, reg);
    I2CMasterControl(MPU6050_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(MPU6050_I2C_BASE));
    
    I2CMasterDataPut(MPU6050_I2C_BASE, data);
    I2CMasterControl(MPU6050_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(MPU6050_I2C_BASE));
}

static uint8_t I2C_ReadByte(uint8_t reg) {
    I2CMasterSlaveAddrSet(MPU6050_I2C_BASE, MPU6050_I2C_ADDR, false);
    I2CMasterDataPut(MPU6050_I2C_BASE, reg);
    I2CMasterControl(MPU6050_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(MPU6050_I2C_BASE));
    
    I2CMasterSlaveAddrSet(MPU6050_I2C_BASE, MPU6050_I2C_ADDR, true);
    I2CMasterControl(MPU6050_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(MPU6050_I2C_BASE));
    
    return I2CMasterDataGet(MPU6050_I2C_BASE);
}

// Инициализация MPU6050
void Sensors_Init(void) {
    // Сброс MPU6050
    I2C_WriteByte(MPU6050_PWR_MGMT_1, 0x80);
    DelayMs(100);
    
    // Включение и настройка
    I2C_WriteByte(MPU6050_PWR_MGMT_1, 0x00);    // Выход из спящего режима
    I2C_WriteByte(MPU6050_SMPLRT_DIV, 0x07);    // Частота сэмплирования = 1kHz
    I2C_WriteByte(MPU6050_CONFIG, 0x06);        // Фильтр низких частот
    I2C_WriteByte(MPU6050_GYRO_CONFIG, 0x08);   // Диапазон гироскопа ±500°/s
    I2C_WriteByte(MPU6050_ACCEL_CONFIG, 0x00);  // Диапазон акселерометра ±2g
}

// Чтение данных с MPU6050
void Sensors_Read(void) {
    uint8_t buffer[14];
    int16_t raw_data[7];
    
    // Чтение всех данных за раз
    for(int i = 0; i < 14; i++) {
        buffer[i] = I2C_ReadByte(MPU6050_ACCEL_XOUT_H + i);
    }
    
    // Преобразование байтов в 16-битные значения
    for(int i = 0; i < 7; i++) {
        raw_data[i] = (buffer[i*2] << 8) | buffer[i*2+1];
    }
    
    // Преобразование в физические величины
    sensor_data.accel_x = (float)raw_data[0] / MPU6050_ACCEL_SCALE;
    sensor_data.accel_y = (float)raw_data[1] / MPU6050_ACCEL_SCALE;
    sensor_data.accel_z = (float)raw_data[2] / MPU6050_ACCEL_SCALE;
    sensor_data.gyro_x = (float)raw_data[4] / MPU6050_GYRO_SCALE;
    sensor_data.gyro_y = (float)raw_data[5] / MPU6050_GYRO_SCALE;
    sensor_data.gyro_z = (float)raw_data[6] / MPU6050_GYRO_SCALE;
    
    // Расчет углов (простой комплементарный фильтр)
    float accel_roll = atan2f(sensor_data.accel_y, sensor_data.accel_z) * 57.3f;
    float accel_pitch = atan2f(-sensor_data.accel_x, sqrtf(sensor_data.accel_y * sensor_data.accel_y + 
                              sensor_data.accel_z * sensor_data.accel_z)) * 57.3f;
    
    static uint32_t last_update = 0;
    uint32_t now = GetMillis();
    float dt = (now - last_update) / 1000.0f;
    last_update = now;
    
    sensor_data.roll = COMP_FILTER_ALPHA * (sensor_data.roll + sensor_data.gyro_x * dt) + 
                      (1.0f - COMP_FILTER_ALPHA) * accel_roll;
    sensor_data.pitch = COMP_FILTER_ALPHA * (sensor_data.pitch + sensor_data.gyro_y * dt) + 
                       (1.0f - COMP_FILTER_ALPHA) * accel_pitch;
    sensor_data.yaw += sensor_data.gyro_z * dt;
    
    // Чтение температуры из MPU6050
    int16_t temp_raw = (buffer[6] << 8) | buffer[7];
    sensor_data.temperature = (float)temp_raw / 340.0f + 36.53f;
    
    // TODO: Реализовать измерение напряжения через АЦП
    sensor_data.battery_voltage = 12.0f;
    
    // TODO: Добавить чтение барометра
    sensor_data.barometer_altitude = 0.0f;
}

// Получение данных с датчиков
const SensorData* Sensors_GetData(void) {
    return &sensor_data;
}
