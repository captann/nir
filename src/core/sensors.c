#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "TM4C123GH6PM.h"
#include "../include/sensors.h"
#include "../include/diagnostics.h"

// Эмуляция данных с датчиков
static struct {
    bool mpu6050_initialized;
    bool bmp280_initialized;
    float current_altitude;
    float rotation_angle;
} sensor_state = {
    .mpu6050_initialized = false,
    .bmp280_initialized = false,
    .current_altitude = 0.0f,
    .rotation_angle = 0.0f
};

// Внутренние переменные для эмуляции данных датчиков
static MPU6050_Data mpu6050_data = {0};
static BMP280_Data bmp280_data = {0};

// Переменные для эмуляции движения
static float vertical_speed = 0.0f;

void MPU6050_Init(void) {
    // Инициализация значений по умолчанию
    mpu6050_data.accel_x = 0.0f;
    mpu6050_data.accel_y = 0.0f;
    mpu6050_data.accel_z = 1.0f;  // 1g в состоянии покоя
    mpu6050_data.gyro_x = 0.0f;
    mpu6050_data.gyro_y = 0.0f;
    mpu6050_data.gyro_z = 0.0f;
    mpu6050_data.temperature = 25.0f;  // Комнатная температура
    
    sensor_state.mpu6050_initialized = true;
}

void BMP280_Init(void) {
    // Инициализация значений по умолчанию
    bmp280_data.pressure = 1013.25f;  // Стандартное атмосферное давление (гПа)
    bmp280_data.altitude = 0.0f;
    bmp280_data.temperature = 25.0f;  // Комнатная температура
    
    sensor_state.bmp280_initialized = true;
}

void MPU6050_Read(MPU6050_Data* data) {
    if (!sensor_state.mpu6050_initialized || data == NULL) {
        return;
    }
    
    // Копируем текущие данные
    *data = mpu6050_data;
}

void BMP280_Read(BMP280_Data* data) {
    if (!sensor_state.bmp280_initialized || data == NULL) {
        return;
    }
    
    // Копируем текущие данные
    *data = bmp280_data;
}

void MPU6050_Calibrate(void) {
    // В эмуляции просто сбрасываем значения к исходным
    mpu6050_data.accel_x = 0.0f;
    mpu6050_data.accel_y = 0.0f;
    mpu6050_data.accel_z = 1.0f;
    mpu6050_data.gyro_x = 0.0f;
    mpu6050_data.gyro_y = 0.0f;
    mpu6050_data.gyro_z = 0.0f;
}

void BMP280_Calibrate(void) {
    // В эмуляции устанавливаем стандартные значения на уровне моря
    bmp280_data.pressure = 1013.25f;
    bmp280_data.altitude = 0.0f;
}

// Функции эмуляции различных ситуаций полета
void Sensors_EmulateStableHover(void) {
    // Эмулируем стабильное зависание
    mpu6050_data.accel_x = 0.0f;
    mpu6050_data.accel_y = 0.0f;
    mpu6050_data.accel_z = 1.0f;
    mpu6050_data.gyro_x = 0.0f;
    mpu6050_data.gyro_y = 0.0f;
    mpu6050_data.gyro_z = 0.0f;
    
    // Небольшие колебания высоты
    sensor_state.current_altitude += (float)((rand() % 10) - 5) / 100.0f;
    bmp280_data.altitude = sensor_state.current_altitude;
    bmp280_data.pressure = 1013.25f - (sensor_state.current_altitude * 0.12f); // Примерное изменение давления с высотой
}

void Sensors_EmulateAscent(void) {
    // Эмулируем подъём
    vertical_speed = 2.0f; // м/с
    sensor_state.current_altitude += vertical_speed * 0.1f; // Предполагаем вызов каждые 100мс
    
    mpu6050_data.accel_z = 1.2f; // Повышенная нагрузка при наборе высоты
    bmp280_data.altitude = sensor_state.current_altitude;
    bmp280_data.pressure = 1013.25f - (sensor_state.current_altitude * 0.12f);
}

void Sensors_EmulateDescent(void) {
    // Эмулируем снижение
    vertical_speed = -1.5f; // м/с
    sensor_state.current_altitude += vertical_speed * 0.1f;
    if (sensor_state.current_altitude < 0.0f) sensor_state.current_altitude = 0.0f;
    
    mpu6050_data.accel_z = 0.8f; // Пониженная нагрузка при снижении
    bmp280_data.altitude = sensor_state.current_altitude;
    bmp280_data.pressure = 1013.25f - (sensor_state.current_altitude * 0.12f);
}

void Sensors_EmulateRotation(void) {
    // Эмулируем поворот вокруг вертикальной оси
    sensor_state.rotation_angle += 45.0f * 0.1f; // 45 градусов в секунду
    if (sensor_state.rotation_angle >= 360.0f) sensor_state.rotation_angle -= 360.0f;
    
    mpu6050_data.gyro_z = 45.0f;
    mpu6050_data.accel_x = (float)sin(sensor_state.rotation_angle * 3.14159f / 180.0f) * 0.1f;
    mpu6050_data.accel_y = (float)cos(sensor_state.rotation_angle * 3.14159f / 180.0f) * 0.1f;
}

void Sensors_EmulateForwardMovement(void) {
    // Эмулируем движение вперед
    mpu6050_data.accel_x = 0.2f;
    mpu6050_data.gyro_y = -5.0f; // Небольшой наклон вперед
    
    // Небольшие колебания высоты при движении
    sensor_state.current_altitude += (float)((rand() % 10) - 5) / 100.0f;
    bmp280_data.altitude = sensor_state.current_altitude;
    bmp280_data.pressure = 1013.25f - (sensor_state.current_altitude * 0.12f);
}

void Sensors_GetIMUData(MPU6050_Data* data) {
    if (data == NULL) return;

    *data = mpu6050_data;
}

void Sensors_GetOrientation(Orientation* orientation) {
    if (orientation == NULL) return;

    MPU6050_Data imu;
    Sensors_GetIMUData(&imu);

    // Вычисляем углы ориентации из данных акселерометра и гироскопа
    // Используем комплементарный фильтр для объединения данных
    static float filteredRoll = 0.0f;
    static float filteredPitch = 0.0f;
    static float filteredYaw = 0.0f;

    // Вычисляем углы из акселерометра
    float accelRoll = atan2f(imu.accel_y, imu.accel_z) * 180.0f / 3.14159f;
    float accelPitch = atan2f(-imu.accel_x, sqrtf(imu.accel_y * imu.accel_y + imu.accel_z * imu.accel_z)) * 180.0f / 3.14159f;

    // Интегрируем данные гироскопа (предполагаем, что Update вызывается каждые 10мс)
    float dt = 0.01f;
    filteredRoll = 0.96f * (filteredRoll + imu.gyro_x * dt) + 0.04f * accelRoll;
    filteredPitch = 0.96f * (filteredPitch + imu.gyro_y * dt) + 0.04f * accelPitch;
    filteredYaw += imu.gyro_z * dt;

    // Нормализуем yaw в диапазоне 0-360 градусов
    while (filteredYaw >= 360.0f) filteredYaw -= 360.0f;
    while (filteredYaw < 0.0f) filteredYaw += 360.0f;

    orientation->roll = filteredRoll;
    orientation->pitch = filteredPitch;
    orientation->yaw = filteredYaw;
}

// Заглушки для сенсоров
void Sensors_GetAllData(SensorData* data) {
    if(data) {
        data->imu.accel_x = 0.0f;
        data->imu.accel_y = 0.0f;
        data->imu.accel_z = 1.0f;
        data->imu.gyro_x = 0.0f;
        data->imu.gyro_y = 0.0f;
        data->imu.gyro_z = 0.0f;
        data->baro.pressure = 1013.25f;
        data->baro.altitude = 0.0f;
        data->isCalibrated = true;
    }
}

void Sensors_GetBaroData(BMP280_Data* data) {
    if(data) {
        data->pressure = 1013.25f;
        data->altitude = 0.0f;
        data->temperature = 25.0f;
    }
}

void Sensors_Update(void) {
    // Пустая заглушка
}
