#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#pragma pack(push, 4)  // Выравнивание по 4 байта для оптимизации

// Структура для данных акселерометра и гироскопа (MPU6050)
typedef struct {
    // Акселерометр (в единицах g)
    float accel_x;    // Ускорение по оси X
    float accel_y;    // Ускорение по оси Y
    float accel_z;    // Ускорение по оси Z
    
    // Гироскоп (в градусах в секунду)
    float gyro_x;     // Угловая скорость вокруг оси X (крен)
    float gyro_y;     // Угловая скорость вокруг оси Y (тангаж)
    float gyro_z;     // Угловая скорость вокруг оси Z (рыскание)
    
    // Температура (в градусах Цельсия)
    float temperature;
} MPU6050_Data;

// Структура для данных барометра (BMP280)
typedef struct {
    float pressure;    // Давление в гПа (гектопаскалях)
    float altitude;    // Высота в метрах
    float temperature; // Температура в градусах Цельсия
} BMP280_Data;

// Структура для углов ориентации
typedef struct {
    float roll;      // Крен (в градусах)
    float pitch;     // Тангаж (в градусах)
    float yaw;       // Рыскание (в градусах)
} Orientation;

// Общая структура данных сенсоров
typedef struct {
    MPU6050_Data imu;
    BMP280_Data baro;
    bool isCalibrated;
} SensorData;

#pragma pack(pop)  // Восстанавливаем стандартное выравнивание

// Основные функции работы с датчиками
void Sensors_Init(void);
bool Sensors_Update(void);
bool Sensors_IsCalibrated(void);
void Sensors_GetAllData(SensorData* data);
void Sensors_GetBaroData(BMP280_Data* data);

#endif // SENSORS_H
