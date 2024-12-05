#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <stdint.h>
#include "constants.h"

/*
 * Перечисление состояний дрона
 */
typedef enum {
    DRONE_STATE_INIT,        // Инициализация систем
    DRONE_STATE_IDLE,        // Ожидание команд
    DRONE_STATE_ARMED,       // Моторы активированы, готов к взлету
    DRONE_STATE_FLYING,      // В полете
    DRONE_STATE_LANDING,     // Выполняет посадку
    DRONE_STATE_EMERGENCY,   // Аварийный режим
    DRONE_STATE_ERROR        // Ошибка системы
} DroneState;

/*
 * Структура для хранения данных с датчиков
 */
typedef struct {
    struct {
        float x;            // Ускорение по оси X (g)
        float y;            // Ускорение по оси Y (g)
        float z;            // Ускорение по оси Z (g)
        uint32_t timestamp; // Временная метка измерения
    } accelerometer;

    struct {
        float x;            // Угловая скорость по оси X (град/с)
        float y;            // Угловая скорость по оси Y (град/с)
        float z;            // Угловая скорость по оси Z (град/с)
        uint32_t timestamp; // Временная метка измерения
    } gyroscope;

    struct {
        float x;            // Магнитное поле по оси X (мГаусс)
        float y;            // Магнитное поле по оси Y (мГаусс)
        float z;            // Магнитное поле по оси Z (мГаусс)
        uint32_t timestamp; // Временная метка измерения
    } magnetometer;

    float barometer_altitude;    // Высота по барометру (метры)
    float temperature;          // Температура (градусы Цельсия)
    float battery_voltage;      // Напряжение батареи (вольты)
} SensorData;

/*
 * Структура для ориентации дрона
 */
typedef struct {
    float roll;            // Крен (градусы)
    float pitch;           // Тангаж (градусы)
    float yaw;            // Рыскание (градусы)
    float altitude;       // Высота (метры)
} Attitude;

/*
 * Структура для PID регулятора
 */
typedef struct {
    float kp;             // Пропорциональный коэффициент
    float ki;             // Интегральный коэффициент
    float kd;             // Дифференциальный коэффициент
    float setpoint;       // Целевое значение
    float integral;       // Интегральная составляющая
    float prev_error;     // Предыдущая ошибка
    float output;         // Выход регулятора
} PIDController;

/*
 * Структура для управления моторами
 */
typedef struct {
    uint16_t front_left;   // Значение ШИМ для переднего левого мотора
    uint16_t front_right;  // Значение ШИМ для переднего правого мотора
    uint16_t rear_left;    // Значение ШИМ для заднего левого мотора
    uint16_t rear_right;   // Значение ШИМ для заднего правого мотора
} MotorControl;

/*
 * Структура для телеметрии
 */
typedef struct {
    DroneState state;          // Текущее состояние дрона
    Attitude attitude;         // Текущая ориентация
    SensorData sensors;        // Данные с датчиков
    MotorControl motors;       // Значения моторов
    uint32_t flight_time;      // Время полета (мс)
    uint8_t errors;           // Битовое поле ошибок
} Telemetry;

/*
 * Структура для команд управления
 */
typedef struct {
    float throttle;        // Газ (0.0 - 1.0)
    float roll;           // Заданный крен (-1.0 - 1.0)
    float pitch;          // Заданный тангаж (-1.0 - 1.0)
    float yaw;            // Заданное рыскание (-1.0 - 1.0)
    uint8_t aux_flags;    // Дополнительные флаги управления
} ControlCommand;

/*
 * Структура для конфигурации дрона
 */
typedef struct {
    PIDController roll_pid;    // PID для крена
    PIDController pitch_pid;   // PID для тангажа
    PIDController yaw_pid;     // PID для рыскания
    float max_angle;           // Максимальный угол наклона
    float max_altitude;        // Максимальная высота
    uint16_t failsafe_timeout; // Таймаут потери связи (мс)
} DroneConfig;

#endif // STRUCTURES_H
