#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "diagnostics.h"
#include "communication.h"
#include "sensors.h"
#include "navigation.h"
#include "stabilization.h"
#include "actuators.h"
#include "gpio.h"

// Функции управления периферией
bool IsPeripheralReady(uint32_t peripheral);
void EnablePeripheral(uint32_t peripheral);

// Состояние дрона
typedef enum {
    DRONE_STATE_INIT,           // Инициализация
    DRONE_STATE_IDLE,           // Ожидание команд
    DRONE_STATE_CALIBRATING,    // Калибровка датчиков
    DRONE_STATE_ARMED,          // Готов к взлету
    DRONE_STATE_TAKEOFF,        // Взлет
    DRONE_STATE_FLYING,         // Полет
    DRONE_STATE_LANDING,        // Посадка
    DRONE_STATE_EMERGENCY       // Аварийный режим
} DroneState;

// Параметры системы
#define MAIN_LOOP_FREQ_HZ      200     // Частота основного цикла (Гц)
#define TELEMETRY_FREQ_HZ      10      // Частота отправки телеметрии (Гц)
#define DIAGNOSTIC_FREQ_HZ      1       // Частота диагностики (Гц)
#define WDT_TIMEOUT_MS         1000    // Таймаут watchdog (мс)

// Параметры полета
#define MAX_ALTITUDE           100.0f   // Максимальная высота (м)
#define MIN_BATTERY_VOLTAGE    10.5f    // Минимальное напряжение батареи (В)
#define MAX_ANGLE              45.0f    // Максимальный угол наклона (градусы)
#define MAX_YAW_RATE          90.0f    // Максимальная скорость рыскания (градусы/с)

// Параметры системы
typedef enum {
    PARAM_PID_ROLL = 0,
    PARAM_PID_PITCH,
    PARAM_PID_YAW,
    PARAM_PID_ALT
} SystemParameter;

// Тип команды
typedef enum {
    CMD_TYPE_POSITION = 0,     // Команда позиционирования
    CMD_TYPE_ATTITUDE,         // Команда управления ориентацией
    CMD_TYPE_CONFIG           // Команда конфигурации
} DroneCommandType;

// Структура команды
typedef struct {
    DroneCommandType type;
    union {
        struct {
            float x, y, z;      // Целевая позиция (м)
        } position;
        struct {
            float roll, pitch, yaw; // Целевые углы (градусы)
        } attitude;
        struct {
            SystemParameter param;
            float value;
        } config;
    } data;
} CommandPacket;

// Структура данных для телеметрии
typedef struct {
    SensorData sensorData;         // Данные с датчиков
    NavigationData navData;        // Навигационные данные
    StabilizationData stabData;    // Данные стабилизации
    DroneState state;              // Текущее состояние
    uint32_t errors;              // Флаги ошибок
} TelemetryPacket;

// Прототипы функций
void SystemInit(void);
void ProcessCommands(void);
void CheckSafety(void);
void EnterEmergencyMode(const char* reason);
void ExitEmergencyMode(void);
void UpdatePIDParameters(SystemParameter param, float value);

#endif // MAIN_H
