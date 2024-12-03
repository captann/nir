#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdint.h>
#include <stdbool.h>

// Определение количества моторов для квадрокоптера
#define MOTOR_COUNT 4

// Определение индексов моторов
#define MOTOR_FRONT_LEFT  0
#define MOTOR_FRONT_RIGHT 1
#define MOTOR_BACK_LEFT   2
#define MOTOR_BACK_RIGHT  3

// Определение констант для PWM
#define PWM_MIN_VALUE     0    // Минимальное значение PWM (мотор выключен)
#define PWM_MAX_VALUE     1000 // Максимальное значение PWM (полная мощность)
#define PWM_IDLE_VALUE    100  // Значение PWM на холостом ходу

// Структура для хранения состояния моторов
typedef struct {
    uint16_t pwm_value;     // Текущее значение PWM (0-1000)
    bool is_running;        // Флаг работы мотора
    float temperature;      // Эмуляция температуры мотора
    uint32_t runtime;       // Время работы мотора в миллисекундах
} MotorState;

// Структура для управления всеми моторами
typedef struct {
    MotorState motors[MOTOR_COUNT];
    bool all_motors_enabled;
    uint16_t global_throttle;  // Общий уровень газа (0-1000)
} ActuatorsState;

// Функции инициализации и управления
void Actuators_Init(void);
void Actuators_DeInit(void);

// Основные функции управления моторами
void Actuators_SetMotorPWM(uint8_t motor_index, uint16_t pwm_value);
void Actuators_SetAllMotorsPWM(uint16_t pwm_value);
void Actuators_SetAllMotors(float power_level);  // Устанавливает одинаковую мощность для всех моторов
uint16_t Actuators_GetMotorPWM(uint8_t motor_index);

// Функции управления состоянием
void Actuators_EnableMotor(uint8_t motor_index);
void Actuators_DisableMotor(uint8_t motor_index);
void Actuators_EnableAllMotors(void);
void Actuators_DisableAllMotors(void);
bool Actuators_IsMotorEnabled(uint8_t motor_index);

// Функции управления полетом
void Actuators_SetThrottle(uint16_t throttle);
void Actuators_SetYaw(int16_t yaw);
void Actuators_SetPitch(int16_t pitch);
void Actuators_SetRoll(int16_t roll);

// Функции для получения состояния
void Actuators_GetState(ActuatorsState* state);
float Actuators_GetMotorTemperature(uint8_t motor_index);
uint32_t Actuators_GetMotorRuntime(uint8_t motor_index);

// Функции эмуляции различных ситуаций
void Actuators_EmulateHover(void);
void Actuators_EmulateAscent(void);
void Actuators_EmulateDescent(void);
void Actuators_EmulateRotation(int16_t yaw_rate);
void Actuators_EmulateForwardFlight(uint16_t speed);

// Функции безопасности
void Actuators_EmergencyStop(void);
bool Actuators_IsEmergencyStopActive(void);
void Actuators_ResetEmergencyStop(void);

#endif // ACTUATORS_H
