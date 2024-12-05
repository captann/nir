#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <stdint.h>

/*
 * Системные константы для тактирования
 */

// Частота основного тактового генератора в Гц
#define SYSTEM_CLOCK_FREQ      80000000UL  // 80 МГц - основная частота системы

// Периоды для различных задач (в миллисекундах)
#define TELEMETRY_UPDATE_PERIOD    100     // Период обновления телеметрии (10 Гц)
#define SENSOR_READ_PERIOD         10      // Период опроса датчиков (100 Гц)
#define CONTROL_LOOP_PERIOD        5       // Период цикла управления (200 Гц)
#define WATCHDOG_UPDATE_PERIOD     500     // Период обновления сторожевого таймера

/*
 * Константы для MPU6050
 */
#define MPU6050_GYRO_SCALE    131.0f      // Для диапазона ±250°/s
#define MPU6050_ACCEL_SCALE   16384.0f    // Для диапазона ±2g

/*
 * Константы для моторов
 */
#define MOTOR_PWM_FREQ        20000       // Частота PWM (20 кГц)
#define MOTOR_MIN_PULSE       1000        // Минимальное значение PWM
#define MOTOR_MAX_PULSE       2000        // Максимальное значение PWM
#define MOTOR_ARM_PULSE       1100        // Значение PWM для армирования моторов

/*
 * Параметры PID регулятора для стабилизации
 */
// Коэффициенты для канала крена (Roll)
#define ROLL_KP              2.0f    // Пропорциональный коэффициент
#define ROLL_KI              0.1f    // Интегральный коэффициент
#define ROLL_KD              0.05f   // Дифференциальный коэффициент

// Коэффициенты для канала тангажа (Pitch)
#define PITCH_KP             2.0f    // Пропорциональный коэффициент
#define PITCH_KI             0.1f    // Интегральный коэффициент
#define PITCH_KD             0.05f   // Дифференциальный коэффициент

// Коэффициенты для канала рыскания (Yaw)
#define YAW_KP               4.0f    // Пропорциональный коэффициент
#define YAW_KI               0.05f   // Интегральный коэффициент
#define YAW_KD              0.0f    // Дифференциальный коэффициент

/*
 * Пределы управления
 */
#define MAX_ROLL_ANGLE       45.0f   // Максимальный угол крена (градусы)
#define MAX_PITCH_ANGLE      45.0f   // Максимальный угол тангажа (градусы)
#define MAX_YAW_RATE         180.0f  // Максимальная скорость рыскания (градусы/с)

/*
 * Параметры фильтрации
 */
#define COMP_FILTER_ALPHA    0.96f   // Коэффициент комплементарного фильтра

/*
 * Параметры безопасности
 */
#define MAX_SAFE_ANGLE       60.0f   // Максимальный безопасный угол наклона
#define LOW_VOLTAGE_THRESHOLD 10.5f   // Порог низкого напряжения батареи
#define MAX_ALTITUDE         100.0f   // Максимальная высота (метры)

#endif // CONSTANTS_H
