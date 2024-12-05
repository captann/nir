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
 * Базовые адреса GPIO портов
 */
#define GPIO_PORTA_BASE    0x40004000  // Порт A - основной порт для UART и датчиков
#define GPIO_PORTB_BASE    0x40005000  // Порт B - дополнительные периферийные устройства
#define GPIO_PORTC_BASE    0x40006000  // Порт C - отладочные выходы и LED индикация
#define GPIO_PORTD_BASE    0x40007000  // Порт D - интерфейсы для моторов
#define GPIO_PORTE_BASE    0x40024000  // Порт E - дополнительные входы/выходы
#define GPIO_PORTF_BASE    0x40025000  // Порт F - встроенные светодиоды и кнопки

/*
 * Регистры системного управления
 */
#define SYSCTL_RCC2_R     (*((volatile uint32_t *)0x400FE070))  // Регистр управления тактированием
#define SYSCTL_RIS_R      (*((volatile uint32_t *)0x400FE050))  // Регистр статуса прерываний
#define SYSCTL_RCGC2_R    (*((volatile uint32_t *)0x400FE108))  // Регистр тактирования периферии

/*
 * Параметры PID регулятора для стабилизации
 */
// Коэффициенты для канала крена (Roll)
#define ROLL_KP           2.0f    // Пропорциональный коэффициент
#define ROLL_KI           0.1f    // Интегральный коэффициент
#define ROLL_KD           0.05f   // Дифференциальный коэффициент

// Коэффициенты для канала тангажа (Pitch)
#define PITCH_KP          2.0f
#define PITCH_KI          0.1f
#define PITCH_KD          0.05f

// Коэффициенты для канала рыскания (Yaw)
#define YAW_KP            3.0f
#define YAW_KI            0.15f
#define YAW_KD           0.075f

/*
 * Пороговые значения для безопасности
 */
#define MAX_SAFE_ANGLE        45.0f   // Максимальный безопасный угол наклона (градусы)
#define MIN_BATTERY_VOLTAGE   10.5f   // Минимальное напряжение батареи (вольты)
#define MAX_ALTITUDE         100.0f   // Максимальная разрешенная высота (метры)
#define CRITICAL_TEMP         60.0f   // Критическая температура контроллера (°C)

/*
 * Константы для моторов
 */
#define MOTOR_PWM_FREQ        400     // Частота ШИМ для моторов (Гц)
#define MOTOR_MIN_PULSE       1000    // Минимальная длительность импульса (мкс)
#define MOTOR_MAX_PULSE       2000    // Максимальная длительность импульса (мкс)
#define MOTOR_ARM_PULSE       1100    // Импульс для армирования моторов

/*
 * Коммуникационные константы
 */
#define UART_BAUD_RATE       115200   // Скорость UART (бод)
#define PACKET_START_BYTE    0xAA     // Стартовый байт пакета
#define PACKET_END_BYTE      0x55     // Конечный байт пакета
#define MAX_PACKET_SIZE      64       // Максимальный размер пакета данных

/*
 * Калибровочные константы для датчиков
 */
#define ACCEL_SCALE_FACTOR   0.000244f    // Коэффициент масштабирования акселерометра (g/LSB)
#define GYRO_SCALE_FACTOR    0.0175f      // Коэффициент масштабирования гироскопа (град/с/LSB)
#define MAG_SCALE_FACTOR     0.15f        // Коэффициент масштабирования магнетометра (мГаусс/LSB)

#endif // CONSTANTS_H
