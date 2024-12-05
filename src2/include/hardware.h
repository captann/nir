#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>
#include <stdbool.h>

// Функции инициализации
void SystemClock_Config(void);
void GPIO_Init(void);
void I2C_Init(void);
void Timer_Init(void);

// Функции времени
uint32_t GetMillis(void);
void DelayMs(uint32_t ms);

// Определения для MPU6050
#define MPU6050_I2C_BASE I2C1_BASE
#define MPU6050_I2C_ADDR 0x68

// Определения для моторов (PWM выходы)
#define MOTOR1_PWM PWM0_BASE
#define MOTOR1_OUT PWM_OUT_0
#define MOTOR2_PWM PWM0_BASE
#define MOTOR2_OUT PWM_OUT_1
#define MOTOR3_PWM PWM0_BASE
#define MOTOR3_OUT PWM_OUT_2
#define MOTOR4_PWM PWM0_BASE
#define MOTOR4_OUT PWM_OUT_3

// Определения для LED индикации (Port F)
#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3
#define LED_PORT GPIO_PORTF_BASE

#endif // HARDWARE_H
