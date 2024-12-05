#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "../include/hardware.h"

// Системное время в миллисекундах
static volatile uint32_t system_ticks = 0;

// Обработчик прерывания системного таймера
void SysTick_Handler(void) {
    system_ticks++;
}

// Получение текущего времени в миллисекундах
uint32_t GetMillis(void) {
    return system_ticks;
}

// Задержка в миллисекундах
void DelayMs(uint32_t ms) {
    SysCtlDelay(SysCtlClockGet() / 3000 * ms);
}

// Конфигурация системного тактирования
void SystemClock_Config(void) {
    // Установка частоты системы на 80 МГц используя PLL
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    // Настройка SysTick для отсчета миллисекунд
    SysTickPeriodSet(SysCtlClockGet() / 1000);
    SysTickIntEnable();
    SysTickEnable();
}

// Инициализация GPIO
void GPIO_Init(void) {
    // Включение тактирования портов
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    
    // Ожидание готовности портов
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA) ||
          !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB) ||
          !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC) ||
          !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD) ||
          !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE) ||
          !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
}

// Инициализация I2C для MPU6050
void I2C_Init(void) {
    // Включение периферии I2C1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    // Настройка пинов I2C1 (PA6 = SCL, PA7 = SDA)
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    
    // Инициализация I2C1 на 400 кГц
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true);
}

// Инициализация PWM для управления моторами
void Timer_Init(void) {
    // Включение периферии PWM0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    
    // Настройка пинов PWM для моторов (PB6, PB7, PB4, PB5)
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5);
    
    // Настройка PWM генераторов
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    // Установка периода PWM (20 кГц)
    uint32_t pwmClock = SysCtlClockGet() / 64;
    uint32_t loadVal = (pwmClock / 20000) - 1;
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, loadVal);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, loadVal);
    
    // Запуск PWM генераторов
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    
    // Включение выходов PWM
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
}
