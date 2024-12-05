#include "../include/hardware.h"
#include "../include/constants.h"
#include "inc/hw_memmap.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

// Структура для хранения значений PWM моторов
static struct {
    uint32_t values[4];
    bool armed;
} motors = {0};

static uint32_t pwm_period;

// Преобразование значения 1000-2000 в значение PWM
static uint32_t convert_to_pwm(uint32_t pulse) {
    if(pulse < MOTOR_MIN_PULSE) pulse = MOTOR_MIN_PULSE;
    if(pulse > MOTOR_MAX_PULSE) pulse = MOTOR_MAX_PULSE;
    return (pulse - MOTOR_MIN_PULSE) * pwm_period / (MOTOR_MAX_PULSE - MOTOR_MIN_PULSE);
}

// Установка значения PWM для конкретного мотора
static void set_motor_pwm(uint8_t motor, uint32_t pulse) {
    uint32_t pwm_value = convert_to_pwm(pulse);
    motors.values[motor] = pwm_value;
    
    switch(motor) {
        case 0:
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm_value);
            break;
        case 1:
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm_value);
            break;
        case 2:
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm_value);
            break;
        case 3:
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwm_value);
            break;
    }
}

void Motors_Init(void) {
    // Включение PWM0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    // Ждем готовности периферии
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0) ||
          !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    
    // Настройка пинов для PWM
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5);
    
    // Настройка PWM
    uint32_t pwm_clock = SysCtlClockGet() / 64;
    pwm_period = (pwm_clock / MOTOR_PWM_FREQ) - 1;
    
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwm_period);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwm_period);
    
    // Запуск PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    
    // Установка минимальных значений
    for(uint8_t i = 0; i < 4; i++) {
        set_motor_pwm(i, MOTOR_MIN_PULSE);
    }
    
    // Включение выходов
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | 
                            PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    
    motors.armed = false;
}

void Motors_SetPWM(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4) {
    if(!motors.armed) return;
    
    set_motor_pwm(0, m1);
    set_motor_pwm(1, m2);
    set_motor_pwm(2, m3);
    set_motor_pwm(3, m4);
}

void Motors_Arm(void) {
    motors.armed = true;
    Motors_SetPWM(MOTOR_ARM_PULSE, MOTOR_ARM_PULSE, 
                 MOTOR_ARM_PULSE, MOTOR_ARM_PULSE);
}

void Motors_Disarm(void) {
    motors.armed = false;
    for(uint8_t i = 0; i < 4; i++) {
        set_motor_pwm(i, MOTOR_MIN_PULSE);
    }
}

bool Motors_IsArmed(void) {
    return motors.armed;
}

void Motors_EmergencyStop(void) {
    motors.armed = false;
    for(uint8_t i = 0; i < 4; i++) {
        set_motor_pwm(i, MOTOR_MIN_PULSE);
    }
}
