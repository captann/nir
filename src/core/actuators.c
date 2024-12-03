#include <string.h>
#include "../include/actuators.h"

// Внутреннее состояние модуля
static ActuatorsState actuators_state = {0};
static bool emergency_stop_active = false;
static uint32_t system_time_ms = 0; // Эмуляция системного времени

// Вспомогательные функции
static void UpdateMotorTemperatures(void);
static void UpdateMotorRuntimes(void);
static void ClampPWMValue(uint16_t* pwm_value);

void Actuators_Init(void) {
    // Инициализация состояния всех моторов
    memset(&actuators_state, 0, sizeof(ActuatorsState));
    
    // Установка начальных значений температуры
    for(int i = 0; i < MOTOR_COUNT; i++) {
        actuators_state.motors[i].temperature = 25.0f; // Комнатная температура
    }
    
    emergency_stop_active = false;
}

void Actuators_DeInit(void) {
    // Останавливаем все моторы
    Actuators_DisableAllMotors();
}

void Actuators_SetMotorPWM(uint8_t motor_index, uint16_t pwm_value) {
    if (motor_index >= MOTOR_COUNT || emergency_stop_active) {
        return;
    }
    
    ClampPWMValue(&pwm_value);
    actuators_state.motors[motor_index].pwm_value = pwm_value;
    actuators_state.motors[motor_index].is_running = (pwm_value > (uint16_t)PWM_MIN_VALUE);
}

void Actuators_SetAllMotorsPWM(uint16_t pwm_value) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
        Actuators_SetMotorPWM(i, pwm_value);
    }
}

uint16_t Actuators_GetMotorPWM(uint8_t motor_index) {
    if (motor_index >= MOTOR_COUNT) {
        return 0;
    }
    return actuators_state.motors[motor_index].pwm_value;
}

void Actuators_EnableMotor(uint8_t motor_index) {
    if (motor_index >= MOTOR_COUNT || emergency_stop_active) {
        return;
    }
    actuators_state.motors[motor_index].is_running = true;
}

void Actuators_DisableMotor(uint8_t motor_index) {
    if (motor_index >= MOTOR_COUNT) {
        return;
    }
    actuators_state.motors[motor_index].is_running = false;
    actuators_state.motors[motor_index].pwm_value = PWM_MIN_VALUE;
}

void Actuators_EnableAllMotors(void) {
    if (emergency_stop_active) {
        return;
    }
    for(int i = 0; i < MOTOR_COUNT; i++) {
        Actuators_EnableMotor(i);
    }
    actuators_state.all_motors_enabled = true;
}

void Actuators_DisableAllMotors(void) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
        Actuators_DisableMotor(i);
    }
    actuators_state.all_motors_enabled = false;
}

bool Actuators_IsMotorEnabled(uint8_t motor_index) {
    if (motor_index >= MOTOR_COUNT) {
        return false;
    }
    return actuators_state.motors[motor_index].is_running;
}

// Функции управления полетом
void Actuators_SetThrottle(uint16_t throttle) {
    if (emergency_stop_active) return;
    
    uint16_t pwm_value = (uint16_t)((throttle * (PWM_MAX_VALUE - PWM_MIN_VALUE)) / 100U + PWM_MIN_VALUE);
    Actuators_SetAllMotorsPWM(pwm_value);
    actuators_state.global_throttle = pwm_value;
}

void Actuators_SetYaw(int16_t yaw) {
    if (emergency_stop_active) return;
    
    // Для поворота вокруг вертикальной оси:
    // Увеличиваем мощность на диагональных моторах
    uint16_t base_pwm = actuators_state.global_throttle;
    int32_t pwm_delta = ((int32_t)yaw * (PWM_MAX_VALUE - PWM_MIN_VALUE)) / 100;
    
    Actuators_SetMotorPWM(MOTOR_FRONT_LEFT,  (uint16_t)(base_pwm + pwm_delta));
    Actuators_SetMotorPWM(MOTOR_BACK_RIGHT,  (uint16_t)(base_pwm + pwm_delta));
    Actuators_SetMotorPWM(MOTOR_FRONT_RIGHT, (uint16_t)(base_pwm - pwm_delta));
    Actuators_SetMotorPWM(MOTOR_BACK_LEFT,   (uint16_t)(base_pwm - pwm_delta));
}

void Actuators_SetPitch(int16_t pitch) {
    if (emergency_stop_active) return;
    
    // Для наклона вперед/назад:
    // Увеличиваем/уменьшаем мощность передних/задних моторов
    uint16_t base_pwm = actuators_state.global_throttle;
    int16_t pitch_pwm = pitch / 2;
    
    Actuators_SetMotorPWM(MOTOR_FRONT_LEFT,  (uint16_t)(base_pwm - pitch_pwm));
    Actuators_SetMotorPWM(MOTOR_FRONT_RIGHT, (uint16_t)(base_pwm - pitch_pwm));
    Actuators_SetMotorPWM(MOTOR_BACK_LEFT,   (uint16_t)(base_pwm + pitch_pwm));
    Actuators_SetMotorPWM(MOTOR_BACK_RIGHT,  (uint16_t)(base_pwm + pitch_pwm));
}

void Actuators_SetRoll(int16_t roll) {
    if (emergency_stop_active) return;
    
    // Для наклона влево/вправо:
    // Увеличиваем/уменьшаем мощность левых/правых моторов
    uint16_t base_pwm = actuators_state.global_throttle;
    int16_t roll_pwm = roll / 2;
    
    Actuators_SetMotorPWM(MOTOR_FRONT_LEFT,  (uint16_t)(base_pwm + roll_pwm));
    Actuators_SetMotorPWM(MOTOR_BACK_LEFT,   (uint16_t)(base_pwm + roll_pwm));
    Actuators_SetMotorPWM(MOTOR_FRONT_RIGHT, (uint16_t)(base_pwm - roll_pwm));
    Actuators_SetMotorPWM(MOTOR_BACK_RIGHT,  (uint16_t)(base_pwm - roll_pwm));
}

// Функции для получения состояния
void Actuators_GetState(ActuatorsState* state) {
    if (state != NULL) {
        *state = actuators_state;
    }
}

float Actuators_GetMotorTemperature(uint8_t motor_index) {
    if (motor_index >= MOTOR_COUNT) {
        return 0.0f;
    }
    return actuators_state.motors[motor_index].temperature;
}

uint32_t Actuators_GetMotorRuntime(uint8_t motor_index) {
    if (motor_index >= MOTOR_COUNT) {
        return 0;
    }
    return actuators_state.motors[motor_index].runtime;
}

// Функции эмуляции
void Actuators_EmulateHover(void) {
    if (emergency_stop_active) return;
    
    // Для зависания устанавливаем всем моторам одинаковую мощность
    uint16_t hover_pwm = PWM_MAX_VALUE / 2;
    Actuators_SetAllMotorsPWM(hover_pwm);
    actuators_state.global_throttle = hover_pwm;
}

void Actuators_EmulateAscent(void) {
    if (emergency_stop_active) return;
    
    // Для подъёма увеличиваем мощность всех моторов
    uint16_t ascent_pwm = (uint16_t)(PWM_MAX_VALUE * 0.7f);
    Actuators_SetAllMotorsPWM(ascent_pwm);
    actuators_state.global_throttle = ascent_pwm;
}

void Actuators_EmulateDescent(void) {
    if (emergency_stop_active) return;
    
    // Для снижения уменьшаем мощность всех моторов
    uint16_t descent_pwm = (uint16_t)(PWM_MAX_VALUE * 0.3f);
    Actuators_SetAllMotorsPWM(descent_pwm);
    actuators_state.global_throttle = descent_pwm;
}

void Actuators_EmulateRotation(int16_t yaw_rate) {
    if (emergency_stop_active) return;
    
    // Эмуляция вращения вокруг вертикальной оси
    Actuators_SetYaw(yaw_rate);
}

void Actuators_EmulateForwardFlight(uint16_t speed) {
    if (emergency_stop_active) return;
    
    // Эмуляция движения вперед
    Actuators_SetPitch(-speed); // Отрицательный питч для движения вперед
}

// Функции безопасности
void Actuators_EmergencyStop(void) {
    emergency_stop_active = true;
    Actuators_DisableAllMotors();
}

bool Actuators_IsEmergencyStopActive(void) {
    return emergency_stop_active;
}

void Actuators_ResetEmergencyStop(void) {
    emergency_stop_active = false;
}

// Функции управления моторами
void Actuators_SetAllMotors(float power) { }
void Actuators_SetMotor(uint8_t index, float power) { }

// Вспомогательные функции
static void UpdateMotorTemperatures(void) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
        if (actuators_state.motors[i].is_running) {
            // Эмуляция нагрева мотора в зависимости от PWM
            float pwm_factor = (float)actuators_state.motors[i].pwm_value / PWM_MAX_VALUE;
            actuators_state.motors[i].temperature += pwm_factor * 0.1f;
        } else {
            // Остывание мотора
            if (actuators_state.motors[i].temperature > 25.0f) {
                actuators_state.motors[i].temperature -= 0.05f;
            }
        }
    }
}

static void UpdateMotorRuntimes(void) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
        if (actuators_state.motors[i].is_running) {
            actuators_state.motors[i].runtime += 10; // Предполагаем вызов каждые 10 мс
        }
    }
}

static void ClampPWMValue(uint16_t* pwm_value) {
    if (*pwm_value > PWM_MAX_VALUE) {
        *pwm_value = PWM_MAX_VALUE;
    }
}
