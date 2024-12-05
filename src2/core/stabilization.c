#include "../include/constants.h"
#include "../include/structures.h"
#include <math.h>

// Локальные переменные для PID регуляторов
static PIDController roll_pid;
static PIDController pitch_pid;
static PIDController yaw_pid;
static PIDController altitude_pid;

// Локальные функции
static float update_pid(PIDController* pid, float current_value, float dt);

void InitializeStabilization(void) {
    // Инициализация PID регуляторов для крена
    roll_pid.kp = ROLL_KP;
    roll_pid.ki = ROLL_KI;
    roll_pid.kd = ROLL_KD;
    roll_pid.integral = 0.0f;
    roll_pid.prev_error = 0.0f;
    roll_pid.setpoint = 0.0f; // Целевой угол - 0 для удержания положения
    
    // Инициализация PID регуляторов для тангажа
    pitch_pid.kp = PITCH_KP;
    pitch_pid.ki = PITCH_KI;
    pitch_pid.kd = PITCH_KD;
    pitch_pid.integral = 0.0f;
    pitch_pid.prev_error = 0.0f;
    pitch_pid.setpoint = 0.0f; // Целевой угол - 0 для удержания положения
    
    // Инициализация PID регуляторов для рыскания
    yaw_pid.kp = YAW_KP;
    yaw_pid.ki = YAW_KI;
    yaw_pid.kd = YAW_KD;
    yaw_pid.integral = 0.0f;
    yaw_pid.prev_error = 0.0f;
    yaw_pid.setpoint = 0.0f; // Целевой угол - 0 для удержания положения
}

void UpdateStabilization(const ControlCommand* cmd, const SensorData* sensors, MotorControl* output) {
    static uint32_t last_update = 0;
    uint32_t current_time = 0; // TODO: Реализовать функцию получения времени
    float dt = (current_time - last_update) / 1000.0f; // Перевод в секунды
    
    // Обновление уставок PID регуляторов
    roll_pid.setpoint = 0.0f;  // Для удержания позиции всегда 0
    pitch_pid.setpoint = 0.0f; // Для удержания позиции всегда 0
    yaw_pid.setpoint = 0.0f;   // Для удержания позиции всегда 0
    
    // Расчет ошибок и обновление PID регуляторов
    float roll_correction = update_pid(&roll_pid, sensors->gyroscope.x, dt);
    float pitch_correction = update_pid(&pitch_pid, sensors->gyroscope.y, dt);
    float yaw_correction = update_pid(&yaw_pid, sensors->gyroscope.z, dt);
    
    // Базовая тяга для удержания высоты (можно настроить экспериментально)
    float base_throttle = 0.5f; // 50% мощности
    
    // Смешивание выходных сигналов для моторов
    mix_outputs(base_throttle, roll_correction, pitch_correction, yaw_correction, output);
    
    last_update = current_time;
}

// Обновление PID регулятора
static float update_pid(PIDController* pid, float current_value, float dt) {
    float error = pid->setpoint - current_value;
    
    // Пропорциональная составляющая
    float p_term = pid->kp * error;
    
    // Интегральная составляющая с ограничением
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;
    
    // Ограничение интегральной составляющей
    const float max_i = 100.0f;
    if (i_term > max_i) {
        i_term = max_i;
        pid->integral = max_i / pid->ki;
    } else if (i_term < -max_i) {
        i_term = -max_i;
        pid->integral = -max_i / pid->ki;
    }
    
    // Дифференциальная составляющая
    float d_term = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // Суммарный выход PID
    pid->output = p_term + i_term + d_term;
    
    // Ограничение выходного сигнала
    const float max_output = 500.0f;
    if (pid->output > max_output) {
        pid->output = max_output;
    } else if (pid->output < -max_output) {
        pid->output = -max_output;
    }
    
    return pid->output;
}

// Смешивание сигналов управления для моторов
void mix_outputs(float throttle, float roll, float pitch, float yaw, MotorControl* output) {
    // Расчет значений для каждого мотора
    output->front_left = (uint16_t)(MOTOR_MIN_PULSE + throttle * (MOTOR_MAX_PULSE - MOTOR_MIN_PULSE) 
                        + roll - pitch + yaw);
    output->front_right = (uint16_t)(MOTOR_MIN_PULSE + throttle * (MOTOR_MAX_PULSE - MOTOR_MIN_PULSE) 
                         - roll - pitch - yaw);
    output->rear_left = (uint16_t)(MOTOR_MIN_PULSE + throttle * (MOTOR_MAX_PULSE - MOTOR_MIN_PULSE) 
                       + roll + pitch - yaw);
    output->rear_right = (uint16_t)(MOTOR_MIN_PULSE + throttle * (MOTOR_MAX_PULSE - MOTOR_MIN_PULSE) 
                        - roll + pitch + yaw);
    
    // Ограничение значений
    output->front_left = output->front_left > MOTOR_MAX_PULSE ? MOTOR_MAX_PULSE : output->front_left;
    output->front_right = output->front_right > MOTOR_MAX_PULSE ? MOTOR_MAX_PULSE : output->front_right;
    output->rear_left = output->rear_left > MOTOR_MAX_PULSE ? MOTOR_MAX_PULSE : output->rear_left;
    output->rear_right = output->rear_right > MOTOR_MAX_PULSE ? MOTOR_MAX_PULSE : output->rear_right;
}
