#include <math.h>
#include <stddef.h>
#include "../include/sensors.h"
#include "../include/stabilization.h"
#include "../include/actuators.h"

// Прототипы вспомогательных функций
static float UpdatePID(PIDController* pid, float error);
static float ClampValue(float value, float min, float max);
static void UpdateMotorOutputs(float rollOutput, float pitchOutput, float yawOutput, float altitudeOutput);

// Константы для преобразования углов
#define DEG_TO_RAD(x) ((x) * 0.017453292519943295)
#define RAD_TO_DEG(x) ((x) * 57.295779513082323)

// Частота обновления системы стабилизации (Гц)
#define UPDATE_FREQUENCY 100
#define DT (1.0f / UPDATE_FREQUENCY)

// Ограничения на углы (градусы)
#define MAX_ROLL_ANGLE  45.0f
#define MAX_PITCH_ANGLE 45.0f
#define MAX_YAW_RATE    180.0f

// Структура состояния системы стабилизации
static struct {
    bool enabled;
    SetPoint targetSetPoint;
    Orientation currentOrientation;
    PIDController rollPID;
    PIDController pitchPID;
    PIDController yawPID;
    PIDController altitudePID;
} stabState;

// Вспомогательные функции
void Stabilization_Init(void) {
    // Сброс состояния
    stabState.enabled = false;
    
    // Инициализация уставок
    stabState.targetSetPoint.roll = 0.0f;
    stabState.targetSetPoint.pitch = 0.0f;
    stabState.targetSetPoint.yaw = 0.0f;
    stabState.targetSetPoint.altitude = 0.0f;
    
    // Инициализация текущей ориентации
    stabState.currentOrientation.roll = 0.0f;
    stabState.currentOrientation.pitch = 0.0f;
    stabState.currentOrientation.yaw = 0.0f;
    
    // Настройка PID-регуляторов по умолчанию
    Stabilization_ConfigurePID(
        1.0f, 0.1f, 0.05f,  // Roll
        1.0f, 0.1f, 0.05f,  // Pitch
        2.0f, 0.15f, 0.1f,  // Yaw
        1.5f, 0.2f, 0.1f    // Altitude
    );
}

void Stabilization_ConfigurePID(float rollKp, float rollKi, float rollKd,
                              float pitchKp, float pitchKi, float pitchKd,
                              float yawKp, float yawKi, float yawKd,
                              float altKp, float altKi, float altKd) {
    // Настройка PID для крена
    stabState.rollPID.kP = rollKp;
    stabState.rollPID.kI = rollKi;
    stabState.rollPID.kD = rollKd;
    stabState.rollPID.outputMin = -1.0f;
    stabState.rollPID.outputMax = 1.0f;
    
    // Настройка PID для тангажа
    stabState.pitchPID.kP = pitchKp;
    stabState.pitchPID.kI = pitchKi;
    stabState.pitchPID.kD = pitchKd;
    stabState.pitchPID.outputMin = -1.0f;
    stabState.pitchPID.outputMax = 1.0f;
    
    // Настройка PID для рыскания
    stabState.yawPID.kP = yawKp;
    stabState.yawPID.kI = yawKi;
    stabState.yawPID.kD = yawKd;
    stabState.yawPID.outputMin = -1.0f;
    stabState.yawPID.outputMax = 1.0f;
    
    // Настройка PID для высоты
    stabState.altitudePID.kP = altKp;
    stabState.altitudePID.kI = altKi;
    stabState.altitudePID.kD = altKd;
    stabState.altitudePID.outputMin = 0.0f;  // Минимальная тяга
    stabState.altitudePID.outputMax = 1.0f;  // Максимальная тяга
    
    // Сброс интегральных и дифференциальных составляющих
    Stabilization_ResetPID();
}

void Stabilization_SetTargetOrientation(float roll, float pitch, float yaw, float altitude) {
    // Ограничение углов
    stabState.targetSetPoint.roll = ClampValue(roll, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
    stabState.targetSetPoint.pitch = ClampValue(pitch, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
    stabState.targetSetPoint.yaw = yaw;  // Рыскание может быть любым (0-360)
    stabState.targetSetPoint.altitude = fmaxf(altitude, 0.0f);  // Высота не может быть отрицательной
}

void Stabilization_Update(void) {
    if (!stabState.enabled) {
        return;
    }
    
    // Получаем данные с датчиков
    MPU6050_Data imuData;
    BMP280_Data altData;
    Orientation currentOrientation;

    Sensors_GetIMUData(&imuData);
    Sensors_GetBaroData(&altData);
    Sensors_GetOrientation(&currentOrientation);

    // Обновляем текущую ориентацию
    stabState.currentOrientation.roll = currentOrientation.roll;
    stabState.currentOrientation.pitch = currentOrientation.pitch;
    stabState.currentOrientation.yaw = currentOrientation.yaw;
    
    // Вычисление ошибок
    float rollError = stabState.targetSetPoint.roll - stabState.currentOrientation.roll;
    float pitchError = stabState.targetSetPoint.pitch - stabState.currentOrientation.pitch;
    float yawError = stabState.targetSetPoint.yaw - stabState.currentOrientation.yaw;
    float altitudeError = stabState.targetSetPoint.altitude - altData.altitude;
    
    // Нормализация ошибки рыскания в диапазон [-180, 180]
    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;
    
    // Обновление PID регуляторов
    float rollOutput = UpdatePID(&stabState.rollPID, rollError);
    float pitchOutput = UpdatePID(&stabState.pitchPID, pitchError);
    float yawOutput = UpdatePID(&stabState.yawPID, yawError);
    float altitudeOutput = UpdatePID(&stabState.altitudePID, altitudeError);
    
    // Обновление выходных сигналов на моторы
    UpdateMotorOutputs(rollOutput, pitchOutput, yawOutput, altitudeOutput);
}

void Stabilization_GetCurrentOrientation(Orientation* orientation) {
    if (orientation != NULL) {
        *orientation = stabState.currentOrientation;
    }
}

void Stabilization_GetSetPoint(SetPoint* setPoint) {
    if (setPoint != NULL) {
        *setPoint = stabState.targetSetPoint;
    }
}

void Stabilization_Enable(void) {
    stabState.enabled = true;
    Stabilization_ResetPID();
}

void Stabilization_Disable(void) {
    stabState.enabled = false;
    Stabilization_ResetPID();
    
    // Остановка всех моторов
    Actuators_SetAllMotors(0.0f);
}

bool Stabilization_IsEnabled(void) {
    return stabState.enabled;
}

void Stabilization_ResetPID(void) {
    // Сброс всех PID регуляторов
    stabState.rollPID.integral = 0.0f;
    stabState.rollPID.prevError = 0.0f;
    
    stabState.pitchPID.integral = 0.0f;
    stabState.pitchPID.prevError = 0.0f;
    
    stabState.yawPID.integral = 0.0f;
    stabState.yawPID.prevError = 0.0f;
    
    stabState.altitudePID.integral = 0.0f;
    stabState.altitudePID.prevError = 0.0f;
}

// Вспомогательные функции
static float UpdatePID(PIDController* pid, float error) {
    if (pid == NULL) {
        return 0.0f;
    }
    
    // Пропорциональная составляющая
    float pTerm = pid->kP * error;
    
    // Интегральная составляющая с ограничением
    pid->integral += error * DT;
    float iTerm = pid->kI * pid->integral;
    
    // Дифференциальная составляющая
    float dTerm = pid->kD * (error - pid->prevError) / DT;
    pid->prevError = error;
    
    // Суммируем все составляющие
    float output = pTerm + iTerm + dTerm;
    
    // Ограничиваем выход
    return ClampValue(output, pid->outputMin, pid->outputMax);
}

static float ClampValue(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static void UpdateMotorOutputs(float rollOutput, float pitchOutput, float yawOutput, float altitudeOutput) {
    // Базовая тяга от регулятора высоты
    float baseThrust = altitudeOutput;
    
    // Вычисляем сигналы для каждого мотора
    // Моторы расположены в следующем порядке:
    // M1 - передний левый  (CCW)
    // M2 - передний правый (CW)
    // M3 - задний правый   (CCW)
    // M4 - задний левый    (CW)
    
    float m1 = baseThrust - rollOutput - pitchOutput + yawOutput;  // Передний левый
    float m2 = baseThrust + rollOutput - pitchOutput - yawOutput;  // Передний правый
    float m3 = baseThrust + rollOutput + pitchOutput + yawOutput;  // Задний правый
    float m4 = baseThrust - rollOutput + pitchOutput - yawOutput;  // Задний левый
    
    // Ограничиваем значения для каждого мотора
    m1 = ClampValue(m1, 0.0f, 1.0f);
    m2 = ClampValue(m2, 0.0f, 1.0f);
    m3 = ClampValue(m3, 0.0f, 1.0f);
    m4 = ClampValue(m4, 0.0f, 1.0f);
    
    // Устанавливаем значения на моторы
    Actuators_SetMotor(0, m1);
    Actuators_SetMotor(1, m2);
    Actuators_SetMotor(2, m3);
    Actuators_SetMotor(3, m4);
}
