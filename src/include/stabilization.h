#ifndef STABILIZATION_H
#define STABILIZATION_H

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"

// Структура для хранения параметров PID-регулятора
typedef struct {
    float kP;          // Пропорциональный коэффициент
    float kI;          // Интегральный коэффициент
    float kD;          // Дифференциальный коэффициент
    float integral;    // Накопленная интегральная составляющая
    float prevError;   // Предыдущая ошибка для вычисления производной
    float outputMin;   // Минимальное значение выхода
    float outputMax;   // Максимальное значение выхода
} PIDController;

// Структура для хранения желаемых значений углов
typedef struct {
    float roll;     // Желаемый крен
    float pitch;    // Желаемый тангаж
    float yaw;      // Желаемое рыскание
    float altitude; // Желаемая высота
} SetPoint;

// Структура данных стабилизации
typedef struct {
    Orientation currentOrientation;
    SetPoint targetSetPoint;
    float motorOutputs[4];
    bool isStable;
} StabilizationData;

// Основные функции стабилизации
void Stabilization_Init(void);
void Stabilization_Update(void);
void Stabilization_ResetPID(void);

// Настройка PID-регуляторов
void Stabilization_ConfigurePID(float rollKp, float rollKi, float rollKd,
                              float pitchKp, float pitchKi, float pitchKd,
                              float yawKp, float yawKi, float yawKd,
                              float altKp, float altKi, float altKd);

// Функции управления ориентацией
void Stabilization_SetTargetOrientation(float roll, float pitch, float yaw, float altitude);
void Stabilization_GetCurrentOrientation(Orientation* orientation);
void Stabilization_GetCurrentSetPoint(SetPoint* setPoint);

// Функции управления состоянием
void Stabilization_Enable(void);
void Stabilization_Disable(void);
bool Stabilization_IsEnabled(void);

#endif // STABILIZATION_H
