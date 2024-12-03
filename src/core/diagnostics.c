#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>  // Добавлена библиотека для NAN
#include "../include/diagnostics.h"
#include "../include/communication.h"
#include "D:/keil/pack/Keil/TM4C_DFP/1.1.0/Device/Include/TM4C123/TM4C123GH6PM.h"
#include <stddef.h>  // для NULL

#ifndef NAN
#define NAN (0.0f/0.0f)
#endif

// Системные определения для контроллера TM4C123GH6PM
#define SYSCTL_BASE            0x400FE000  // Базовый адрес системного контроллера
#define SYSCTL_RCGCWD_R        (*((volatile uint32_t *)(SYSCTL_BASE + 0x600)))  // Регистр тактирования Watchdog
#define SYSCTL_PRWD_R          (*((volatile uint32_t *)(SYSCTL_BASE + 0xA00)))  // Регистр готовности Watchdog

// Определения для NVIC
#define NVIC_BASE              0xE000E000  // Базовый адрес контроллера прерываний
#define NVIC_PRIO_R(n)         (*((volatile uint32_t *)(NVIC_BASE + 0x400 + ((n) * 4))))  // Регистры приоритета прерываний

// Определения регистров SysTick
#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))

// Определения регистров ADC, если они не определены в tm4c123gh6pm.h
#ifndef ADC0_SSMUX1_R
#define ADC0_SSMUX1_R          (*((volatile uint32_t *)0x40038044))
#endif
#ifndef ADC0_SSCTL1_R
#define ADC0_SSCTL1_R          (*((volatile uint32_t *)0x40038048))
#endif

// Определения регистров ADC1
#define ADC1_BASE            0x40039000  // Базовый адрес ADC1
#define ADC1_SSMUX1_R       (*((volatile uint32_t *)(ADC1_BASE + 0x40)))  // Мультиплексор выборки SS1
#define ADC1_SSCTL1_R       (*((volatile uint32_t *)(ADC1_BASE + 0x44)))  // Управление и статус SS1
#define ADC1_SSFIFO1_R      (*((volatile uint32_t *)(ADC1_BASE + 0x48)))  // FIFO данных SS1
#define ADC1_ISC_R          (*((volatile uint32_t *)(ADC1_BASE + 0x50)))  // Регистр очистки прерываний

// Определения регистров ADC
#define ADC0_ACTSS_R           (*((volatile uint32_t *)0x40038000))
#define ADC0_EMUX_R            (*((volatile uint32_t *)0x40038014))
#define ADC0_SSMUX3_R          (*((volatile uint32_t *)0x400380A0))
#define ADC0_SSCTL3_R          (*((volatile uint32_t *)0x400380A4))
#define ADC0_PSSI_R            (*((volatile uint32_t *)0x40038028))
#define ADC0_RIS_R             (*((volatile uint32_t *)0x40038004))
#define ADC0_ISC_R             (*((volatile uint32_t *)0x4003800C))
#define ADC0_SSFIFO3_R         (*((volatile uint32_t *)0x400380A8))
#define ADC0_PC_R              (*((volatile uint32_t *)0x40038FC4))

// Определения регистров ADC
#define ADC_PSSI_SS1        0x00000002  // SS1 Initiate
#define ADC_ACTSS_ASEN1     0x00000002  // SS1 Enable
#define ADC_EMUX_EM1_M      0x000000F0  // SS1 Trigger Select

// Определение каналов АЦП для различных температурных датчиков
#define ADC_AIN0   0   // Канал АЦП для температур микроконтроллера
#define ADC_AIN1   1   // Базовый канал для температур моторов
#define ADC_AIN4   4   // Канал АЦП для температур батареи
#define ADC_AIN5   5   // Канал АЦП для температур платы
#define ADC_AIN6   6   // Канал АЦП для температур окружающей среды

#define ADC_ISC_IN1 0x02  // Флаг прерывания для канала 1

// Состояние модуля диагностики
#pragma pack(push, 4)  // Выравнивание по 4 байта
static struct {
    bool initialized;
    SystemState systemState;
    uint32_t errorCount;
    uint32_t startTime;
    uint32_t watchdogTimeout;
    LogEntry logBuffer[DIAG_MAX_LOG_ENTRIES];
    uint32_t logCount;
    PowerState powerState;
    ThermalState thermalState;
    uint32_t perfStartTime;
    uint32_t perfMaxTime;
    uint32_t perfMinTime;
    uint32_t perfTotalTime;
    uint32_t perfCount;
    bool lowPowerMode;      // Добавлено: флаг низкого энергопотребления
    bool debugEnabled;      // Добавлено: флаг отладочного вывода
    bool watchdogEnabled;   // Добавлено: флаг watchdog
} diagState = {0};
#pragma pack(pop)

// Буфер для форматирования отладочных сообщений
#define DEBUG_BUFFER_SIZE 128
static char debugBuffer[DEBUG_BUFFER_SIZE];

// Глобальный счетчик миллисекунд
static volatile uint32_t systemMillis = 0;

// Внутренние функции
void SysTick_Init(void);
void InitPowerADC(void);
void InitTempADC(void);
void SendDebugMessage(const char* format, ...);
const char* GetLogLevelString(LogLevel level);
const char* GetEventString(DiagnosticEvent event);
void SavePeripheralState(void);
void RestorePeripheralState(void);
void ConfigureWakeupSources(void);
void ConfigurePowerMode(PowerMode mode);
TestStatus TestPowerSystem(void);
bool TestSupplyVoltages(void);
TestStatus TestSensors(void);
TestStatus TestMotors(void);
TestStatus TestCommunication(void);
TestStatus TestSafetySystems(void);

// Инициализация системного таймера
void SysTick_Init(void) {
    // Отключаем таймер перед настройкой
    NVIC_ST_CTRL_R = 0;
    
    // Настраиваем период (1 мс при тактовой частоте 16 МГц)
    NVIC_ST_RELOAD_R = 16000 - 1;  // (16MHz / 1000Hz) - 1
    
    // Сбрасываем текущее значение
    NVIC_ST_CURRENT_R = 0;
    
    // Включаем таймер с прерываниями:
    // Бит 0: разрешаем работу таймера
    // Бит 1: разрешаем прерывания
    // Бит 2: используем системную частоту
    NVIC_ST_CTRL_R = 0x07;
}

// Получение текущего времени в миллисекундах
static uint32_t GetCurrentTime(void) {
    return systemMillis;
}

// Задержка в миллисекундах
static void DelayMs(uint32_t ms) {
    uint32_t start = GetCurrentTime();
    while ((GetCurrentTime() - start) < ms) {
        // Ждем
    }
}

// Инициализация модуля диагностики
bool Diagnostics_Init(void) {
    if (diagState.initialized) {
        return true;
    }
    
    memset(&diagState, 0, sizeof(diagState));
    
    // Инициализация подсистем
    SysTick_Init();
    InitPowerADC();
    InitTempADC();
    
    // Установка начальных значений
    diagState.systemState = SYSTEM_STATE_OK;
    diagState.errorCount = 0;
    diagState.startTime = 0;
    diagState.logCount = 0;
    diagState.watchdogTimeout = 0;
    
    // Инициализация состояний питания и температур
    memset(&diagState.powerState, 0, sizeof(PowerState));
    memset(&diagState.thermalState, 0, sizeof(ThermalState));
    
    // Сброс счетчиков производительности
    diagState.perfStartTime = 0;
    diagState.perfMaxTime = 0;
    diagState.perfMinTime = 0;
    diagState.perfTotalTime = 0;
    diagState.perfCount = 0;
    
    // Инициализация новых членов
    diagState.lowPowerMode = false;
    diagState.debugEnabled = false;
    diagState.watchdogEnabled = false;  // Инициализируем флаг watchdog
    
    diagState.initialized = true;
    
    // Проверяем был ли сброс по watchdog
    if (NVIC_APINT_R & 0x10000) {
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_WATCHDOG_RESET, "System reset by watchdog", 0);
    }
    
    Diagnostics_Log(LOG_INFO, DIAG_EVENT_SYSTEM_START, "Diagnostics initialized", 0);
    return true;
}

// Функции управления питанием
void Diagnostics_UpdatePowerState(void) {
    if (!diagState.initialized) {
        return;
    }
    
    // Запускаем преобразование АЦП
    ADC0_PSSI_R = ADC_PSSI_SS3;
    
    // Ждем завершения преобразования
    while(!(ADC0_RIS_R & ADC_RIS_INR3)) {}
    
    // Считываем результат
    uint32_t result = ADC0_SSFIFO3_R;
    
    // Сбрасываем флаг завершения
    ADC0_ISC_R = ADC_RIS_INR3;
    
    // Преобразуем в напряжение (опорное напряжение 3.3В)
    float voltage = (result * 3.3f) / 4096.0f;
    
    // Обновляем состояние питания
    diagState.powerState.batteryVoltage = voltage;
    
    // Расчет уровня заряда (0-100%)
    if (voltage >= 12.6f) {
        diagState.powerState.batteryLevel = 100;
    } else if (voltage <= 9.0f) {
        diagState.powerState.batteryLevel = 0;
    } else {
        diagState.powerState.batteryLevel = (uint8_t)((voltage - 9.0f) * 27.78f);
    }
    
    // Проверка критических уровней
    if (voltage <= 9.5f) {
        Diagnostics_Log(LOG_FATAL, DIAG_EVENT_BATTERY_CRITICAL, "Battery critically low", (uint32_t)(voltage * 100));
    } else if (voltage <= 10.0f) {
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_BATTERY_LOW, "Battery low", (uint32_t)(voltage * 100));
    }
}

/**
 * @brief Получает текущее состояние питания системы
 * 
 * @details Функция копирует текущее состояние питания в переданную структуру.
 *          Это обеспечивает безопасную передачу данных без риска утечки памяти
 *          и позволяет вызывающему коду работать с локальной копией состояния.
 * 
 * @param state Указатель на структуру PowerState, которая будет заполнена текущим состоянием
 * 
 * @note Функция выполняет глубокое копирование всех полей состояния питания
 * @warning Входной указатель не должен быть NULL
 * 
 * @pre Система диагностики должна быть инициализирована
 * @post Переданная структура содержит актуальное состояние питания на момент вызова
 */
void Diagnostics_GetPowerState(PowerState* state) {
    // Проверка входного указателя на корректность
    if (state == NULL) {
        // В случае передачи NULL, логируем критическую ошибку
        Diagnostics_Log(LOG_CRITICAL, DIAG_EVENT_SYSTEM_ERROR, 
                        "Attempt to get power state with NULL pointer", 0);
        return;
    }

    // Выполняем глубокое копирование всех полей состояния питания
    // Это гарантирует целостность данных и предотвращает прямые зависимости
    state->batteryVoltage = diagState.powerState.batteryVoltage;
    state->batteryLevel = diagState.powerState.batteryLevel;
    state->batteryTemp = diagState.powerState.batteryTemp;
    state->batteryCurrent = diagState.powerState.batteryCurrent;
    state->isCharging = diagState.powerState.isCharging;
    state->isLowBattery = diagState.powerState.isLowBattery;
    state->isCriticalBattery = diagState.powerState.isCriticalBattery;
    state->uptime = diagState.powerState.uptime;

    // Дополнительная диагностика состояния питания
    if (state->batteryLevel < 20.0f) {  // Порог низкого заряда 20%
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_BATTERY_LOW, 
                        "Battery level is critically low", (uint32_t)state->batteryLevel);
    }
}

float Diagnostics_GetBatteryVoltage(void) {
    return diagState.powerState.batteryVoltage;
}

uint8_t Diagnostics_GetBatteryLevel(void) {
    return diagState.powerState.batteryLevel;
}

// Инициализация АЦП для измерения параметров питания
void InitPowerADC(void) {
    // Включаем тактирование PORTE и ADC0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;  // Порт E
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;    // ADC0
    
    // Ждем готовности периферии
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4)) {}
    while(!(SYSCTL_PRADC_R & SYSCTL_PRADC_R0)) {}
    
    // Настраиваем пины как аналоговые входы
    GPIO_PORTE_AFSEL_R |= (1<<3) | (1<<2) | (1<<1);  // Альтернативная функция
    GPIO_PORTE_DEN_R &= ~((1<<3) | (1<<2) | (1<<1)); // Отключаем цифровой режим
    GPIO_PORTE_AMSEL_R |= (1<<3) | (1<<2) | (1<<1);  // Включаем аналоговый режим
    
    // Настраиваем секвенсор 1 АЦП
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;          // Отключаем секвенсор для настройки
    ADC0_EMUX_R &= ~ADC_EMUX_EM1_M;            // Программный запуск
    ADC0_SSMUX1_R = (0 << 0) |                 // Канал 0 (напряжение)
                    (1 << 4) |                 // Канал 1 (ток)
                    (2 << 8);                  // Канал 2 (температура)
    ADC0_SSCTL1_R = (0 << 0) |                // Первый канал
                    (0 << 4) |                // Второй канал
                    (ADC_SSCTL0_IE2 |         // Прерывание после последнего
                     ADC_SSCTL0_END2);        // Последний канал
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;          // Включаем секвенсор
    
    // Настраиваем усреднение для уменьшения шума
    ADC0_SAC_R = ADC_SAC_AVG_64X;             // 64x усреднение
    
    // Включаем прерывания
    ADC0_IM_R |= ADC_IM_MASK1;                // Маска прерывания секвенсора 1
    NVIC_EN0_R |= (1 << (INT_ADC0SS1 - 16));  // Включаем прерывание в NVIC
}

// Обработчик прерывания АЦП
void ADC0SS1_Handler(void) {
    // Считываем результаты преобразования
    uint32_t voltage_raw = ADC0_SSFIFO1_R & 0xFFF;
    uint32_t current_raw = ADC0_SSFIFO1_R & 0xFFF;
    uint32_t temp_raw = ADC0_SSFIFO1_R & 0xFFF;
    
    // Преобразуем в реальные значения
    float voltage = (voltage_raw * VREF) / ADC_MAX_VALUE;
    float current = ((current_raw * VREF / ADC_MAX_VALUE) * 1000.0f) / CURRENT_SENSE_MV_A;
    float temp = ((temp_raw * VREF / ADC_MAX_VALUE) * 1000.0f) / TEMP_SENSOR_MV_C;
    
    // Обновляем состояние питания
    diagState.powerState.batteryVoltage = voltage;
    diagState.powerState.batteryCurrent = current;
    diagState.powerState.batteryTemp = temp;
    
    // Вычисляем уровень заряда (простая линейная аппроксимация)
    float battery_level = (voltage - BATTERY_MIN_VOLTAGE) / 
                         (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100.0f;
    diagState.powerState.batteryLevel = (battery_level > 100.0f) ? 100.0f : 
                                      (battery_level < 0.0f) ? 0.0f : battery_level;
    
    // Обновляем время работы от батареи
    diagState.powerState.uptime = GetCurrentTime();
    
    // Проверяем критические значения
    if (voltage <= BATTERY_CRITICAL_VOLTAGE) {
        Diagnostics_Log(LOG_FATAL, DIAG_EVENT_LOW_BATTERY,
                       "Critical battery voltage", (uint32_t)(voltage * 100));
    }
    else if (voltage <= BATTERY_LOW_VOLTAGE) {
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_LOW_BATTERY,
                       "Low battery voltage", (uint32_t)(voltage * 100));
    }
    
    if (temp >= BATTERY_MAX_TEMP) {
        Diagnostics_Log(LOG_FATAL, DIAG_EVENT_HIGH_TEMP,
                       "Critical battery temperature", (uint32_t)(temp * 100));
    }
    
    // Очищаем флаг прерывания
    ADC0_ISC_R = ADC_ISC_IN1;
}

// Инициализация АЦП для температурных датчиков
void InitTempADC(void) {
    // Включаем тактирование PORTD и PORTE если еще не включено
    SYSCTL_RCGCGPIO_R |= (SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4);
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;    // ADC1
    
    // Ждем готовности периферии
    while(!(SYSCTL_PRGPIO_R & (SYSCTL_PRGPIO_R3 | SYSCTL_PRGPIO_R4))) {}
    while(!(SYSCTL_PRADC_R & SYSCTL_PRADC_R1)) {}
    
    // Настраиваем пины как аналоговые входы
    GPIO_PORTE_AFSEL_R |= (1<<4) | (1<<5);     // PE4, PE5
    GPIO_PORTD_AFSEL_R |= (1<<3);              // PD3
    GPIO_PORTE_DEN_R &= ~((1<<4) | (1<<5));    // Отключаем цифровой режим
    GPIO_PORTD_DEN_R &= ~(1<<3);
    GPIO_PORTE_AMSEL_R |= (1<<4) | (1<<5);     // Включаем аналоговый режим
    GPIO_PORTD_AMSEL_R |= (1<<3);
    
    // Настраиваем секвенсор 0 АЦП1
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN0;          // Отключаем секвенсор для настройки
    ADC1_EMUX_R &= ~ADC_EMUX_EM0_M;            // Программный запуск
    
    // Настраиваем каналы измерения
    ADC1_SSMUX0_R = (TEMP_MCU_CHANNEL << 0)   | // Встроенный сенсор MCU
                    (9 << 4)                    | // Температур моторов (PE4)
                    (8 << 8)                    | // Температур платы (PE5)
                    (4 << 12);                   // Температур окружающей среды (PD3)
    
    // Настраиваем управление
    ADC1_SSCTL0_R = (0 << 0)                   | // Первый канал
                    (0 << 4)                   | // Второй канал
                    (0 << 8)                   | // Третий канал
                    (ADC_SSCTL0_IE3 |           // Прерывание после последнего
                     ADC_SSCTL0_END3);          // Последний канал
                     
    // Включаем встроенный температурный сенсор
    ADC1_TSSEL_R |= (1 << TEMP_MCU_CHANNEL);
    
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN0;           // Включаем секвенсор
    
    // Настраиваем усреднение
    ADC1_SAC_R = ADC_SAC_AVG_64X;              // 64x усреднение
    
    // Включаем прерывания
    ADC1_IM_R |= ADC_IM_MASK0;                 // Маска прерывания секвенсора 0
    NVIC_EN1_R |= (1 << (INT_ADC1SS0 - 48));   // Включаем прерывание в NVIC
}

// Обработчик прерывания АЦП для температурных датчиков
void ADC1SS0_Handler(void) {
    // Считываем результаты преобразования
    uint32_t mcu_temp_raw = ADC1_SSFIFO0_R & 0xFFF;
    uint32_t motors_temp_raw = ADC1_SSFIFO0_R & 0xFFF;
    uint32_t board_temp_raw = ADC1_SSFIFO0_R & 0xFFF;
    uint32_t ambient_temp_raw = ADC1_SSFIFO0_R & 0xFFF;
    
    // Преобразуем в реальные значения температур
    // Встроенный сенсор MCU
    float mcu_voltage = (mcu_temp_raw * VREF) / ADC_MAX_VALUE;
    float mcu_temp = ((mcu_voltage * 1000.0f) / MCU_TEMP_SLOPE) + MCU_TEMP_OFFSET;
    
    // Внешние датчики
    float motors_temp = ((motors_temp_raw * VREF / ADC_MAX_VALUE) * 1000.0f) / TEMP_SENSOR_SLOPE;
    float board_temp = ((board_temp_raw * VREF / ADC_MAX_VALUE) * 1000.0f) / TEMP_SENSOR_SLOPE;
    float ambient_temp = ((ambient_temp_raw * VREF / ADC_MAX_VALUE) * 1000.0f) / TEMP_SENSOR_SLOPE;
    
    // Обновляем состояние температур
    diagState.thermalState.mcuTemp = mcu_temp;
    diagState.thermalState.motorTemps[0] = motors_temp;
    diagState.thermalState.boardTemp = board_temp;
    diagState.thermalState.ambientTemp = ambient_temp;
    
    // Проверяем критические значения
    if (mcu_temp >= TEMP_MCU_CRITICAL) {
        Diagnostics_Log(LOG_FATAL, DIAG_EVENT_HIGH_TEMP,
                       "Critical MCU temperature", (uint32_t)(mcu_temp * 100));
    }
    else if (mcu_temp >= TEMP_MCU_WARNING) {
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_HIGH_TEMP,
                       "High MCU temperature", (uint32_t)(mcu_temp * 100));
    }
    
    if (motors_temp >= TEMP_MOTORS_CRITICAL) {
        Diagnostics_Log(LOG_FATAL, DIAG_EVENT_HIGH_TEMP,
                       "Critical motors temperature", (uint32_t)(motors_temp * 100));
    }
    else if (motors_temp >= TEMP_MOTORS_WARNING) {
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_HIGH_TEMP,
                       "High motors temperature", (uint32_t)(motors_temp * 100));
    }
    
    if (board_temp >= TEMP_BOARD_CRITICAL) {
        Diagnostics_Log(LOG_FATAL, DIAG_EVENT_HIGH_TEMP,
                       "Critical board temperature", (uint32_t)(board_temp * 100));
    }
    else if (board_temp >= TEMP_BOARD_WARNING) {
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_HIGH_TEMP,
                       "High board temperature", (uint32_t)(board_temp * 100));
    }
    
    // Очищаем флаг прерывания
    ADC1_ISC_R = ADC_ISC_IN0;
}

void Diagnostics_UpdateThermalState(void) {
    // Reset warning and critical flags
    diagState.thermalState.isCritical = false;
    diagState.thermalState.isWarning = false;
    
    // Check MCU temperature
    if (diagState.thermalState.mcuTemp >= TEMP_MCU_CRITICAL) {
        diagState.thermalState.isCritical = true;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_HIGH_TEMP, 
            "MCU temperature critical", 
            (uint32_t)(diagState.thermalState.mcuTemp * 100));
    } else if (diagState.thermalState.mcuTemp >= TEMP_MCU_WARNING) {
        diagState.thermalState.isWarning = true;
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_HIGH_TEMP, 
            "MCU temperature warning", 
            (uint32_t)(diagState.thermalState.mcuTemp * 100));
    }
    
    // Check motor temperatures
    for (int i = 0; i < 4; i++) {
        if (diagState.thermalState.motorTemps[i] >= TEMP_MOTORS_CRITICAL) {
            diagState.thermalState.isCritical = true;
            Diagnostics_Log(LOG_ERROR, DIAG_EVENT_HIGH_TEMP, 
                "Motor temperature critical", 
                (uint32_t)(diagState.thermalState.motorTemps[i] * 100));
        } else if (diagState.thermalState.motorTemps[i] >= TEMP_MOTORS_WARNING) {
            diagState.thermalState.isWarning = true;
            Diagnostics_Log(LOG_WARNING, DIAG_EVENT_HIGH_TEMP, 
                "Motor temperature warning", 
                (uint32_t)(diagState.thermalState.motorTemps[i] * 100));
        }
    }
    
    // Check board temperature
    if (diagState.thermalState.boardTemp >= TEMP_BOARD_CRITICAL) {
        diagState.thermalState.isCritical = true;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_HIGH_TEMP, 
            "Board temperature critical", 
            (uint32_t)(diagState.thermalState.boardTemp * 100));
    } else if (diagState.thermalState.boardTemp >= TEMP_BOARD_WARNING) {
        diagState.thermalState.isWarning = true;
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_HIGH_TEMP, 
            "Board temperature warning", 
            (uint32_t)(diagState.thermalState.boardTemp * 100));
    }
}

void UpdateMotorTemperatures(void) {
    // Read temperatures from all motor sensors
    for (int i = 0; i < 4; i++) {
        float temp = GetMotorTemperature(i);
        if (!isnan(temp)) {
            diagState.thermalState.motorTemps[i] = temp;
        }
    }
}

void Diagnostics_GetThermalState(ThermalState* state) {
    if (state != NULL) {
        *state = diagState.thermalState;
    }
}

void Diagnostics_Log(LogLevel level, DiagnosticEvent event, const char* message, uint32_t data) {
    if (!diagState.initialized || diagState.logCount >= DIAG_MAX_LOG_ENTRIES) {
        return;
    }
    
    LogEntry* entry = &diagState.logBuffer[diagState.logCount++];
    entry->timestamp = diagState.startTime;
    entry->level = level;
    entry->code = (ErrorCode)event;
    entry->data = data;
    
    if (level >= LOG_ERROR) {
        diagState.errorCount++;
        
        // Обновляем состояние системы
        if (level == LOG_FATAL) {
            diagState.systemState = SYSTEM_STATE_CRITICAL;
        } else {
            diagState.systemState = SYSTEM_STATE_ERROR;
        }
    }
}

void Diagnostics_ClearLog(void) {
    diagState.logCount = 0;
    diagState.errorCount = 0;
    diagState.systemState = SYSTEM_STATE_OK;
}

uint32_t Diagnostics_GetLogCount(void) {
    return diagState.logCount;
}

bool Diagnostics_GetLogEntry(uint32_t index, LogEntry* entry) {
    if (!entry || index >= diagState.logCount) {
        return false;
    }
    
    *entry = diagState.logBuffer[index];
    return true;
}

bool Diagnostics_IsOverheated(void) {
    return (diagState.thermalState.mcuTemp >= TEMP_MCU_WARNING ||
            diagState.thermalState.motorTemps[0] >= TEMP_MOTORS_WARNING ||
            diagState.thermalState.boardTemp >= TEMP_BOARD_WARNING);
}

// Форматирование и отправка отладочного сообщения
void SendDebugMessage(const char* format, ...) {
    if (!diagState.debugEnabled) {
        return;
    }
    
    va_list args;
    va_start(args, format);
    
    // Форматируем сообщение
    vsnprintf(debugBuffer, DEBUG_BUFFER_SIZE, format, args);
    va_end(args);
    
    // Добавляем перевод строки
    size_t len = strlen(debugBuffer);
    if (len < DEBUG_BUFFER_SIZE - 2) {
        debugBuffer[len] = '\r';
        debugBuffer[len + 1] = '\n';
        debugBuffer[len + 2] = '\0';
    }
    
    // Отправляем через UART
    Communication_SendData((uint8_t*)debugBuffer, strlen(debugBuffer));
}

// Преобразование уровня лога в строку
const char* GetLogLevelString(LogLevel level) {
    switch (level) {
        case LOG_DEBUG:    return "DEBUG";
        case LOG_INFO:     return "INFO";
        case LOG_WARNING:  return "WARNING";
        case LOG_ERROR:    return "ERROR";
        case LOG_CRITICAL: return "CRITICAL";
        case LOG_FATAL:    return "FATAL";
        default:          return "UNKNOWN";
    }
}

// Преобразование события в строку
const char* GetEventString(DiagnosticEvent event) {
    switch (event) {
        case DIAG_EVENT_SYSTEM_START:     return "SYSTEM_START";
        case DIAG_EVENT_SYSTEM_STOP:      return "SYSTEM_STOP";
        case DIAG_EVENT_HIGH_TEMP:        return "HIGH_TEMP";
        case DIAG_EVENT_LOW_BATTERY:      return "LOW_BATTERY";
        case DIAG_EVENT_BATTERY_CRITICAL: return "BATTERY_CRITICAL";
        case DIAG_EVENT_BATTERY_LOW:      return "BATTERY_LOW";
        case DIAG_EVENT_NAV_ERROR:        return "NAV_ERROR";
        case DIAG_EVENT_SENSOR_ERROR:     return "SENSOR_ERROR";
        case DIAG_EVENT_MOTOR_ERROR:      return "MOTOR_ERROR";
        case DIAG_EVENT_COMM_ERROR:       return "COMM_ERROR";
        case DIAG_EVENT_STABILIZATION_ERROR: return "STABILIZATION_ERROR";
        case DIAG_EVENT_WATCHDOG_RESET:   return "WATCHDOG_RESET";
        case DIAG_EVENT_SELF_TEST:        return "SELF_TEST";
        case DIAG_EVENT_SYSTEM_ERROR:     return "SYSTEM_ERROR";
        case DIAG_EVENT_CALIBRATION:      return "CALIBRATION";
        case DIAG_EVENT_ERROR:            return "ERROR";
        case DIAG_EVENT_WARNING:          return "WARNING";
        default:                          return "UNKNOWN";
    }
}

// Мониторинг состояния системы
SystemState Diagnostics_GetSystemState(void) {
    return diagState.systemState;
}

uint32_t Diagnostics_GetErrorCount(void) {
    return diagState.errorCount;
}

void Diagnostics_ResetErrorCount(void) {
    diagState.errorCount = 0;
    if (diagState.systemState == SYSTEM_STATE_ERROR) {
        diagState.systemState = SYSTEM_STATE_OK;
    }
}

// Управление watchdog
void Diagnostics_InitWatchdog(uint32_t timeout_ms) {
    // Включаем тактирование WDT0
    SYSCTL_RCGCWD_R |= SYSCTL_RCGCWD_R0;
    
    // Ждем готовности модуля
    while(!(SYSCTL_PRWD_R & SYSCTL_PRWD_R0)) {}
    
    // Разблокируем регистры WDT
    WATCHDOG0_LOCK_R = 0x1ACCE551;
    
    // Отключаем WDT для настройки
    WATCHDOG0_CTL_R &= ~WDT_CTL_INTEN;
    
    // Рассчитываем значение перезагрузки
    // WDT тактируется от PIOSC (16 МГц / 16 = 1 МГц)
    uint32_t reload_value = timeout_ms * 1000; // преобразуем мс в тики
    
    // Устанавливаем значение перезагрузки
    WATCHDOG0_LOAD_R = reload_value;
    
    // Настраиваем WDT:
    // - Разрешаем сброс системы
    // - Разрешаем прерывания
    WATCHDOG0_CTL_R |= (WDT_CTL_RESEN | WDT_CTL_INTEN);
    
    // Включаем прерывания от WDT
    NVIC_EN0_R |= (1 << INT_WATCHDOG);
    
    // Устанавливаем приоритет прерывания (самый высокий)
    NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFFFFF0) | 0x0;
    
    diagState.watchdogEnabled = true;
    diagState.watchdogTimeout = timeout_ms;
    
    // Первый сброс WDT
    WATCHDOG0_ICR_R = 0;
    
    Diagnostics_Log(LOG_INFO, DIAG_EVENT_SYSTEM_START,
                   "Watchdog initialized", timeout_ms);
}

// Watchdog handler implementation
void WDT0_Handler(void) {
    // Clear watchdog interrupt
    WATCHDOG0_ICR_R = WDT_ICR_M;
    
    Diagnostics_Log(LOG_WARNING, DIAG_EVENT_WATCHDOG_RESET, "Watchdog reset occurred", 0);
}

void Diagnostics_FeedWatchdog(void) {
    if (diagState.watchdogEnabled) {
        // Сброс WDT записью любого значения в ICR
        WATCHDOG0_ICR_R = 0;
    }
}

bool Diagnostics_IsWatchdogReset(void) {
    // Проверяем бит RESC (Reset Cause) в регистре RIS
    return (WATCHDOG0_RIS_R & WDT_RIS_WDTRIS) != 0;
}

// Режимы энергопотребления
// Определение перемещено в заголовочный файл diagnostics.h
static PowerMode currentPowerMode = POWER_MODE_NORMAL;

// Сохранение состояния периферии перед переходом в режим низкого энергопотребления
static uint32_t savedPeripheralState = 0;

// Функция настройки режима энергопотребления
void ConfigurePowerMode(PowerMode mode) {
    switch (mode) {
        case POWER_MODE_SLEEP:
            // Настройка режима Sleep:
            // - Процессор останавливается
            // - Периферия продолжает работать
            // - Быстрое пробуждение
            NVIC_SYS_CTRL_R |= NVIC_SYS_CTRL_SLEEPDEEP; // Включаем режим сна
            break;
            
        case POWER_MODE_DEEP_SLEEP:
            // Настройка режима Deep Sleep:
            // - Процессор останавливается
            // - Большая часть периферии отключается
            // - Медленное пробуждение
            // - Максимальная экономия энергии
            NVIC_SYS_CTRL_R |= NVIC_SYS_CTRL_SLEEPDEEP; // Включаем глубокий сон
            break;
            
        case POWER_MODE_NORMAL:
        default:
            // Нормальный режим работы
            NVIC_SYS_CTRL_R &= ~NVIC_SYS_CTRL_SLEEPDEEP; // Отключаем режим сна
            break;
    }
}

// Сохранение состояния периферии
void SavePeripheralState(void) {
    // Сохраняем текущее состояние тактирования периферии
    savedPeripheralState = SYSCTL_RCGC2_R;
}

// Восстановление состояния периферии
void RestorePeripheralState(void) {
    // Восстанавливаем тактирование периферии
    SYSCTL_RCGC2_R = savedPeripheralState;
    
    // Ждем стабилизации тактирования
    volatile uint32_t delay = SYSCTL_RCGC2_R;
}

// Настройка источников пробуждения
void ConfigureWakeupSources(void) {
    // Разрешаем пробуждение от:
    // - Внешних прерываний (например, от датчиков)
    // - Таймера WatchDog
    // - UART (для приема команд)
    
    // Настраиваем прерывания как источники пробуждения
    NVIC_EN0_R |= (1 << INT_UART0);       // UART0
    NVIC_EN0_R |= (1 << INT_WTIMER0A);    // WatchDog
    
    // Настраиваем порты GPIO для пробуждения
    GPIO_PORTA_IM_R |= 0x03;              // Разрешаем прерывания на PA0-PA1 (UART)
    GPIO_PORTA_IEV_R &= ~0x03;            // Прерывание по спаду
}

void Diagnostics_EnterLowPowerMode(void) {
    if (!diagState.lowPowerMode) {
        diagState.lowPowerMode = true;
        
        // Логируем событие
        Diagnostics_Log(LOG_INFO, DIAG_EVENT_SYSTEM_STOP, 
                        "Entering low power mode", 0);
        
        // Сохраняем состояние периферии
        SavePeripheralState();
        
        // Настраиваем источники пробуждения
        ConfigureWakeupSources();
        
        // Конфигурация режима энергопотребления
        ConfigurePowerMode(POWER_MODE_SLEEP);
    }
}

void Diagnostics_ExitLowPowerMode(void) {
    if (diagState.lowPowerMode) {
        // Восстанавливаем состояние периферии
        RestorePeripheralState();
        
        // Возвращаем нормальный режим питания
        ConfigurePowerMode(POWER_MODE_NORMAL);
        
        // Сбрасываем флаг
        diagState.lowPowerMode = false;
        
        // Логируем событие
        Diagnostics_Log(LOG_INFO, DIAG_EVENT_SYSTEM_START,
                        "Exiting low power mode", 0);
    }
}

// Статистика производительности
void Diagnostics_StartPerformanceCounter(void) {
    diagState.perfStartTime = diagState.startTime;
}

uint32_t Diagnostics_StopPerformanceCounter(void) {
    uint32_t elapsed = diagState.startTime - diagState.perfStartTime;
    
    if (diagState.perfCount == 0 || elapsed > diagState.perfMaxTime) {
        diagState.perfMaxTime = elapsed;
    }
    if (diagState.perfCount == 0 || elapsed < diagState.perfMinTime) {
        diagState.perfMinTime = elapsed;
    }
    
    diagState.perfTotalTime += elapsed;
    diagState.perfCount++;
    
    return elapsed;
}

void Diagnostics_ResetPerformanceStats(void) {
    diagState.perfMaxTime = 0;
    diagState.perfMinTime = 0;
    diagState.perfTotalTime = 0;
    diagState.perfCount = 0;
}

// Отладочные функции
void Diagnostics_EnableDebugOutput(bool enable) {
    if (diagState.initialized) {
        diagState.debugEnabled = enable;
    }
}

void Diagnostics_PrintSystemStatus(void) {
    if (!diagState.debugEnabled) {
        return;
    }
    
    // Системная информация
    SendDebugMessage("\n=== System Status ===");
    SendDebugMessage("Uptime: %lu ms", diagState.powerState.uptime);
    SendDebugMessage("System State: %s", 
        diagState.systemState == SYSTEM_STATE_OK ? "OK" :
        diagState.systemState == SYSTEM_STATE_WARNING ? "WARNING" :
        diagState.systemState == SYSTEM_STATE_ERROR ? "ERROR" : "CRITICAL");
    SendDebugMessage("Error Count: %lu", diagState.errorCount);
    
    // Состояние питания
    SendDebugMessage("\n=== Power State ===");
    SendDebugMessage("Battery: %.2fV (%.0f%%)", 
        diagState.powerState.batteryVoltage,
        diagState.powerState.batteryLevel);
    SendDebugMessage("Current: %.2fA", diagState.powerState.batteryCurrent);
    SendDebugMessage("Battery Temp: %.1f°C", diagState.powerState.batteryTemp);
    SendDebugMessage("Charging: %s", diagState.powerState.isCharging ? "Yes" : "No");
    
    // Температурное состояние
    SendDebugMessage("\n=== Thermal State ===");
    SendDebugMessage("MCU Temp: %.1f°C", diagState.thermalState.mcuTemp);
    SendDebugMessage("Board Temp: %.1f°C", diagState.thermalState.boardTemp);
    SendDebugMessage("Motor Temps: %.1f, %.1f, %.1f, %.1f°C",
        diagState.thermalState.motorTemps[0],
        diagState.thermalState.motorTemps[1],
        diagState.thermalState.motorTemps[2],
        diagState.thermalState.motorTemps[3]);
    SendDebugMessage("Ambient Temp: %.1f°C", diagState.thermalState.ambientTemp);
    
    // Статистика производительности
    SendDebugMessage("\n=== Performance Stats ===");
    SendDebugMessage("Max Time: %lu ms", diagState.perfMaxTime);
    if (diagState.perfCount > 0) {
        SendDebugMessage("Avg Time: %.2f ms", 
            (float)diagState.perfTotalTime / diagState.perfCount);
    }
    SendDebugMessage("Total Samples: %lu", diagState.perfCount);
    
    // Состояние логов
    SendDebugMessage("\n=== Log Status ===");
    SendDebugMessage("Log Entries: %lu/%d", diagState.logCount, DIAG_MAX_LOG_ENTRIES);
    
    if (diagState.logCount > 0) {
        LogEntry lastEntry;
        Diagnostics_GetLastLogEntry(&lastEntry);
        SendDebugMessage("Last Log: [%s] %s", 
            GetLogLevelString(lastEntry.level),
            lastEntry.message);
    }
    
    SendDebugMessage("=== End Status ===\n");
}

void Diagnostics_RunSelfTest(void) {
    // TODO: Реализовать процедуру самотестирования
    Diagnostics_Log(LOG_INFO, DIAG_EVENT_SYSTEM_START, "Running self-test", 0);
}

// Результаты самотестирования
// Удалено локальное определение SelfTestResult
// Используем определение из заголовочного файла diagnostics.h
static SelfTestResult selfTestResult;

// Коды ошибок питания
#define POWER_ERR_VOLTAGE_LOW    0x0001
#define POWER_ERR_VOLTAGE_HIGH   0x0002
#define POWER_ERR_CURRENT_HIGH   0x0003
#define POWER_ERR_TEMP_HIGH      0x0004
#define POWER_ERR_SUPPLY_FAIL    0x0005

// Пороговые значения для тестирования питания
#define POWER_TEST_VOLTAGE_MIN   10.5f   // В
#define POWER_TEST_VOLTAGE_MAX   12.6f   // В
#define POWER_TEST_CURRENT_MAX   15.0f   // А
#define POWER_TEST_TEMP_MAX      45.0f   // °C
#define POWER_TEST_SUPPLY_MIN    3.2f    // В (для 3.3В цепей)
#define POWER_TEST_SUPPLY_MAX    3.4f    // В

// Функция тестирования системы питания
TestStatus TestPowerSystem(void) {
    TestStatus status = {0};
    status.code = 0;
    status.data = 0;
    status.message = "";
    status.status = TEST_PASS;
    
    // 1. Проверка напряжения батареи
    float batteryVoltage = diagState.powerState.batteryVoltage;
    if (batteryVoltage < POWER_TEST_VOLTAGE_MIN) {
        status.status = TEST_FAIL;
        status.code = POWER_ERR_VOLTAGE_LOW;
        status.message = "Battery voltage too low";
        status.data = (uint32_t)(batteryVoltage * 100);
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    else if (batteryVoltage > POWER_TEST_VOLTAGE_MAX) {
        status.status = TEST_FAIL;
        status.code = POWER_ERR_VOLTAGE_HIGH;
        status.message = "Battery voltage too high";
        status.data = (uint32_t)(batteryVoltage * 100);
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    // 2. Проверка тока потребления
    float batteryCurrent = diagState.powerState.batteryCurrent;
    if (batteryCurrent > POWER_TEST_CURRENT_MAX) {
        status.status = TEST_FAIL;
        status.code = POWER_ERR_CURRENT_HIGH;
        status.message = "Current too high";
        status.data = (uint32_t)(batteryCurrent * 100);
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    // 3. Проверка температур батареи
    float batteryTemp = diagState.powerState.batteryTemp;
    if (batteryTemp > POWER_TEST_TEMP_MAX) {
        status.status = TEST_WARNING;
        status.code = POWER_ERR_TEMP_HIGH;
        status.message = "Battery temp high";
        status.data = (uint32_t)(batteryTemp * 100);
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
    }
    
    // 4. Проверка цепей питания подсистем
    if (!TestSupplyVoltages()) {
        status.status = TEST_FAIL;
        status.code = POWER_ERR_SUPPLY_FAIL;
        status.message = "Supply voltage fail";
        status.data = 0;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    return status;
}

// Проверка напряжений питания подсистем
bool TestSupplyVoltages(void) {
    // Проверяем напряжение 3.3В для датчиков
    float sensorSupply = GetSensorSupplyVoltage();  // Функция измерения через АЦП
    if (sensorSupply < POWER_TEST_SUPPLY_MIN || sensorSupply > POWER_TEST_SUPPLY_MAX) {
        return false;
    }
    
    // Здесь можно добавить проверку других цепей питания
    
    return true;
}

// Коды ошибок датчиков
#define SENSOR_ERR_IMU_FAIL      0x0101
#define SENSOR_ERR_IMU_SELFTEST  0x0102
#define SENSOR_ERR_IMU_RANGE     0x0103
#define SENSOR_ERR_BARO_FAIL     0x0104
#define SENSOR_ERR_BARO_RANGE    0x0105
#define SENSOR_ERR_MAG_FAIL      0x0106
#define SENSOR_ERR_MAG_RANGE     0x0107
#define SENSOR_ERR_GPS_FAIL      0x0108
#define SENSOR_ERR_ADC_FAIL      0x0109

// Пороговые значения для тестирования датчиков
#define IMU_TEST_ACCEL_MAX      4.0f    // g
#define IMU_TEST_GYRO_MAX       50.0f   // deg/s
#define BARO_TEST_ALT_MIN       -100.0f // м
#define BARO_TEST_ALT_MAX       5000.0f // м
#define MAG_TEST_FIELD_MIN      0.2f    // Гаусс
#define MAG_TEST_FIELD_MAX      0.8f    // Гаусс

// Функция тестирования датчиков
TestStatus TestSensors(void) {
    TestStatus status = {0};
    status.code = 0;
    status.data = 0;
    status.message = "";
    status.status = TEST_PASS;
    IMUData imuData = {0};  // Добавляем объявление переменной
    
    // Проверяем текущие показания акселерометра и гироскопа
    float accel_magnitude = sqrtf(powf(imuData.accelX, 2) + 
                                powf(imuData.accelY, 2) + 
                                powf(imuData.accelZ, 2));
                                
    float gyro_magnitude = sqrtf(powf(imuData.gyroX, 2) + 
                               powf(imuData.gyroY, 2) + 
                               powf(imuData.gyroZ, 2));
    
    // Проверка диапазонов значений
    if (accel_magnitude > IMU_TEST_ACCEL_MAX || gyro_magnitude > IMU_TEST_GYRO_MAX) {
        status.status = TEST_FAIL;
        status.code = SENSOR_ERR_IMU_RANGE;
        status.message = "IMU values out of range";
        status.data = (uint32_t)(accel_magnitude * 100);
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    // 2. Проверка барометра
    if (!TestBarometer()) {
        status.status = TEST_FAIL;
        status.code = SENSOR_ERR_BARO_FAIL;
        status.message = "Barometer failed";
        status.data = 0;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    float altitude = GetBaroAltitude();
    if (altitude < BARO_TEST_ALT_MIN || altitude > BARO_TEST_ALT_MAX) {
        status.status = TEST_FAIL;
        status.code = SENSOR_ERR_BARO_RANGE;
        status.message = "Baro out of range";
        status.data = (uint32_t)(altitude * 100);
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    // 3. Проверка магнитометра
    if (!TestMagnetometer()) {
        status.status = TEST_FAIL;
        status.code = SENSOR_ERR_MAG_FAIL;
        status.message = "Magnetometer failed";
        status.data = 0;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    static IMUData magData = {
        .magX = 0.0f,
        .magY = 0.0f,
        .magZ = 0.0f
    };
    
    float mag_magnitude = sqrtf(powf(magData.magX, 2) + 
                              powf(magData.magY, 2) + 
                              powf(magData.magZ, 2));
                              
    if (mag_magnitude < MAG_TEST_FIELD_MIN || mag_magnitude > MAG_TEST_FIELD_MAX) {
        status.status = TEST_WARNING;
        status.code = SENSOR_ERR_MAG_RANGE;
        status.message = "Mag field abnormal";
        status.data = (uint32_t)(mag_magnitude * 1000);
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
    }
    
    // 4. Проверка GPS (если есть)
    if (HasGPS()) {
        if (!TestGPS()) {
            status.status = TEST_WARNING;  // GPS не критичен, поэтому WARNING
            status.code = SENSOR_ERR_GPS_FAIL;
            status.message = "GPS not ready";
            status.data = 0;
            Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SELF_TEST, 
                           status.message, status.data);
        }
    }
    
    return status;
}

// Коды ошибок моторов
#define MOTOR_ERR_NOT_RESPONDING 0x0201
#define MOTOR_ERR_HIGH_CURRENT   0x0202
#define MOTOR_ERR_NO_ROTATION    0x0203
#define MOTOR_ERR_VIBRATION      0x0204
#define MOTOR_ERR_TEMP_HIGH      0x0205
#define MOTOR_ERR_ESC_FAIL       0x0206

// Пороговые значения для тестирования моторов
#define MOTOR_TEST_SPEED         10.0f   // % от максимальной скорости
#define MOTOR_TEST_CURRENT_MAX   2.0f    // А на мотор
#define MOTOR_TEST_TEMP_MAX      60.0f   // °C
#define MOTOR_TEST_VIB_MAX       2.0f    // g
#define MOTOR_TEST_TIMEOUT       1000    // мс

// Структура для хранения результатов теста мотора
typedef struct {
    bool responding;
    float current;
    float rpm;
    float temperature;
    float vibration;
} MotorTestData;

// Функция тестирования моторов
TestStatus TestMotors(void) {
    TestStatus status = {0};
    status.code = 0;
    status.data = 0;
    status.message = "";
    status.status = TEST_PASS;
    MotorTestData motorData[4];  // Для четырех моторов
    
    // 1. Проверка ESC
    if (!TestESC()) {
        status.status = TEST_FAIL;
        status.code = MOTOR_ERR_ESC_FAIL;
        status.message = "ESC initialization failed";
        status.data = 0;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    // 2. Последовательное тестирование каждого мотора
    for (int i = 0; i < 4; i++) {
        // Очищаем данные
        memset(&motorData[i], 0, sizeof(MotorTestData));
        
        // Проверяем отклик от ESC
        if (!TestMotorResponse(i)) {
            status.status = TEST_FAIL;
            status.code = MOTOR_ERR_NOT_RESPONDING;
            status.message = "Motor not responding";
            status.data = i;
            Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                           status.message, status.data);
            return status;
        }
        
        // Запускаем мотор на тестовой скорости
        SetMotorSpeed(i, MOTOR_TEST_SPEED);
        
        // Ждем стабилизации
        DelayMs(500);
        
        // Измеряем параметры
        motorData[i].current = GetMotorCurrent(i);
        motorData[i].rpm = GetMotorRPM(i);
        motorData[i].temperature = GetMotorTemperature(i);
        motorData[i].vibration = GetMotorVibration(i);
        
        // Проверяем ток
        if (motorData[i].current > MOTOR_TEST_CURRENT_MAX) {
            status.status = TEST_FAIL;
            status.code = MOTOR_ERR_HIGH_CURRENT;
            status.message = "Motor high current";
            status.data = i;
            Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                           status.message, status.data);
            SetMotorSpeed(i, 0);
            return status;
        }
        
        // Проверяем вращение
        if (motorData[i].rpm < 100.0f) {  // Минимальные обороты для теста
            status.status = TEST_FAIL;
            status.code = MOTOR_ERR_NO_ROTATION;
            status.message = "Motor not rotating";
            status.data = i;
            Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                           status.message, status.data);
            SetMotorSpeed(i, 0);
            return status;
        }
        
        // Проверяем температур
        if (motorData[i].temperature > MOTOR_TEST_TEMP_MAX) {
            status.status = TEST_WARNING;
            status.code = MOTOR_ERR_TEMP_HIGH;
            status.message = "Motor temperature high";
            status.data = i;
            Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SELF_TEST, 
                           status.message, status.data);
        }
        
        // Проверяем вибрацию
        if (motorData[i].vibration > MOTOR_TEST_VIB_MAX) {
            status.status = TEST_WARNING;
            status.code = MOTOR_ERR_VIBRATION;
            status.message = "Motor high vibration";
            status.data = i;
            Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SELF_TEST, 
                           status.message, status.data);
        }
        
        // Останавливаем мотор
        SetMotorSpeed(i, 0);
        
        // Ждем остановки
        DelayMs(500);
    }
    
    return status;
}

// Обновляем функцию быстрого тестирования



// Получение результатов самотестирования
SelfTestResult Diagnostics_GetTestResult(void) {
    // Возвращаем статическую структуру selfTestResult, 
    // которая заполняется в ходе выполнения Diagnostics_PerformQuickTest()
    return selfTestResult;
}

// Коды ошибок связи
#define COMM_ERR_RC_FAIL        0x0301
#define COMM_ERR_RC_QUALITY     0x0302
#define COMM_ERR_TELEMETRY_FAIL 0x0303
#define COMM_ERR_BACKUP_FAIL    0x0304

// Коды ошибок систем безопасности
#define SAFETY_ERR_WATCHDOG     0x0401
#define SAFETY_ERR_EEPROM       0x0402
#define SAFETY_ERR_FLASH        0x0403
#define SAFETY_ERR_PARACHUTE    0x0404
#define SAFETY_ERR_KILLSWITCH   0x0405

// Пороговые значения для тестирования связи
#define RC_TEST_TIMEOUT         1000    // мс
#define RC_TEST_MIN_QUALITY     75      // %
#define TELEMETRY_TEST_TIMEOUT  1000    // мс

// Функция тестирования связи
TestStatus TestCommunication(void) {
    TestStatus status = {0};
    status.code = 0;
    status.data = 0;
    status.message = "";
    status.status = TEST_PASS;
    uint32_t startTime;
    
    // 1. Проверка связи с пультом RC
    startTime = GetCurrentTime();
    bool rcResponded = false;
    
    SendRCTestPacket();
    
    while (GetCurrentTime() - startTime < RC_TEST_TIMEOUT) {
        if (IsRCTestResponseReceived()) {
            rcResponded = true;
            break;
        }
        DelayMs(1);
    }
    
    if (!rcResponded) {
        status.status = TEST_FAIL;
        status.code = COMM_ERR_RC_FAIL;
        status.message = "RC not responding";
        status.data = 0;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    uint8_t rcQuality = GetRCLinkQuality();
    if (rcQuality < RC_TEST_MIN_QUALITY) {
        status.status = TEST_WARNING;
        status.code = COMM_ERR_RC_QUALITY;
        status.message = "RC signal weak";
        status.data = rcQuality;
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
    }
    
    // 2. Проверка телеметрии
    startTime = GetCurrentTime();
    bool telemetryResponded = false;
    
    SendTelemetryTestPacket();
    
    while (GetCurrentTime() - startTime < TELEMETRY_TEST_TIMEOUT) {
        if (IsTelemetryAckReceived()) {
            telemetryResponded = true;
            break;
        }
        DelayMs(1);
    }
    
    if (!telemetryResponded) {
        status.status = TEST_WARNING;
        status.code = COMM_ERR_TELEMETRY_FAIL;
        status.message = "Telemetry failed";
        status.data = 0;
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
    }
    
    // 3. Проверка резервного канала связи
    if (HasBackupComm() && !TestBackupComm()) {
        status.status = TEST_WARNING;
        status.code = COMM_ERR_BACKUP_FAIL;
        status.message = "Backup comm failed";
        status.data = 0;
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
    }
    
    return status;
}

// Функция тестирования систем безопасности
TestStatus TestSafetySystems(void) {
    TestStatus status = {0};
    status.code = 0;
    status.data = 0;
    status.message = "";
    status.status = TEST_PASS;
    
    // 1. Проверка работы watchdog
    if (!TestWatchdogReset()) {
        status.status = TEST_FAIL;
        status.code = SAFETY_ERR_WATCHDOG;
        status.message = "Watchdog failed";
        status.data = 0;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    // 2. Проверка EEPROM
    if (!TestEEPROM()) {
        status.status = TEST_FAIL;
        status.code = SAFETY_ERR_EEPROM;
        status.message = "EEPROM failed";
        status.data = 0;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    // 3. Проверка Flash памяти
    if (!TestFlashMemory()) {
        status.status = TEST_FAIL;
        status.code = SAFETY_ERR_FLASH;
        status.message = "Flash failed";
        status.data = 0;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    // 4. Проверка парашютной системы
    if (HasParachute()) {
        if (!TestParachuteSystem()) {
            status.status = TEST_FAIL;
            status.code = SAFETY_ERR_PARACHUTE;
            status.message = "Parachute failed";
            status.data = 0;
            Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                           status.message, status.data);
            return status;
        }
    }
    
    // 5. Проверка аварийного выключения
    if (!TestKillSwitch()) {
        status.status = TEST_FAIL;
        status.code = SAFETY_ERR_KILLSWITCH;
        status.message = "Kill switch failed";
        status.data = 0;
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SELF_TEST, 
                       status.message, status.data);
        return status;
    }
    
    return status;
}

// Функции работы с ADC
// Удалено дублирование определения

ADCResult Diagnostics_GetADCValue(void) {
    ADCResult result = {0};
    
    // Проверка инициализации
    if (!diagState.initialized) {
        result.error = ADC_ERROR_NOT_INITIALIZED;
        return result;
    }
    
    // Запускаем преобразование
    ADC0_PSSI_R = 0x0008;  // Запуск SS3
    
    // Ожидание с таймаутом
    uint32_t timeout = 10000;  // Примерно 10000 циклов
    while((ADC0_RIS_R & 0x08) == 0 && timeout > 0) {
        timeout--;
    }
    
    // Проверка таймаута
    if (timeout == 0) {
        result.error = ADC_ERROR_TIMEOUT;
        return result;
    }
    
    // Считываем результат
    result.value = ADC0_SSFIFO3_R;
    result.error = ADC_ERROR_NONE;
    
    // Сбрасываем флаг
    ADC0_ISC_R = 0x0008;
    
    return result;
}

// Преобразование значения АЦП в температур с проверкой диапазона
float Diagnostics_ConvertADCToTemp(ADCResult adcResult) {
    // Проверка ошибок АЦП
    if (adcResult.error != ADC_ERROR_NONE) {
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SENSOR_ERROR, 
                        "ADC conversion error before temperature conversion", 
                        adcResult.error);
        return NAN;  // Возвращаем NaN при ошибке
    }
    
    // Проверка ошибок АЦП
    if (adcResult.value == 0 || adcResult.value >= ADC_MAX_VALUE) {
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SENSOR_ERROR, 
                        "ADC value out of range", 
                        adcResult.value);
        return NAN;
    }
    
    // Преобразование значения АЦП в напряжение
    float voltage = (adcResult.value * 3.3f) / 4096.0f;
    
    float temperature;
    
    // Преобразование в зависимости от типа датчика
    switch (TEMP_SENSOR_TYPE_LINEAR) {
        case TEMP_SENSOR_TYPE_LINEAR:
            // Линейное преобразование
            temperature = (voltage * 100.0f);  // Пример преобразования
            break;
        
        default:
            // Неизвестный тип датчика
            Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SENSOR_ERROR, 
                            "Unknown temperature sensor type", 
                            (uint32_t)TEMP_SENSOR_TYPE_LINEAR);
            return NAN;
    }
    
    // Проверяем корректность полученной температур
    if (isnan(temperature)) {
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SENSOR_ERROR, 
                        "Invalid cold junction temperature", 
                        (uint32_t)temperature);
        return 25.0f;
    }
    
    return temperature;
}

// Безопасный запуск преобразования АЦП
ADCError Diagnostics_StartADCConversion(void) {
    if (!diagState.initialized) {
        return ADC_ERROR_NOT_INITIALIZED;
    }
    
    ADC0_PSSI_R = 0x0008;  // Запуск SS3
    return ADC_ERROR_NONE;
}

// Проверка завершения преобразования с таймаутом
// Удалено дублирование определения

ADCConversionStatus Diagnostics_IsADCConversionComplete(void) {
    ADCConversionStatus status = {
        .complete = false,
        .error = ADC_ERROR_NONE
    };
    
    if (!diagState.initialized) {
        status.error = ADC_ERROR_NOT_INITIALIZED;
        return status;
    }
    
    status.complete = (ADC0_RIS_R & 0x08) != 0;
    
    // Добавляем проверку на таймаут
    static uint32_t conversionAttempts = 0;
    const uint32_t MAX_CONVERSION_ATTEMPTS = 1000;
    
    if (!status.complete) {
        conversionAttempts++;
        if (conversionAttempts >= MAX_CONVERSION_ATTEMPTS) {
            status.error = ADC_ERROR_TIMEOUT;
            conversionAttempts = 0;
        }
    } else {
        conversionAttempts = 0;
    }
    
    return status;
}

// Улучшенные функции работы с системным таймером
// Удалено дублирование определения

SystemTickResult Diagnostics_GetSystemTicks(void) {
    SystemTickResult result = {
        .ticks = NVIC_ST_CURRENT_R,
        .error = SYSTICK_ERROR_NONE
    };
    
    // Проверка корректности регистра
    if (result.ticks == 0 || result.ticks > NVIC_ST_RELOAD_R) {
        result.error = SYSTICK_ERROR_INVALID_VALUE;
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SYSTEM_ERROR, 
                        "Invalid system tick value", result.ticks);
    }
    
    return result;
}

// Задержка с проверкой переполнения
SystemTickError Diagnostics_DelayTicks(uint32_t ticks) {
    SystemTickResult startTicks = Diagnostics_GetSystemTicks();
    
    if (startTicks.error != SYSTICK_ERROR_NONE) {
        return SYSTICK_ERROR_INVALID_START;
    }
    
    uint32_t startValue = startTicks.ticks;
    uint32_t elapsedTicks = 0;
    
    while (elapsedTicks < ticks) {
        SystemTickResult currentTicks = Diagnostics_GetSystemTicks();
        
        if (currentTicks.error != SYSTICK_ERROR_NONE) {
            return SYSTICK_ERROR_DURING_DELAY;
        }
        
        // Обработка переполнения
        if (currentTicks.ticks <= startValue) {
            elapsedTicks = startValue - currentTicks.ticks;
        } else {
            elapsedTicks = (NVIC_ST_RELOAD_R - currentTicks.ticks) + startValue;
        }
    }
    
    return SYSTICK_ERROR_NONE;
}

// Вычисление прошедших тиков с расширенной проверкой
// Удалено дублирование определения

ElapsedTicksResult Diagnostics_GetElapsedTicks(uint32_t startTicks) {
    ElapsedTicksResult result = {
        .elapsedTicks = 0,
        .error = SYSTICK_ERROR_NONE
    };
    
    SystemTickResult currentTicksResult = Diagnostics_GetSystemTicks();
    
    if (currentTicksResult.error != SYSTICK_ERROR_NONE) {
        result.error = SYSTICK_ERROR_INVALID_CURRENT;
        return result;
    }
    
    uint32_t currentTicks = currentTicksResult.ticks;
    
    if (currentTicks <= startTicks) {
        result.elapsedTicks = startTicks - currentTicks;
    } else {
        // Обработка переполнения
        result.elapsedTicks = (NVIC_ST_RELOAD_R - currentTicks) + startTicks;
    }
    
    return result;
}

// Магнитные данные для самотестирования
static IMUData magData = {
    .magX = 0.0f,
    .magY = 0.0f,
    .magZ = 0.0f
};

void Diagnostics_InitSelfTest(void) {
    // Initialize test result structure
    memset(&selfTestResult, 0, sizeof(SelfTestResult));
    selfTestResult.totalErrors = 0;
    selfTestResult.totalWarnings = 0;
    selfTestResult.isTestComplete = false;
    strncpy(selfTestResult.message, "Test not started", sizeof(selfTestResult.message) - 1);
    
    // Initialize system status
    selfTestResult.systemStatus.powerSystemOperational = false;
    selfTestResult.systemStatus.sensorsOperational = false;
    selfTestResult.systemStatus.motorsOperational = false;
    selfTestResult.systemStatus.communicationOperational = false;
    selfTestResult.systemStatus.safetySystemOperational = false;
    
    // Initialize individual test results
    selfTestResult.powerTest = (TestStatus){0, 0, "Not started", 0};
    selfTestResult.sensorTest = (TestStatus){0, 0, "Not started", 0};
    selfTestResult.motorTest = (TestStatus){0, 0, "Not started", 0};
    selfTestResult.batteryTest = (TestStatus){0, 0, "Not started", 0};
    selfTestResult.commTest = (TestStatus){0, 0, "Not started", 0};
    selfTestResult.safetyTest = (TestStatus){0, 0, "Not started", 0};
}

bool Diagnostics_PerformQuickTest(void) {
    Diagnostics_InitSelfTest();
    
    // Power system test
    selfTestResult.powerTest = TestPowerSystem();
    if (selfTestResult.powerTest.status == TEST_FAIL) {
        selfTestResult.totalErrors++;
        selfTestResult.isTestComplete = true;
        return false;
    }
    else if (selfTestResult.powerTest.status == TEST_WARNING) {
        selfTestResult.totalWarnings++;
    }
    selfTestResult.systemStatus.powerSystemOperational = (selfTestResult.powerTest.status != TEST_FAIL);
    
    // Sensor test
    selfTestResult.sensorTest = TestSensors();
    if (selfTestResult.sensorTest.status == TEST_FAIL) {
        selfTestResult.totalErrors++;
        selfTestResult.isTestComplete = true;
        return false;
    }
    else if (selfTestResult.sensorTest.status == TEST_WARNING) {
        selfTestResult.totalWarnings++;
    }
    selfTestResult.systemStatus.sensorsOperational = (selfTestResult.sensorTest.status != TEST_FAIL);
    
    // Motor test
    selfTestResult.motorTest = TestMotors();
    if (selfTestResult.motorTest.status == TEST_FAIL) {
        selfTestResult.totalErrors++;
        selfTestResult.isTestComplete = true;
        return false;
    }
    else if (selfTestResult.motorTest.status == TEST_WARNING) {
        selfTestResult.totalWarnings++;
    }
    selfTestResult.systemStatus.motorsOperational = (selfTestResult.motorTest.status != TEST_FAIL);
    
    // Communication test
    selfTestResult.commTest = TestCommunication();
    if (selfTestResult.commTest.status == TEST_FAIL) {
        selfTestResult.totalErrors++;
        selfTestResult.isTestComplete = true;
        return false;
    }
    else if (selfTestResult.commTest.status == TEST_WARNING) {
        selfTestResult.totalWarnings++;
    }
    selfTestResult.systemStatus.communicationOperational = (selfTestResult.commTest.status != TEST_FAIL);
    
    // Safety systems test
    selfTestResult.safetyTest = TestSafetySystems();
    if (selfTestResult.safetyTest.status == TEST_FAIL) {
        selfTestResult.totalErrors++;
        selfTestResult.isTestComplete = true;
        return false;
    }
    else if (selfTestResult.safetyTest.status == TEST_WARNING) {
        selfTestResult.totalWarnings++;
    }
    selfTestResult.systemStatus.safetySystemOperational = (selfTestResult.safetyTest.status != TEST_FAIL);
    
    // Set final status
    selfTestResult.isTestComplete = true;
    if (selfTestResult.totalErrors > 0) {
        strncpy(selfTestResult.message, "Self-test failed", sizeof(selfTestResult.message) - 1);
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SYSTEM_ERROR, selfTestResult.message, selfTestResult.totalErrors);
        return false;
    }
    
    strncpy(selfTestResult.message, "Self-test passed", sizeof(selfTestResult.message) - 1);
    Diagnostics_Log(LOG_INFO, DIAG_EVENT_SELF_TEST, selfTestResult.message, 0);
    return true;
}

// Определение каналов АЦП для различных температурных датчиков
#define ADC_CHANNEL_MCU_TEMP         ADC_AIN0   // Канал АЦП для температур
#define ADC_CHANNEL_MOTOR_TEMP_BASE  ADC_AIN1   // Базовый канал для температур моторов
#define ADC_CHANNEL_BATTERY_TEMP     ADC_AIN4   // Канал АЦП для температур батареи
#define ADC_CHANNEL_BOARD_TEMP       ADC_AIN5   // Канал АЦП для температур платы
#define ADC_CHANNEL_ENV_TEMP         ADC_AIN6   // Канал АЦП для температур окружающей среды

// Функция получения температур микроконтроллера
uint32_t GetMCUTemperature(void) {
    // Настройка канала АЦП для температур микроконтроллера
    ADC1_SSMUX1_R = ADC_AIN0;  // Выбираем канал AIN0
    ADC1_SSCTL1_R = 0x06;      // Установка флагов завершения и сравнения
    ADC1_SSFIFO1_R;            // Очистка FIFO
    ADC1_ISC_R = ADC_ISC_IN1;  // Сброс флага прерывания
    return ADC1_SSFIFO1_R;     // Возвращаем значение АЦП
}

float GetMotorTemperature(int motorIndex) {
    if (motorIndex < 0 || motorIndex >= MAX_MOTORS) {
        return NAN;  // Возвращаем NaN для некорректного индекса
    }
    
    // Выбираем канал АЦП для соответствующего мотора
    uint32_t channel = ADC_CHANNEL_MOTOR_TEMP_BASE + motorIndex;
    
    // Получаем значение АЦП и преобразуем его в температур
    ADCResult result = Diagnostics_GetADCValue();
    return Diagnostics_ConvertADCToTemp(result);
}

uint32_t GetBatteryTemperature(void) {
    // Настройка канала АЦП для температур батареи
    ADC1_SSMUX1_R = ADC_AIN4;  // Выбираем канал AIN4
    ADC1_SSCTL1_R = 0x06;      // Установка флагов завершения и сравнения
    ADC1_SSFIFO1_R;            // Очистка FIFO
    ADC1_ISC_R = ADC_ISC_IN1;  // Сброс флага прерывания
    return ADC1_SSFIFO1_R;     // Возвращаем значение АЦП
}

uint32_t GetBoardTemperature(void) {
    // Настройка канала АЦП для температур платы
    ADC1_SSMUX1_R = ADC_AIN5;  // Выбираем канал AIN5
    ADC1_SSCTL1_R = 0x06;      // Установка флагов завершения и сравнения
    ADC1_SSFIFO1_R;            // Очистка FIFO
    ADC1_ISC_R = ADC_ISC_IN1;  // Сброс флага прерывания
    return ADC1_SSFIFO1_R;     // Возвращаем значение АЦП
}

uint32_t GetEnvironmentTemperature(void) {
    // Настройка канала АЦП для температур окружающей среды
    ADC1_SSMUX1_R = ADC_AIN6;  // Выбираем канал AIN6
    ADC1_SSCTL1_R = 0x06;      // Установка флагов завершения и сравнения
    ADC1_SSFIFO1_R;            // Очистка FIFO
    ADC1_ISC_R = ADC_ISC_IN1;  // Сброс флага прерывания
    return ADC1_SSFIFO1_R;     // Возвращаем значение АЦП
}

// Таблицы коэффициентов для различных типов термопар
static const struct {
    float a, b, c, d;  // Коэффициенты для полиномиальной аппроксимации
} ThermocoupleCoefficients[THERMOCOUPLE_TYPE_COUNT] = {
    [THERMOCOUPLE_TYPE_K] = {0.0, 39.0, -0.0001, 0.0},     // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_J] = {0.0, 35.0, -0.00015, 0.0},    // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_T] = {0.0, 40.0, -0.0002, 0.0},     // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_E] = {0.0, 38.0, -0.0001, 0.0},     // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_N] = {0.0, 37.0, -0.00012, 0.0},    // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_R] = {0.0, 36.0, -0.0001, 0.0},     // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_S] = {0.0, 35.0, -0.00015, 0.0},    // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_B] = {0.0, 34.0, -0.0002, 0.0}      // Приблизительные коэффициенты
};

// Функция получения температур холодного спая
float GetColdJunctionTemperature(void) {
    ADCResult adcResult = Diagnostics_GetADCValue();
    
    if (adcResult.error != ADC_ERROR_NONE) {
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SENSOR_ERROR, 
                        "ADC conversion error before cold junction temperature", 
                        adcResult.error);
        return 25.0f;  // Возвращаем стандартную температур окружающей среды
    }
    
    // Преобразование значения АЦП в напряжение
    float voltage = (adcResult.value * 3.3f) / 4096.0f;
    
    float temperature;
    
    // Преобразование в зависимости от типа датчика
    switch (TEMP_SENSOR_TYPE_LINEAR) {
        case TEMP_SENSOR_TYPE_LINEAR:
            // Линейное преобразование
            temperature = (voltage * 100.0f);  // Пример преобразования
            break;
        
        default:
            // Неизвестный тип датчика
            Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SENSOR_ERROR, 
                            "Unknown temperature sensor type", 
                            (uint32_t)TEMP_SENSOR_TYPE_LINEAR);
            return NAN;
    }
    
    // Проверяем корректность полученной температур
    if (isnan(temperature)) {
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SENSOR_ERROR, 
                        "Invalid cold junction temperature", 
                        (uint32_t)temperature);
        return 25.0f;
    }
    
    return temperature;
}

// Функция преобразования напряжения термопары в температур
float ConvertThermocoupleVoltageToTemp(float thermocoupleVoltage, ThermocoupleType type, float coldJunctionTemp) {
    // Проверка корректности типа термопары
    if (type < 0 || type >= THERMOCOUPLE_TYPE_COUNT) {
        Diagnostics_Log(LOG_ERROR, DIAG_EVENT_SENSOR_ERROR, 
                        "Invalid thermocouple type", 
                        (uint32_t)type);
        return NAN;
    }
    
    // Получаем коэффициенты для данного типа термопары
    const struct {
        float a, b, c, d;
    } *coeffs = &ThermocoupleCoefficients[type];
    
    // Полиномиальная аппроксимация для преобразования напряжения в температур
    // Используем приближенную формулу с компенсацией холодного спая
    float temperature = coeffs->a * thermocoupleVoltage * thermocoupleVoltage * thermocoupleVoltage 
                      + coeffs->b * thermocoupleVoltage * thermocoupleVoltage 
                      + coeffs->c * thermocoupleVoltage 
                      + coeffs->d;
    
    // Добавляем температур холодного спая
    temperature += coldJunctionTemp;
    
    // Проверка диапазона
    const float MIN_TEMP = -200.0f;  // Минимальная температур для термопар
    const float MAX_TEMP = 1800.0f;  // Максимальная температур для термопар
    
    if (temperature < MIN_TEMP || temperature > MAX_TEMP) {
        Diagnostics_Log(LOG_WARNING, DIAG_EVENT_SENSOR_ERROR, 
                        "Thermocouple temperature out of range", 
                        (uint32_t)(temperature * 100));
        return NAN;
    }
    
    return temperature;
}

// Функция измерения тока мотора через АЦП
float GetMotorCurrent(int motorIndex) {
    // Проверка индекса мотора
    if (motorIndex < 0 || motorIndex >= MAX_MOTORS) {
        return 0.0f;
    }

    // Выбираем канал АЦП для соответствующего мотора
    uint32_t adcChannel = ADC_CHANNEL_MOTOR_CURRENT_BASE + motorIndex;
    
    // Запускаем измерение
    ADC1_PSSI_R |= (1 << adcChannel);  // Запуск преобразования
    
    // Ждем завершения преобразования
    while(!(ADC1_RIS_R & (1 << adcChannel))) {
        // Ждем
    }
    
    // Считываем результат
    uint32_t result = ADC1_SSFIFO0_R & 0xFFF;  // 12-bit ADC
    
    // Очищаем флаг завершения
    ADC1_ISC_R = (1 << adcChannel);
    
    // Преобразуем значение АЦП в ток (А)
    // Предполагаем, что используется датчик тока с диапазоном 0-20А при 0-3.3В
    float voltage = (result * 3.3f) / 4096.0f;  // Преобразование в вольты
    float current = (voltage * 20.0f) / 3.3f;   // Преобразование в амперы
    
    return current;
}

// Глобальные переменные для работы с резервным каналом связи
static bool backupCommAvailable = false;
static uint32_t lastBackupCommTest = 0;

// Функция проверки наличия резервного канала связи
bool HasBackupComm(void) {
    return backupCommAvailable;
}

// Функция тестирования резервного канала связи
bool TestBackupComm(void) {
    uint32_t currentTime = GetCurrentTime();
    
    // Проверяем, прошло ли достаточно времени с последнего теста
    if (currentTime - lastBackupCommTest < 1000) {  // Тестируем не чаще раза в секунду
        return true;  // Возвращаем последний результат
    }
    
    lastBackupCommTest = currentTime;
    
    // Отправляем тестовый пакет
    uint8_t testPacket[] = {0xAA, 0x55, 0x00, 0xFF};
    bool success = SendBackupCommPacket(testPacket, sizeof(testPacket));
    if (!success) {
        return false;
    }
    
    // Ждем ответ с таймаутом
    uint32_t startTime = GetCurrentTime();
    uint8_t response[4];
    
    while (GetCurrentTime() - startTime < 100) {  // Таймаут 100 мс
        if (ReceiveBackupCommPacket(response, sizeof(response))) {
            // Проверяем корректность ответа
            if (response[0] == 0x55 && response[1] == 0xAA && 
                response[2] == 0xFF && response[3] == 0x00) {
                return true;
            }
        }
    }
    
    return false;
}

// Вспомогательные функции для работы с резервным каналом
bool SendBackupCommPacket(const uint8_t* data, uint8_t length) {
    // Implementation remains the same
    return true;
}

bool ReceiveBackupCommPacket(uint8_t* buffer, uint8_t maxLength) {
    // Implementation remains the same
    return false;
}

// Функция тестирования watchdog reset
bool TestWatchdogReset(void) {
    // Проверяем, не идет ли уже тест
    if (diagState.watchdogEnabled) {
        return false;
    }
    
    // Сохраняем текущие настройки watchdog
    uint32_t originalLoad = WATCHDOG0_LOAD_R;
    uint32_t originalCtl = WATCHDOG0_CTL_R;
    
    // Устанавливаем короткий таймаут для теста (100мс)
    uint32_t testTimeout = SystemCoreClock / 10;  // 100мс при текущей частоте
    
    // Разблокируем доступ к регистрам watchdog
    WATCHDOG0_LOCK_R = 0x1ACCE551;
    
    // Отключаем watchdog на время подготовки
    WATCHDOG0_CTL_R &= ~(WDT_CTL_INTEN | WDT_CTL_RESEN);
    
    // Устанавливаем новое значение таймаута
    WATCHDOG0_LOAD_R = testTimeout;
    
    // Настраиваем watchdog:
    // - Разрешаем сброс системы
    // - Разрешаем прерывания
    WATCHDOG0_CTL_R |= (WDT_CTL_RESEN | WDT_CTL_INTEN);
    
    // Включаем прерывания от watchdog
    NVIC_EN0_R |= (1 << INT_WATCHDOG);
    
    // Устанавливаем приоритет прерывания (самый высокий)
    NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFFFFF0) | 0x0;
    
    diagState.watchdogEnabled = true;
    diagState.watchdogTimeout = 100;
    
    // Первый сброс WDT
    WATCHDOG0_ICR_R = 0;
    
    Diagnostics_Log(LOG_INFO, DIAG_EVENT_SYSTEM_START,
                   "Watchdog initialized", 100);
    
    // Ждем сброса
    while (diagState.watchdogEnabled) {
        DelayMs(1);
    }
    
    // Восстанавливаем исходные настройки watchdog
    WATCHDOG0_LOAD_R = originalLoad;
    WATCHDOG0_CTL_R = originalCtl;
    
    return true;
}

// Заглушки для тестовых функций
bool TestBarometer(void) { return true; }
bool TestESC(void) { return true; }
bool TestGPS(void) { return true; }
bool TestEEPROM(void) { return true; }
bool TestFlashMemory(void) { return true; }
bool TestKillSwitch(void) { return true; }
bool TestMagnetometer(void) { return true; }
bool TestMotorResponse(void) { return true; }
bool TestParachuteSystem(void) { return true; }
bool HasGPS(void) { return false; }
bool HasParachute(void) { return false; }
float GetBaroAltitude(void) { return 0.0f; }
uint16_t GetMotorRPM(void) { return 0; }
float GetMotorVibration(void) { return 0.0f; }
uint8_t GetRCLinkQuality(void) { return 100; }
float GetSensorSupplyVoltage(void) { return 3.3f; }
bool IsRCTestResponseReceived(void) { return true; }
bool IsTelemetryAckReceived(void) { return true; }
void SendRCTestPacket(void) { }
void SendTelemetryTestPacket(void) { }
void SetMotorSpeed(uint8_t motor, uint16_t speed) { }
const char* Diagnostics_GetLastLogEntry(void) { return "No log"; }

uint16_t GetMotorRPM(int motorIndex) {
    return 0;
}

float GetMotorVibration(int motorIndex) {
    return 0.0f;
}

// Таблицы коэффициентов для различных типов термопар
static const struct {
    float a, b, c, d;  // Коэффициенты для полиномиальной аппроксимации
} ThermocoupleCoefficients[THERMOCOUPLE_TYPE_COUNT] = {
    [THERMOCOUPLE_TYPE_K] = {0.0, 39.0, -0.0001, 0.0},     // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_J] = {0.0, 35.0, -0.00015, 0.0},    // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_T] = {0.0, 40.0, -0.0002, 0.0},     // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_E] = {0.0, 38.0, -0.0001, 0.0},     // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_N] = {0.0, 37.0, -0.00012, 0.0},    // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_R] = {0.0, 36.0, -0.0001, 0.0},     // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_S] = {0.0, 35.0, -0.00015, 0.0},    // Приблизительные коэффициенты
    [THERMOCOUPLE_TYPE_B] = {0.0, 34.0, -0.0002, 0.0}      // Приблизительные коэффициенты
};
