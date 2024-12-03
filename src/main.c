#include "include/main.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "include/sensors.h"
#include "include/navigation.h"
#include "include/stabilization.h"
#include "include/communication.h"
#include "include/telemetry.h"

// Объявляем системные функции как внешние
extern uint32_t SystemCoreClock;
extern void SystemInit(void);

// Глобальные переменные состояния
static DroneState currentState = DRONE_STATE_INIT;

// Базовые адреса GPIO портов
#define GPIO_PORTA_BASE        0x40004000
#define GPIO_PORTB_BASE        0x40005000
#define GPIO_PORTC_BASE        0x40006000
#define GPIO_PORTD_BASE        0x40007000
#define GPIO_PORTE_BASE        0x40024000
#define GPIO_PORTF_BASE        0x40025000

// Определения для регистров системного управления
#define SYSCTL_RCC2_R            (*((volatile uint32_t *)0x400FE070))
#define SYSCTL_RIS_R             (*((volatile uint32_t *)0x400FE050))
#define SYSCTL_RCGC2_R          (*((volatile uint32_t *)0x400FE108))

// Битовые маски для RCC2
#define SYSCTL_RCC2_USERCC2     0x80000000  // Use RCC2
#define SYSCTL_RCC2_BYPASS2     0x00000800  // PLL Bypass 2
#define SYSCTL_RCC2_PWRDN2      0x00002000  // Power-Down PLL 2
#define SYSCTL_RCC2_SYSDIV2_MASK 0x1F800000  // System Clock Divisor 2
#define SYSCTL_RCC2_SYSDIV2LSB   0x00400000  // System Clock Divisor 2 LSB

// Инвертированные маски для очистки битов
#define SYSCTL_RCC2_OSCSRC2_MASK_INV 0xFFFFFF8F  // Инвертированная маска для OSCSRC2
#define SYSCTL_RCC2_PWRDN2_MASK_INV  0xFFFFDFFF  // Инвертированная маска для PWRDN2
#define SYSCTL_RCC2_SYSDIV_MASK_INV  0xE07FFFFF  // Инвертированная маска для SYSDIV2 и LSB

// Биты источника тактирования
#define SYSCTL_RCC2_OSCSRC2_MASK 0x00000070  // Oscillator Source 2
#define SYSCTL_RCC2_OSCSRC2_MO   0x00000000  // Main Oscillator

// Биты статуса PLL
#define SYSCTL_RIS_PLLLRIS      0x00000040  // PLL Lock Raw Interrupt Status

// System Control определения
#define SYSCTL_PERIPH_GPIOF    0xf0000805  // GPIO F peripheral
#define SYSCTL_PERIPH_GPIOE    0xf0000804  // GPIO E peripheral
#define SYSCTL_PERIPH_GPIOD    0xf0000803  // GPIO D peripheral
#define SYSCTL_PERIPH_GPIOC    0xf0000802  // GPIO C peripheral
#define SYSCTL_PERIPH_GPIOB    0xf0000801  // GPIO B peripheral
#define SYSCTL_PERIPH_GPIOA    0xf0000800  // GPIO A peripheral

// Регистры GPIO порта F
#define GPIO_PORTF_DIR_R       (*((volatile uint32_t *)(GPIO_PORTF_BASE + 0x400)))
#define GPIO_PORTF_DEN_R       (*((volatile uint32_t *)(GPIO_PORTF_BASE + 0x51C)))
#define GPIO_PORTF_DATA_R      (*((volatile uint32_t *)(GPIO_PORTF_BASE + 0x3FC)))

// Параметры системы
#define MAIN_LOOP_FREQ_HZ      200     // Частота основного цикла (Гц)
#define TELEMETRY_FREQ_HZ      10      // Частота отправки телеметрии (Гц)
#define DIAGNOSTIC_FREQ_HZ      1       // Частота диагностики (Гц)
#define WDT_TIMEOUT_MS         1000    // Таймаут watchdog (мс)

// Глобальные переменные
static uint32_t mainLoopCounter = 0;
static bool isEmergencyMode = false;

// Структуры данных для хранения текущего состояния
static SensorData sensorData;

// Проверка готовности периферии
bool IsPeripheralReady(uint32_t peripheral) {
    return (SYSCTL_RCGC2_R & (peripheral & 0x000000FF)) != 0;
}

// Включение периферии
void EnablePeripheral(uint32_t peripheral) {
    SYSCTL_RCGC2_R |= (peripheral & 0x000000FF);
    // Ждем готовности периферии
    while(!IsPeripheralReady(peripheral)) {}
}

// Локальные функции
static void InitializeGPIO(void);
static void UpdateWatchdog(void);
static void HandleLEDs(void);
static void UpdateTelemetry(void);

// Основная функция программы
int main(void) {
    // Инициализация системы
    SystemInit();
    
    // Основной цикл
    while (1) {
        // Сброс watchdog
        UpdateWatchdog();
        
        // Обновление состояния датчиков
        if (!Sensors_Update()) {
            EnterEmergencyMode("Sensor update failed");
            continue;
        }
        
        // Получение данных с датчиков
        Sensors_GetAllData(&sensorData);
        
        // Обработка команд и обновление состояния
        ProcessCommands();
        
        // Обновление навигации и стабилизации
        if (currentState >= DRONE_STATE_ARMED) {
            // Обновляем навигацию
            Navigation_Update();
            
            // Проверяем состояние навигации
            if (!Navigation_IsActive()) {
                EnterEmergencyMode("Navigation update failed");
                continue;
            }
            
            // Обновляем стабилизацию
            Stabilization_Update();
            if (Stabilization_IsEnabled() == false) {
                EnterEmergencyMode("Stabilization update failed");
                continue;
            }
            
            // Обновляем моторы
            ActuatorsState actState;
            Actuators_GetState(&actState);
            if (actState.all_motors_enabled == false) {
                EnterEmergencyMode("Motors not ready");
                continue;
            }
        }
        
        // Обновление телеметрии с заданной частотой
        if (mainLoopCounter % (MAIN_LOOP_FREQ_HZ / TELEMETRY_FREQ_HZ) == 0) {
            UpdateTelemetry();
        }
        
        // Проверка безопасности с заданной частотой
        if (mainLoopCounter % (MAIN_LOOP_FREQ_HZ / DIAGNOSTIC_FREQ_HZ) == 0) {
            CheckSafety();
        }
        
        mainLoopCounter++;
    }
}

// Инициализация GPIO
static void InitializeGPIO(void) {
    // Включение тактирования порта F
    EnablePeripheral(SYSCTL_PERIPH_GPIOF);
    
    // Ожидание готовности порта
    while(!IsPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
    
    // Настройка пинов 1-3 порта F как выходов
    GPIO_PORTF_DIR_R |= (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    
    // Включение цифровой функции для пинов 1-3
    GPIO_PORTF_DEN_R |= (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

// Обновление watchdog
static void UpdateWatchdog(void) {
    Diagnostics_FeedWatchdog();
}

// Управление LED индикацией
static void HandleLEDs(void) {
    // Маски для работы с LED
    #define LED_MASK_CLEAR     0xFFFFFFF9  // 1111 1111 1111 1111 1111 1111 1111 1001
    #define LED_RED            0x00000002  // Красный LED (PIN_1)
    #define LED_BLUE           0x00000004  // Синий LED (PIN_2)
    #define LED_GREEN          0x00000008  // Зеленый LED (PIN_3)

    uint32_t ledState = GPIO_PORTF_DATA_R & LED_MASK_CLEAR;  // Очищаем биты LED
    
    // Устанавливаем состояние LED в зависимости от состояния дрона
    switch (currentState) {
        case DRONE_STATE_INIT:
            ledState |= LED_BLUE;  // Синий - инициализация
            break;
            
        case DRONE_STATE_IDLE:
            ledState |= LED_GREEN;  // Зеленый - готов к работе
            break;
            
        case DRONE_STATE_CALIBRATING:
            ledState |= (LED_GREEN | LED_BLUE);  // Бирюзовый - калибровка
            break;
            
        case DRONE_STATE_ARMED:
            ledState |= LED_RED;  // Красный - вооружен
            break;
            
        case DRONE_STATE_TAKEOFF:
        case DRONE_STATE_FLYING:
        case DRONE_STATE_LANDING:
            ledState |= (LED_RED | LED_GREEN);  // Желтый - в полете
            break;
            
        case DRONE_STATE_EMERGENCY:
            // Мигающий красный в экстренном режиме
            if ((mainLoopCounter % 10) < 5) {
                ledState |= LED_RED;
            }
            break;
    }
    
    GPIO_PORTF_DATA_R = ledState;
}

// Обработка входящих команд
void ProcessCommands(void) {
    CommPacket rxPacket;
    CommandPacket cmdData;
    
    // Проверка наличия новых команд
    if (Communication_Receive(&rxPacket)) {
        // Проверяем, что это пакет команды
        if (rxPacket.type == PACKET_TYPE_COMMAND) {
            // Копируем данные из пакета в структуру команды
            if (rxPacket.length == sizeof(CommandPacket)) {
                memcpy(&cmdData, rxPacket.data, sizeof(CommandPacket));
                
                // Обрабатываем команду
                switch (cmdData.type) {
                    case CMD_TYPE_POSITION:
                        if (currentState == DRONE_STATE_FLYING) {
                            // Создаем новую точку маршрута
                            Waypoint targetPoint = {
                                .position = {
                                    .latitude = cmdData.data.position.x,
                                    .longitude = cmdData.data.position.y,
                                    .altitude = cmdData.data.position.z,
                                    .valid = true
                                },
                                .speed = NAV_DEFAULT_SPEED,
                                .hover_time = 0,
                                .reached = false
                            };
                            
                            // Очищаем текущие точки и добавляем новую
                            Navigation_ClearWaypoints();
                            Navigation_AddWaypoint(&targetPoint);
                        }
                        break;
                        
                    case CMD_TYPE_ATTITUDE:
                        if (currentState >= DRONE_STATE_ARMED) {
                            Stabilization_SetTargetOrientation(
                                cmdData.data.attitude.roll,
                                cmdData.data.attitude.pitch,
                                cmdData.data.attitude.yaw,
                                0.0f  // Сохраняем текущую высоту
                            );
                        }
                        break;
                        
                    case CMD_TYPE_CONFIG:
                        UpdatePIDParameters(cmdData.data.config.param, cmdData.data.config.value);
                        break;
                }
            }
        }
    }
}

// Обновляем PID параметров
void UpdatePIDParameters(SystemParameter param, float value) {
    // Получаем текущие значения PID
    static float roll_kp = 1.0f, roll_ki = 0.1f, roll_kd = 0.05f;
    static float pitch_kp = 1.0f, pitch_ki = 0.1f, pitch_kd = 0.05f;
    static float yaw_kp = 2.0f, yaw_ki = 0.15f, yaw_kd = 0.1f;
    static float alt_kp = 1.5f, alt_ki = 0.2f, alt_kd = 0.1f;
    
    // Обновляем соответствующий параметр
    switch (param) {
        case PARAM_PID_ROLL:
            roll_kp = value;
            break;
        case PARAM_PID_PITCH:
            pitch_kp = value;
            break;
        case PARAM_PID_YAW:
            yaw_kp = value;
            break;
        case PARAM_PID_ALT:
            alt_kp = value;
            break;
    }
    
    // Применяем все параметры
    Stabilization_ConfigurePID(
        roll_kp, roll_ki, roll_kd,    // Roll PID
        pitch_kp, pitch_ki, pitch_kd,  // Pitch PID
        yaw_kp, yaw_ki, yaw_kd,       // Yaw PID
        alt_kp, alt_ki, alt_kd        // Altitude PID
    );
}

// Обновление телеметрии
static void UpdateTelemetry(void) {
    TelemetryData telemetry;
    
    // Получение данных с датчиков
    Sensors_GetAllData(&sensorData);
    
    // Получение навигационных данных
    GPSPosition currentPos;
    Navigation_GetCurrentPosition(&currentPos);
    float altitude = Navigation_GetCurrentAltitude();
    float distanceToTarget = Navigation_GetDistanceToTarget();
    
    // Получение состояния питания и температур
    PowerState powerState;
    ThermalState thermalState;
    Diagnostics_GetPowerState(&powerState);
    Diagnostics_GetThermalState(&thermalState);
    
    // Заполнение данных телеметрии
    telemetry.batteryVoltage = powerState.batteryVoltage;
    telemetry.temperature = thermalState.mcuTemp;
    telemetry.position = currentPos;
    
    // Получаем текущую ориентацию
    Orientation currentOrientation;
    Sensors_GetOrientation(&currentOrientation);
    telemetry.orientation = currentOrientation;
    
    telemetry.altitude = altitude;
    telemetry.groundSpeed = distanceToTarget;
    telemetry.verticalSpeed = 0.0f;
    telemetry.flightMode = (uint8_t)currentState;
    telemetry.gpsStatus = (sensorData.isCalibrated ? 1 : 0);
    telemetry.batteryStatus = powerState.batteryLevel;
    telemetry.errorFlags = (uint8_t)Diagnostics_GetErrorCount();
    
    // Отправка телеметрии
    Communication_SendTelemetry(&telemetry);
}

// Проверка безопасности
void CheckSafety(void) {
    // Проверка напряжения батареи
    if (sensorData.imu.temperature < MIN_BATTERY_VOLTAGE) { // Временно используем temperature как voltage
        EnterEmergencyMode("Low battery");
        return;
    }
    
    // Проверка высоты
    if (sensorData.baro.altitude > MAX_ALTITUDE) {
        EnterEmergencyMode("Maximum altitude exceeded");
        return;
    }
    
    // Проверка углов наклона (используем данные акселерометра)
    if (fabsf(sensorData.imu.accel_x) > MAX_ANGLE ||
        fabsf(sensorData.imu.accel_y) > MAX_ANGLE) {
        EnterEmergencyMode("Maximum angle exceeded");
        return;
    }
    
    // Проверка связи
    if (!Communication_IsConnected()) {
        EnterEmergencyMode("Communication lost");
        return;
    }
}

// Вход в аварийный режим
void EnterEmergencyMode(const char* reason) {
    if (!isEmergencyMode) {
        isEmergencyMode = true;
        currentState = DRONE_STATE_EMERGENCY;
        
        // Логирование причины
        Diagnostics_Log(LOG_CRITICAL, DIAG_EVENT_SYSTEM_START, reason, 0);
        
        // Отправка сообщения об ошибке
        Communication_SendError(0x01, reason);
        
        // Аварийная остановка моторов и сброс навигации
        Actuators_EmergencyStop();
        // Сброс навигации и аварийной остановки
        Navigation_Stop();              // Останавливаем текущую навигацию
        Navigation_ClearWaypoints();    // Очищаем все точки маршрута
        
        // Включение аварийной индикации
        HandleLEDs();
    }
}

// Выход из аварийного режима
void ExitEmergencyMode(void) {
    if (isEmergencyMode) {
        isEmergencyMode = false;
        currentState = DRONE_STATE_IDLE;
        
        // Сброс лога ошибок
        Diagnostics_ClearLog();
        
        // Сброс навигации и аварийной остановки
        Navigation_Stop();              // Останавливаем текущую навигацию
        Navigation_ClearWaypoints();    // Очищаем все точки маршрута
        Actuators_ResetEmergencyStop();
        
        // Отправка статуса
        Communication_SendStatus(0x00);
        
        // Обновление LED индикации
        HandleLEDs();
    }
}
