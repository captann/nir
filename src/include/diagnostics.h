#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>  // Для isnan() и математических функций

// Log Levels
typedef enum {
    LOG_DEBUG = 0,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_CRITICAL,
    LOG_FATAL
} LogLevel;

// Diagnostic Events - consolidated version
typedef enum {
    DIAG_EVENT_SYSTEM_START = 0,
    DIAG_EVENT_SYSTEM_STOP,
    DIAG_EVENT_HIGH_TEMP,
    DIAG_EVENT_LOW_BATTERY,
    DIAG_EVENT_BATTERY_CRITICAL,
    DIAG_EVENT_BATTERY_LOW,     // Added missing event
    DIAG_EVENT_NAV_ERROR,
    DIAG_EVENT_SENSOR_ERROR,
    DIAG_EVENT_MOTOR_ERROR,
    DIAG_EVENT_COMM_ERROR,
    DIAG_EVENT_STABILIZATION_ERROR,
    DIAG_EVENT_WATCHDOG_RESET,
    DIAG_EVENT_SELF_TEST,
    DIAG_EVENT_SYSTEM_ERROR,
    DIAG_EVENT_CALIBRATION,
    DIAG_EVENT_ERROR,
    DIAG_EVENT_WARNING
} DiagnosticEvent;

// Test Results
typedef enum {
    TEST_RESULT_PASS = 0,
    TEST_RESULT_WARNING,
    TEST_RESULT_FAIL,
    TEST_RESULT_NOT_RUN
} TestResult;

// ADC Error Codes - consolidated
typedef enum {
    ADC_ERROR_NONE = 0,
    ADC_ERROR_NOT_INITIALIZED,
    ADC_ERROR_TIMEOUT,
    ADC_ERROR_OUT_OF_RANGE,
    ADC_ERROR_CONVERSION_FAILED
} ADCError;

// System Tick Error Codes - consolidated
typedef enum {
    SYSTICK_ERROR_NONE = 0,
    SYSTICK_ERROR_INVALID_VALUE,
    SYSTICK_ERROR_INVALID_START,
    SYSTICK_ERROR_DURING_DELAY,
    SYSTICK_ERROR_INVALID_CURRENT
} SystemTickError;

// Power Modes - consolidated
typedef enum {
    POWER_MODE_NORMAL = 0,
    POWER_MODE_SLEEP,
    POWER_MODE_DEEP_SLEEP
} PowerMode;

// System States - consolidated
typedef enum {
    SYSTEM_STATE_OK = 0,
    SYSTEM_STATE_WARNING,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_CRITICAL
} SystemState;

// Thermocouple Types
typedef enum {
    THERMOCOUPLE_TYPE_K = 0,    // Type K thermocouple
    THERMOCOUPLE_TYPE_J,        // Type J thermocouple
    THERMOCOUPLE_TYPE_T,        // Type T thermocouple
    THERMOCOUPLE_TYPE_E,        // Type E thermocouple
    THERMOCOUPLE_TYPE_N,        // Type N thermocouple
    THERMOCOUPLE_TYPE_R,        // Type R thermocouple
    THERMOCOUPLE_TYPE_S,        // Type S thermocouple
    THERMOCOUPLE_TYPE_B,        // Type B thermocouple
    THERMOCOUPLE_TYPE_COUNT     // Number of thermocouple types
} ThermocoupleType;

// Temperature sensor types
#define TEMP_SENSOR_TYPE_LINEAR    0
#define TEMP_SENSOR_TYPE_NTC       1
#define TEMP_SENSOR_TYPE_PTC       2
#define TEMP_SENSOR_TYPE_THERMO    3

// Temperature thresholds
#define TEMP_MCU_WARNING      85.0f
#define TEMP_MCU_CRITICAL     95.0f
#define TEMP_MOTORS_WARNING   70.0f
#define TEMP_MOTORS_CRITICAL  80.0f
#define TEMP_BOARD_WARNING    60.0f
#define TEMP_BOARD_CRITICAL   70.0f
#define MAX_SAFE_TEMPERATURE   125.0f

// Test status constants
#define TEST_PASS    0
#define TEST_WARNING 1
#define TEST_FAIL    2
#define TEST_NOT_RUN 3

// Structure Definitions
typedef struct {
    uint8_t status;     // TEST_PASS, TEST_WARNING, TEST_FAIL, or TEST_NOT_RUN
    uint32_t code;      // Error or status code
    const char* message;// Status message
    uint32_t data;      // Additional data
} TestStatus;

typedef struct {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float magX;
    float magY;
    float magZ;
    float temp;
} IMUData;

typedef struct {
    uint32_t timestamp;      // Время события
    LogLevel level;          // Уровень важности
    DiagnosticEvent event;   // Тип события
    char message[128];       // Текстовое сообщение
    uint32_t code;          // Код события/ошибки
    uint32_t data;          // Дополнительные данные
} LogEntry;

typedef struct {
    float mcuTemp;           // Temperature of the MCU
    float ambientTemp;       // Ambient temperature
    float motorTemps[4];     // Temperature of each motor
    float boardTemp;         // Temperature of the main board
    bool isCritical;         // True if any temperature is critical
    bool isWarning;          // True if any temperature is in warning range
} ThermalState;

typedef struct {
    float batteryVoltage;    // Напряжение батареи
    float batteryCurrent;    // Ток батареи
    float batteryTemp;       // Температура батареи
    float batteryLevel;      // Уровень заряда батареи (0-100%)
    uint32_t uptime;         // Время работы системы
    bool isCharging;         // Флаг зарядки
    bool isLowBattery;       // Флаг низкого заряда
    bool isCriticalBattery;  // Флаг критически низкого заряда
} PowerState;

typedef struct {
    bool powerSystemOperational;
    bool sensorsOperational;
    bool motorsOperational;
    bool communicationOperational;
    bool safetySystemOperational;
} SystemStatus;

// Единое определение структуры SelfTestResult
typedef struct {
    // Общая информация о тесте
    char message[64];        // Общее сообщение о результате
    TestResult result;
    uint32_t errorCode;
    
    // Детальная информация
    uint32_t totalErrors;
    uint32_t totalWarnings;
    bool isTestComplete;
    
    // Результаты отдельных тестов
    TestStatus powerTest;    // Тест системы питания
    TestStatus sensorTest;   // Тест датчиков
    TestStatus motorTest;    // Тест моторов
    TestStatus batteryTest;  // Тест батареи
    TestStatus commTest;     // Тест коммуникации
    TestStatus safetyTest;   // Тест системы безопасности
    
    // Статус системы
    SystemStatus systemStatus;
} SelfTestResult;

typedef struct {
    uint32_t value;
    ADCError error;
} ADCResult;

typedef struct {
    uint32_t ticks;
    SystemTickError error;
} SystemTickResult;

typedef struct {
    uint32_t elapsedTicks;
    SystemTickError error;
} ElapsedTicksResult;

typedef struct {
    bool complete;
    ADCError error;
} ADCConversionStatus;

typedef struct {
    // Параметры для термопар
    struct {
        ThermocoupleType thermocopleType;  // Тип термопары
        float referenceVoltage;            // Опорное напряжение
    } thermocoupleParams;
    
    // Параметры для PT100
    struct {
        float R0;             // Сопротивление при 0°C
        float alpha;          // Температурный коэффициент
    } pt100Params;
} TemperatureSensorConfig;

// ADC Register definitions
#define SYSCTL_RCGCGPIO_R    (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCADC_R     (*((volatile uint32_t *)0x400FE638))
#define SYSCTL_PRGPIO_R      (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_PRADC_R       (*((volatile uint32_t *)0x400FEA38))
#define SYSCTL_RCGC2_R       (*((volatile uint32_t *)0x400FE108))

#define GPIO_PORTD_DEN_R     (*((volatile uint32_t *)0x4005751C))
#define GPIO_PORTD_AMSEL_R   (*((volatile uint32_t *)0x40057528))

#define ADC1_ACTSS_R         (*((volatile uint32_t *)0x40039000))
#define ADC1_EMUX_R          (*((volatile uint32_t *)0x40039014))
#define ADC1_SSMUX0_R        (*((volatile uint32_t *)0x40039040))
#define ADC1_SSCTL0_R        (*((volatile uint32_t *)0x40039044))
#define ADC1_TSSEL_R         (*((volatile uint32_t *)0x4003901C))
#define ADC1_SAC_R           (*((volatile uint32_t *)0x40039030))
#define ADC1_IM_R            (*((volatile uint32_t *)0x40039008))
#define ADC1_SSFIF00_R       (*((volatile uint32_t *)0x40039048))

#define ADC0_SAC_R           (*((volatile uint32_t *)0x40038030))
#define ADC0_IM_R            (*((volatile uint32_t *)0x40038008))
#define ADC0_SSFIFO1_R       (*((volatile uint32_t *)0x40038048))

// ADC Bit definitions
#define ADC_ACTSS_ASEN0      0x00000001
#define ADC_EMUX_EM0_M       0x0000000F
#define ADC_SSCTL0_IE3       0x00000800
#define ADC_SSCTL0_END3      0x00000400
#define ADC_IM_MASK0         0x00000001
#define INT_ADC1SS0          51

// Определения битов ADC
#define ADC_PSSI_SS3          0x00000008
#define ADC_RIS_INR3          0x00000008
#define ADC_SSCTL0_IE2        0x00000400
#define ADC_SSCTL0_END2       0x00000800
#define ADC_SAC_AVG_64X       0x00000006
#define ADC_ACTSS_ASEN1       0x00000002  // SS1 Enable
#define ADC_EMUX_EM1_M        0x000000F0  // SS1 Trigger Select mask
#define ADC_IM_MASK1          0x00000002  // SS1 Interrupt Mask
#define ADC_RIS_INR1          0x00000002  // SS1 Interrupt Status
#define ADC_PSSI_SS1          0x00000002  // SS1 Initiate
#define ADC_SSCTL1_IE2        0x00000400  // Sample 2 Interrupt Enable
#define ADC_SSCTL1_END2       0x00000800  // Sample 2 is End of Sequence

// ADC Interrupt numbers
#define INT_ADC0SS1          15  // ADC0 Sequencer 1 interrupt

// Определения для Watchdog Timer
#define WATCHDOG0_LOAD_R     (*((volatile uint32_t *)0x40000000))  // Регистр загрузки WDT
#define WATCHDOG0_CTL_R      (*((volatile uint32_t *)0x40000008))  // Регистр управления WDT
#define WATCHDOG0_ICR_R      (*((volatile uint32_t *)0x4000000C))  // Регистр очистки прерываний WDT
#define WATCHDOG0_LOCK_R     (*((volatile uint32_t *)0x40000C00))  // Регистр блокировки WDT
#define WATCHDOG0_ICR_M      (*((volatile uint32_t *)0x40000010))  // Регистр очистки прерываний WDT

// Биты управления Watchdog Timer
#define WDT_CTL_INTEN        0x00000001  // Разрешение прерываний
#define WDT_CTL_RESEN        0x00000002  // Разрешение сброса системы

// Регистры NVIC
#define NVIC_SYS_CTRL_R      (*((volatile uint32_t *)0xE000ED10))
#define NVIC_SYS_CTRL_SLEEPDEEP 0x00000004

// Error Code type definition
typedef uint32_t ErrorCode;

// NVIC Register definitions
#define NVIC_EN0_R            (*((volatile uint32_t *)0xE000E100))
#define NVIC_PRI0_R          (*((volatile uint32_t *)0xE000E400))
#define NVIC_APINT_R         (*((volatile uint32_t *)0xE000ED0C))
#define NVIC_APINT_VECTKEY   0x05FA0000
#define NVIC_APINT_SYSRESETREQ 0x00000004

// Watchdog Timer Register definitions
#define SYSCTL_RCGCWD_R      (*((volatile uint32_t *)0x400FE600))
#define SYSCTL_PRWD_R        (*((volatile uint32_t *)0x400FEA00))

// Watchdog Timer bit definitions
#define SYSCTL_RCGCWD_R0     0x00000001
#define SYSCTL_PRWD_R0       0x00000001

// Temperature sensor calibration constants
#define MCU_TEMP_SLOPE       1.0f    // Slope for MCU temperature conversion
#define MCU_TEMP_OFFSET      0.0f    // Offset for MCU temperature conversion
#define TEMP_SENSOR_SLOPE    10.0f   // mV/°C for external temperature sensors

// ADC Channel definitions
#define TEMP_MCU_CHANNEL     0       // Internal temperature sensor channel

// NVIC Register definitions
#define NVIC_EN1_R           (*((volatile uint32_t *)0xE000E104))

// ADC FIFO registers
#define ADC1_SSFIFO0_R       (*((volatile uint32_t *)0x40039044))

// Регистры системного таймера
#define NVIC_ST_CTRL_R       (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R     (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R    (*((volatile uint32_t *)0xE000E018))

// Биты управления системным таймером
#define NVIC_ST_CTRL_ENABLE  0x00000001  // Включение таймера
#define NVIC_ST_CTRL_INTEN   0x00000002  // Включение прерываний
#define NVIC_ST_CTRL_CLKSRC  0x00000004  // Выбор источника тактирования (системная частота)

// Системная тактовая частота
extern uint32_t SystemCoreClock;  // Глобальная переменная тактовой частоты микроконтроллера

// Функции инициализации
bool Diagnostics_Init(void);
bool Diagnostics_PerformSelfTest(SelfTestResult* result);
void Diagnostics_InitSelfTest(void);
bool Diagnostics_PerformQuickTest(void);
void UpdateMotorTemperatures(void);
const char* Diagnostics_GetEventString(DiagnosticEvent event);
void Diagnostics_Log(uint8_t level, DiagnosticEvent event, const char* message, uint32_t data);

// Temperature monitoring functions
float GetColdJunctionTemperature(void);
float ConvertThermocoupleVoltageToTemp(float thermocoupleVoltage, ThermocoupleType type, float coldJunctionTemp);

// Функции работы с логом
void Diagnostics_Log(LogLevel level, DiagnosticEvent event, const char* message, uint32_t data);
void Diagnostics_ClearLog(void);
uint32_t Diagnostics_GetLogCount(void);
bool Diagnostics_GetLogEntry(uint32_t index, LogEntry* entry);

// Функции работы с IMU
bool Diagnostics_GetIMUData(IMUData* data);
bool Diagnostics_CalibrateIMU(void);
bool Diagnostics_IsIMUCalibrated(void);

// Функции мониторинга питания
void Diagnostics_UpdatePowerState(void);
void Diagnostics_GetPowerState(PowerState* state); // Обновленный прототип функции
float Diagnostics_GetBatteryVoltage(void);
uint8_t Diagnostics_GetBatteryLevel(void);

// Функции мониторинга температур
void Diagnostics_UpdateThermalState(void);
void Diagnostics_GetThermalState(ThermalState* state);
bool Diagnostics_IsOverheated(void);

// Функции сторожевого таймера
void Diagnostics_InitWatchdog(uint32_t timeout_ms);
void Diagnostics_FeedWatchdog(void);
bool Diagnostics_IsWatchdogReset(void);

// Функции производительности
void Diagnostics_StartPerformanceCounter(void);
uint32_t Diagnostics_StopPerformanceCounter(void);
void Diagnostics_ResetPerformanceStats(void);

// Функции статистики
uint32_t Diagnostics_GetErrorCount(void);
uint32_t Diagnostics_GetUptimeMs(void);

// Функции отладки
void Diagnostics_PrintSystemStatus(void);

// Обновленные прототипы функций с новыми типами
ADCResult Diagnostics_GetADCValue(void);
float Diagnostics_ConvertADCToTemp(ADCResult adcResult); // Обновленный прототип функции преобразования температур
ADCError Diagnostics_StartADCConversion(void);
ADCConversionStatus Diagnostics_IsADCConversionComplete(void);

SystemTickResult Diagnostics_GetSystemTicks(void);
SystemTickError Diagnostics_DelayTicks(uint32_t ticks);
ElapsedTicksResult Diagnostics_GetElapsedTicks(uint32_t startTicks);

// Функции получения температур с различных датчиков
uint32_t GetMCUTemperature(void);
uint32_t GetBatteryTemperature(void);
uint32_t GetBoardTemperature(void);
uint32_t GetEnvironmentTemperature(void);

// Функции работы с термопарами
float GetColdJunctionTemperature(void);
float ConvertThermocoupleVoltageToTemp(float thermocoupleVoltage, ThermocoupleType type, float coldJunctionTemp);

// Функции для работы с моторами
float GetMotorCurrent(int motorIndex);
uint16_t GetMotorRPM(void);        // Убрали параметр
float GetMotorTemperature(int motorIndex);
float GetMotorVibration(void);     // Убрали параметр

// ADC каналы для измерения тока моторов
#define ADC_CHANNEL_MOTOR_CURRENT_BASE  ADC_AIN5   // Базовый канал для измерения тока моторов

// Функции для работы с резервным каналом связи
bool HasBackupComm(void);      // Проверяет наличие резервного канала связи
bool TestBackupComm(void);     // Тестирует работоспособность резервного канала

// Коды ошибок резервного канала связи
#define BACKUP_COMM_ERR_NOT_RESPONDING  0x0401
#define BACKUP_COMM_ERR_CRC            0x0402
#define BACKUP_COMM_ERR_TIMEOUT        0x0403

// Функции для тестирования watchdog
bool TestWatchdogReset(void);  // Тестирует срабатывание watchdog reset

// Коды ошибок watchdog
#define WATCHDOG_ERR_NO_RESET     0x0501  // Watchdog не вызвал сброс
#define WATCHDOG_ERR_EARLY_RESET  0x0502  // Сброс произошел раньше времени
#define WATCHDOG_ERR_LATE_RESET   0x0503  // Сброс произошел позже времени

// Максимальное количество моторов
#define MAX_MOTORS 4

// Максимальное количество записей в логе
#define DIAG_MAX_LOG_ENTRIES 100

// Регистры SYSCTL
#define SYSCTL_RCGCGPIO_R3    0x00000008
#define SYSCTL_RCGCGPIO_R4    0x00000010
#define SYSCTL_RCGCADC_R0     0x00000001
#define SYSCTL_RCGCADC_R1     0x00000002
#define SYSCTL_PRGPIO_R3      0x00000008
#define SYSCTL_PRGPIO_R4      0x00000010
#define SYSCTL_PRADC_R0       0x00000001
#define SYSCTL_PRADC_R1       0x00000002

// Регистры ADC
#define ADC1_PSSI_R           (*((volatile uint32_t *)0x40039028))
#define ADC1_RIS_R            (*((volatile uint32_t *)0x40039004))
#define ADC_ISC_IN0           0x00000001

// Регистры GPIO
#define GPIO_PORTA_IM_R       (*((volatile uint32_t *)0x40004010))
#define GPIO_PORTA_IEV_R      (*((volatile uint32_t *)0x4000440C))
#define GPIO_PORTA_AFSEL_R    (*((volatile uint32_t *)0x40004420))  // Port A Alternate Function Select
#define GPIO_PORTE_AFSEL_R    (*((volatile uint32_t *)0x40024420))
#define GPIO_PORTE_DEN_R      (*((volatile uint32_t *)0x4002451C))
#define GPIO_PORTE_AMSEL_R    (*((volatile uint32_t *)0x40024528))
#define GPIO_PORTD_AFSEL_R    (*((volatile uint32_t *)0x40007420))

// Регистры Watchdog
#define WATCHDOG0_RIS_R       (*((volatile uint32_t *)0x40000004))
#define WATCHDOG0_ICR_R       (*((volatile uint32_t *)0x40000008))
#define WATCHDOG0_ICR_M       (*((volatile uint32_t *)0x40000010))
#define WDT_RIS_WDTRIS        0x00000001
#define WDT_ICR_M             0xFFFFFFFF  // Watchdog Timer Interrupt Clear Mask

// Векторы прерываний
#define INT_UART0             21
#define INT_WTIMER0A          94
#define INT_WATCHDOG          18

// Function declarations
float GetMotorTemperature(int motorIndex);
uint32_t GetBatteryTemperature(void);
uint32_t GetBoardTemperature(void);
uint32_t GetEnvironmentTemperature(void);

// Communication Functions
bool SendBackupCommPacket(const uint8_t* data, uint8_t length);
bool ReceiveBackupCommPacket(uint8_t* buffer, uint8_t maxLength);

// Watchdog Handler Declaration
void WDT0_Handler(void);

// Функции получения температур с различных датчиков
uint32_t GetMCUTemperature(void);
uint32_t GetBatteryTemperature(void);
uint32_t GetBoardTemperature(void);
uint32_t GetEnvironmentTemperature(void);

// Функции работы с термопарами
float GetColdJunctionTemperature(void);
float ConvertThermocoupleVoltageToTemp(float thermocoupleVoltage, ThermocoupleType type, float coldJunctionTemp);

// Константы АЦП и измерений
#define VREF                    3.3f    // Опорное напряжение (В)
#define ADC_MAX_VALUE          4095     // Максимальное значение АЦП (12 бит)
#define VOLTAGE_DIVIDER        0.5f     // Коэффициент делителя напряжения
#define CURRENT_SENSE_MV_A     100.0f   // мВ/А для датчика тока
#define TEMP_SENSOR_MV_C       10.0f    // мВ/°C для температурного датчика

// Пороговые значения батареи
#define BATTERY_MAX_VOLTAGE    12.6f    // Максимальное напряжение батареи
#define BATTERY_MIN_VOLTAGE    9.0f     // Минимальное напряжение батареи
#define BATTERY_LOW_VOLTAGE    10.0f    // Напряжение для предупреждения
#define BATTERY_CRITICAL_VOLTAGE 9.5f   // Критическое напряжение
#define BATTERY_MAX_TEMP       60.0f    // Максимальная температура

// Function declarations
const char* GetLogLevelString(LogLevel level);
const char* GetEventString(DiagnosticEvent event);
void Diagnostics_GetPowerState(PowerState* state);

// Test functions
TestStatus TestPowerSystem(void);
TestStatus TestSensors(void);
TestStatus TestMotors(void);
TestStatus TestCommunication(void);
TestStatus TestSafetySystems(void);

// Quick diagnostic test function
bool Diagnostics_PerformQuickTest(void);

// Функции для работы с моторами
float GetMotorCurrent(int motorIndex);
uint16_t GetMotorRPM(void);        // Убрали параметр
float GetMotorTemperature(int motorIndex);
float GetMotorVibration(void);     // Убрали параметр

// Тестовые функции диагностики
bool TestBarometer(void);
bool TestESC(void);
bool TestGPS(void);
bool TestEEPROM(void);
bool TestFlashMemory(void);
bool TestKillSwitch(void);
bool TestMagnetometer(void);
bool TestMotorResponse(void);
bool TestParachuteSystem(void);

// Функции проверки наличия компонентов
bool HasGPS(void);
bool HasParachute(void);

// Функции получения данных
float GetBaroAltitude(void);
uint16_t GetMotorRPM(void);        // Убрали параметр
float GetMotorVibration(void);     // Убрали параметр
uint8_t GetRCLinkQuality(void);
float GetSensorSupplyVoltage(void);

// Функции коммуникации и тестирования
bool IsRCTestResponseReceived(void);
bool IsTelemetryAckReceived(void);
void SendRCTestPacket(void);
void SendTelemetryTestPacket(void);
void SetMotorSpeed(uint8_t motor, uint16_t speed);
const char* Diagnostics_GetLastLogEntry(void);

#endif // DIAGNOSTICS_H
