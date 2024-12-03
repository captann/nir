#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"
#include "navigation.h"
#include "stabilization.h"

// Регистры GPIO Port A для UART
#define GPIO_PORTA_AFSEL_R    (*((volatile uint32_t *)0x40004420))  // Port A Alternate Function Select
#define GPIO_PORTA_DEN_R      (*((volatile uint32_t *)0x4000451C))  // Port A Digital Enable
#define GPIO_PORTA_AMSEL_R    (*((volatile uint32_t *)0x40004528))  // Port A Analog Mode Select
#define GPIO_PORTA_PCTL_R     (*((volatile uint32_t *)0x4000452C))  // Port A Port Control

// Константы для протокола связи
#define COMM_BUFFER_SIZE     256
#define COMM_BAUDRATE       115200
#define COMM_MAX_PACKET_SIZE 64

// Типы пакетов
typedef enum {
    PACKET_TYPE_HEARTBEAT    = 0x00,  // Проверка соединения
    PACKET_TYPE_TELEMETRY    = 0x01,  // Телеметрия (сенсоры, положение, статус)
    PACKET_TYPE_COMMAND      = 0x02,  // Команды управления
    PACKET_TYPE_CONFIG       = 0x03,  // Настройка параметров
    PACKET_TYPE_WAYPOINT     = 0x04,  // Данные маршрутных точек
    PACKET_TYPE_STATUS       = 0x05,  // Статус дрона
    PACKET_TYPE_ERROR        = 0xFF   // Сообщения об ошибках
} PacketType;

// Команды управления
typedef enum {
    CMD_ARM           = 0x01,  // Активация моторов
    CMD_DISARM        = 0x02,  // Деактивация моторов
    CMD_TAKEOFF       = 0x03,  // Взлет
    CMD_LAND          = 0x04,  // Посадка
    CMD_RTH           = 0x05,  // Возврат домой
    CMD_HOLD          = 0x06,  // Удержание позиции
    CMD_EMERGENCY_STOP = 0x07  // Аварийная остановка
} CommandType;

#pragma pack(push, 2)  // Выравнивание по 2 байта для оптимизации размера пакетов
// Структура пакета данных
typedef struct {
    uint8_t type;          // Тип пакета
    uint8_t length;        // Длина данных
    uint8_t data[COMM_MAX_PACKET_SIZE];  // Данные
    uint16_t checksum;     // Контрольная сумма
} CommPacket;

// Структура телеметрии
typedef struct {
    // Данные сенсоров
    float batteryVoltage;      // Напряжение батареи (В)
    float temperature;         // Температура MCU (°C)
    
    // Положение и ориентация
    GPSPosition position;      // Текущая GPS позиция
    Orientation orientation;   // Текущая ориентация
    
    // Статус полета
    float altitude;           // Текущая высота (м)
    float groundSpeed;        // Скорость относительно земли (м/с)
    float verticalSpeed;      // Вертикальная скорость (м/с)
    
    // Статус системы
    uint8_t flightMode;       // Режим полета
    uint8_t gpsStatus;        // Статус GPS
    uint8_t batteryStatus;    // Уровень заряда батареи (%)
    uint8_t errorFlags;       // Флаги ошибок
} TelemetryData;
#pragma pack(pop)

// Функции UART
bool UART_Init(uint32_t baudRate);
uint16_t UART_Receive(uint8_t* buffer, uint16_t maxLength);
bool UART_Send(const uint8_t* data, uint16_t length);

// Функции коммуникации
void Communication_Init(void);
void Communication_SendData(uint8_t* data, uint16_t length);
uint16_t Communication_Receive(uint8_t* buffer, uint16_t maxLength);  // Изменили возвращаемый тип и имя параметра
bool Communication_IsConnected(void);
void Communication_Process(void);

// Основные функции связи
bool Communication_Send(PacketType type, const void* data, uint8_t length);
uint16_t Communication_Receive(uint8_t* buffer, uint16_t maxLength);

// Отправка специфических данных
bool Communication_SendTelemetry(const TelemetryData* telemetry);
bool Communication_SendStatus(uint8_t status);
bool Communication_SendError(uint8_t errorCode, const char* message);

// Обработка команд
void Communication_HandleCommand(const CommPacket* packet);
void Communication_HandleConfig(const CommPacket* packet);

// Проверка состояния связи
uint32_t Communication_GetLastPacketTime(void);

// Настройка параметров связи
void Communication_SetBaudRate(uint32_t baudRate);
void Communication_SetTelemetryRate(uint16_t rateHz);

// Функции диагностики
uint32_t Communication_GetPacketsSent(void);
uint32_t Communication_GetPacketsReceived(void);
uint32_t Communication_GetErrorCount(void);

#endif // COMMUNICATION_H
