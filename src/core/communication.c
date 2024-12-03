#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <TM4C123GH6PM.h>  // Используем системный путь
#include "../include/communication.h"
#include "../include/diagnostics.h"  // Добавляем для использования общих определений регистров

// Регистры UART
#define UART_FR_RXFE    0x10    // Флаг пустого буфера приема
#define UART_FR_TXFF    0x20    // Флаг заполненного буфера передачи
#define UART_FR_BUSY    0x08    // Флаг занятости UART

// Регистры GPIO
#define GPIO_PA0_U0RX   0x01    // UART0 RX на PA0
#define GPIO_PA1_U0TX   0x02    // UART0 TX на PA1

// Параметры соединения
#define COMM_TIMEOUT_MS     1000    // Таймаут соединения (1 секунда)
#define SYSTICK_FREQ_HZ    1000    // Частота системного таймера (1 кГц)

// Определения регистров UART
#define UART0_DR_R              (*((volatile uint32_t *)0x4000C000))
#define UART0_FR_R              (*((volatile uint32_t *)0x4000C018))
#define UART0_IBRD_R           (*((volatile uint32_t *)0x4000C024))
#define UART0_FBRD_R           (*((volatile uint32_t *)0x4000C028))
#define UART0_LCRH_R           (*((volatile uint32_t *)0x4000C02C))
#define UART0_CTL_R            (*((volatile uint32_t *)0x4000C030))

// Определения регистров системного управления
#define SYSCTL_RCGCUART_R     (*((volatile uint32_t *)0x400FE104))
// SYSCTL_RCGCGPIO_R теперь используется из diagnostics.h
#define SYSCTL_PRGPIO_R       (*((volatile uint32_t *)0x400FEA08))

#pragma pack(push, 4)  // Выравнивание по 4 байта для оптимизации
// Состояние модуля связи
static struct {
    bool initialized;
    bool connected;
    uint32_t lastPacketTime;    // Время последнего принятого пакета
    uint32_t currentTime;       // Текущее время в миллисекундах
    uint32_t packetsSent;
    uint32_t packetsReceived;
    uint32_t errorCount;
    uint32_t baudRate;
    uint16_t telemetryRateHz;
    uint16_t padding;           // Явное выравнивание
    uint8_t rxBuffer[COMM_BUFFER_SIZE];
    uint8_t txBuffer[COMM_BUFFER_SIZE];
    uint16_t rxIndex;
} commState = {0};
#pragma pack(pop)

// Вспомогательные функции
static uint16_t CalculateChecksum(const uint8_t* data, uint16_t length);
static bool ValidatePacket(const CommPacket* packet);
static void ProcessReceivedPacket(const CommPacket* packet);
static void UpdateConnectionStatus(void);

// Прототипы дополнительных функций
static void SysTick_Init(void);
static uint32_t GetCurrentTime(void);

// Реализация функций UART
bool UART_Init(uint32_t baudRate) {
    // Включаем тактирование UART0 и GPIOA
    SYSCTL_RCGCUART_R |= (1U << 0);
    SYSCTL_RCGCGPIO_R |= (1U << 0);
    
    // Ждем готовности периферии
    while((SYSCTL_PRGPIO_R & (1U << 0)) == 0) {}
    
    // Настраиваем пины PA0 и PA1 для UART
    GPIO_PORTA_AFSEL_R |= (GPIO_PA0_U0RX | GPIO_PA1_U0TX);
    GPIO_PORTA_DEN_R |= (GPIO_PA0_U0RX | GPIO_PA1_U0TX);
    GPIO_PORTA_AMSEL_R &= ~(GPIO_PA0_U0RX | GPIO_PA1_U0TX);
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) | 0x00000011;
    
    // Отключаем UART для настройки
    UART0_CTL_R &= ~0x01;
    
    // Настраиваем скорость передачи
    uint32_t systemClock = 16000000; // 16 MHz
    uint32_t brdi = systemClock / (16 * baudRate);
    uint32_t brdf = (uint32_t)(((float)systemClock / (16.0f * (float)baudRate) - brdi) * 64.0f + 0.5f);
    
    UART0_IBRD_R = brdi;
    UART0_FBRD_R = brdf;
    
    // 8 бит, 1 стоп-бит, без четности
    UART0_LCRH_R = 0x60;
    
    // Включаем UART
    UART0_CTL_R |= 0x01;
    
    return true;
}

uint16_t UART_Receive(uint8_t* buffer, uint16_t maxLength) {
    uint16_t count = 0;
    
    if (buffer == NULL || maxLength == 0) {
        return 0;
    }
    
    while (count < maxLength) {
        if (UART0_FR_R & UART_FR_RXFE) {
            break;  // Буфер приема пуст
        }
        buffer[count] = (uint8_t)(UART0_DR_R & 0xFF);
        count++;
    }
    
    return count;
}

bool UART_Send(const uint8_t* data, uint16_t length) {
    uint16_t i;
    
    if (data == NULL || length == 0) {
        return false;
    }
    
    for (i = 0; i < length; i++) {
        while (UART0_FR_R & UART_FR_TXFF) {}  // Ждем освобождения буфера передачи
        UART0_DR_R = (uint32_t)data[i];
    }
    
    return true;
}

bool Communication_Init(void) {
    if (commState.initialized) {
        return true;
    }
    
    // Инициализация состояния
    memset(&commState, 0, sizeof(commState));
    commState.baudRate = COMM_BAUDRATE;
    commState.telemetryRateHz = 10; // По умолчанию 10 Гц
    
    // Инициализация системного таймера
    SysTick_Init();
    
    // Инициализация UART
    if (!UART_Init(commState.baudRate)) {
        return false;
    }
    
    commState.initialized = true;
    return true;
}

void Communication_Process(void) {
    if (!commState.initialized) {
        return;
    }
    
    // Прием данных
    uint16_t bytesReceived = UART_Receive(
        &commState.rxBuffer[commState.rxIndex],
        (uint16_t)(COMM_BUFFER_SIZE - commState.rxIndex)
    );
    
    if (bytesReceived > 0) {
        commState.rxIndex = (uint16_t)(commState.rxIndex + bytesReceived);
        
        // Поиск и обработка пакетов в буфере
        uint16_t processedIndex = 0;
        while (processedIndex + sizeof(CommPacket) <= commState.rxIndex) {
            CommPacket* packet = (CommPacket*)(commState.rxBuffer + processedIndex);
            
            // Проверяем, получен ли полный пакет
            uint16_t packetSize = sizeof(CommPacket) - COMM_MAX_PACKET_SIZE + packet->length;
            if (processedIndex + packetSize > commState.rxIndex) {
                break;
            }
            
            // Проверяем и обрабатываем пакет
            if (ValidatePacket(packet)) {
                ProcessReceivedPacket(packet);
                commState.packetsReceived++;
                commState.lastPacketTime = GetCurrentTime();
            } else {
                commState.errorCount++;
            }
            
            processedIndex += packetSize;
        }
        
        // Сдвигаем необработанные данные в начало буфера
        if (processedIndex > 0) {
            if (processedIndex < commState.rxIndex) {
                memmove(commState.rxBuffer,
                       commState.rxBuffer + processedIndex,
                       commState.rxIndex - processedIndex);
            }
            commState.rxIndex -= processedIndex;
        }
    }
    
    UpdateConnectionStatus();
}

bool Communication_Send(PacketType type, const void* data, uint8_t length) {
    if (!commState.initialized || length > COMM_MAX_PACKET_SIZE) {
        return false;
    }
    
    // Формируем пакет
    CommPacket packet;
    packet.type = type;
    packet.length = length;
    
    if (data != NULL && length > 0) {
        memcpy(packet.data, data, (size_t)length);
    }
    
    // Вычисляем контрольную сумму
    packet.checksum = CalculateChecksum((const uint8_t*)&packet, (uint16_t)(sizeof(CommPacket) - sizeof(uint16_t)));
    
    // Отправляем пакет
    if (UART_Send((const uint8_t*)&packet, (uint16_t)sizeof(CommPacket))) {
        commState.packetsSent++;
        return true;
    }
    
    commState.errorCount++;
    return false;
}

bool Communication_SendTelemetry(const TelemetryData* telemetry) {
    if (telemetry == NULL) {
        return false;
    }
    return Communication_Send(PACKET_TYPE_TELEMETRY, telemetry, sizeof(TelemetryData));
}

bool Communication_SendStatus(uint8_t status) {
    return Communication_Send(PACKET_TYPE_STATUS, &status, sizeof(status));
}

bool Communication_SendError(uint8_t errorCode, const char* message) {
    struct {
        uint8_t code;
        char message[COMM_MAX_PACKET_SIZE - 1];
    } errorData;
    
    errorData.code = errorCode;
    if (message != NULL) {
        strncpy(errorData.message, message, sizeof(errorData.message) - 1);
        errorData.message[sizeof(errorData.message) - 1] = '\0';
    } else {
        errorData.message[0] = '\0';
    }
    
    return Communication_Send(PACKET_TYPE_ERROR, &errorData,
                            1 + strlen(errorData.message) + 1);
}

void Communication_HandleCommand(const CommPacket* packet) {
    if (packet == NULL || packet->type != PACKET_TYPE_COMMAND) {
        return;
    }
    
    CommandType cmd = packet->data[0];
    switch (cmd) {
        case CMD_ARM:
            // Включение моторов
            break;
            
        case CMD_DISARM:
            // Выключение моторов
            break;
            
        case CMD_TAKEOFF:
            // Взлет
            break;
            
        case CMD_LAND:
            // Посадка
            break;
            
        case CMD_RTH:
            // Возврат домой
            break;
            
        case CMD_HOLD:
            // Удержание позиции
            break;
            
        case CMD_EMERGENCY_STOP:
            // Аварийная остановка
            break;
            
        default:
            Communication_SendError(0x01, "Unknown command");
            break;
    }
}

void Communication_HandleConfig(const CommPacket* packet) {
    if (packet == NULL || packet->type != PACKET_TYPE_CONFIG) {
        return;
    }
    
    // Обработка конфигурационных данных
    // TODO: Реализовать обработку различных параметров
}

bool Communication_IsConnected(void) {
    return commState.connected;
}

uint32_t Communication_GetLastPacketTime(void) {
    return commState.lastPacketTime;
}

void Communication_SetBaudRate(uint32_t baudRate) {
    if (baudRate != commState.baudRate) {
        commState.baudRate = baudRate;
        if (commState.initialized) {
            UART_Init(baudRate);
        }
    }
}

void Communication_SetTelemetryRate(uint16_t rateHz) {
    commState.telemetryRateHz = rateHz;
}

uint32_t Communication_GetPacketsSent(void) {
    return commState.packetsSent;
}

uint32_t Communication_GetPacketsReceived(void) {
    return commState.packetsReceived;
}

uint32_t Communication_GetErrorCount(void) {
    return commState.errorCount;
}

// Обработчик прерывания системного таймера
__attribute__((weak)) void SysTick_Handler(void) {
    // Обновляем счетчик времени
    commState.currentTime++;
    
    // Проверяем таймаут соединения
    if (commState.currentTime - commState.lastPacketTime > COMM_TIMEOUT_MS) {
        commState.connected = false;
    }
}

static void SysTick_Init(void) {
    // Отключаем системный таймер
    NVIC_ST_CTRL_R = 0;
    
    // Устанавливаем период обновления (например, 1 мс)
    NVIC_ST_RELOAD_R = SystemCoreClock / 1000 - 1;
    
    // Очищаем текущее значение таймера
    NVIC_ST_CURRENT_R = 0;
    
    // Включаем системный таймер с прерываниями
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE | 
                     NVIC_ST_CTRL_INTEN | 
                     NVIC_ST_CTRL_CLKSRC;
}

static uint32_t GetCurrentTime(void) {
    return commState.currentTime;
}

static void UpdateConnectionStatus(void) {
    uint32_t currentTime = GetCurrentTime();
    
    // Проверяем таймаут соединения
    if (currentTime - commState.lastPacketTime > COMM_TIMEOUT_MS) {
        if (commState.connected) {
            commState.connected = false;
            // Отправляем сообщение об ошибке при потере связи
            Communication_SendError(0x03, "Connection lost");
        }
    } else {
        if (!commState.connected) {
            commState.connected = true;
            // Отправляем статус восстановления связи
            Communication_SendStatus(0x01); // 0x01 - связь восстановлена
        }
    }
}

// Вспомогательные функции
static uint16_t CalculateChecksum(const uint8_t* data, uint16_t length) {
    uint16_t checksum = 0;
    for(uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

static bool ValidatePacket(const CommPacket* packet) {
    if (packet == NULL) {
        return false;
    }
    
    // Проверяем длину пакета
    if (packet->length > COMM_MAX_PACKET_SIZE) {
        return false;
    }
    
    // Проверяем контрольную сумму
    uint16_t calculatedChecksum = CalculateChecksum(
        (uint8_t*)packet,
        sizeof(CommPacket) - sizeof(packet->checksum)
    );
    
    return calculatedChecksum == packet->checksum;
}

static void ProcessReceivedPacket(const CommPacket* packet) {
    switch (packet->type) {
        case PACKET_TYPE_HEARTBEAT:
            // Отвечаем на heartbeat
            Communication_Send(PACKET_TYPE_HEARTBEAT, NULL, 0);
            break;
            
        case PACKET_TYPE_COMMAND:
            Communication_HandleCommand(packet);
            break;
            
        case PACKET_TYPE_CONFIG:
            Communication_HandleConfig(packet);
            break;
            
        case PACKET_TYPE_WAYPOINT:
            // Обработка маршрутных точек
            break;
            
        default:
            Communication_SendError(0x02, "Unknown packet type");
            break;
    }
}

// Заглушки для коммуникации
void Communication_SendData(uint8_t* data, uint16_t length) {
    // Пустая заглушка
}

void Communication_Receive(uint8_t* buffer, uint16_t* length) {
    if(length) *length = 0;
}
