#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "../include/hardware.h"
#include "../include/constants.h"
#include "../include/structures.h"

// Обновление LED индикации
static void update_led_indication(void) {
    uint32_t now = GetMillis();
    
    switch(current_state) {
        case DRONE_STATE_INIT:
            // Быстрое мигание
            if(now - last_led_toggle >= 100) {
                led_state = !led_state;
                last_led_toggle = now;
            }
            break;
            
        case DRONE_STATE_IDLE:
            // Попеременное мигание
            if(now - last_led_toggle >= 500) {
                led_state = !led_state;
                last_led_toggle = now;
            }
            break;
            
        case DRONE_STATE_ARMED:
            // Медленное мигание
            if(now - last_led_toggle >= 1000) {
                led_state = !led_state;
                last_led_toggle = now;
            }
            break;
            
        case DRONE_STATE_FLYING:
            // Постоянное горение
            led_state = true;
            break;
            
        case DRONE_STATE_EMERGENCY:
            // Очень быстрое мигание
            if(now - last_led_toggle >= 50) {
                led_state = !led_state;
                last_led_toggle = now;
            }
            break;
            
        default:
            led_state = false;
            break;
    }
    
    // Применение состояния LED
    GPIOPinWrite(LED_PORT, LED_BLUE, led_state ? LED_BLUE : 0);
}

// Проверка критических условий
static void check_safety_conditions(const SensorData* sensors) {
    if(fabsf(sensors->roll) > MAX_SAFE_ANGLE || 
       fabsf(sensors->pitch) > MAX_SAFE_ANGLE) {
        current_state = DRONE_STATE_EMERGENCY;
        Motors_EmergencyStop();
    }
    
    if(sensors->battery_voltage < LOW_VOLTAGE_THRESHOLD) {
        current_state = DRONE_STATE_EMERGENCY;
        Motors_EmergencyStop();
    }
}

int main(void) {
    // Настройка системного тактирования
    SystemClock_Config();
    
    // Инициализация периферии
    GPIO_Init();
    I2C_Init();
    Timer_Init();
    
    // Инициализация подсистем
    Sensors_Init();
    Motors_Init();
    
    // Переход в режим калибровки
    current_state = DRONE_STATE_IDLE;
    
    // Основной цикл
    uint32_t last_sensor_read = 0;
    uint32_t last_control_update = 0;
    static uint32_t last_led_toggle = 0;
    static bool led_state = false;

    while(1) {
        uint32_t now = GetMillis();
        
        // Обновление состояния LED
        update_led_indication();
        
        // Чтение данных с датчиков
        if(now - last_sensor_read >= SENSOR_READ_PERIOD) {
            Sensors_Read();
            last_sensor_read = now;
            
            // Проверка безопасности
            const SensorData* sensor_data = Sensors_GetData();
            check_safety_conditions(sensor_data);
        }
        
        // Обновление управления
        if(now - last_control_update >= CONTROL_LOOP_PERIOD && 
           current_state == DRONE_STATE_FLYING) {
            last_control_update = now;
            
            // TODO: Реализовать PID регулирование
            // Пример простого управления:
            Motors_SetPWM(1200, 1200, 1200, 1200);
        }
        
        // TODO: Добавить обработку команд через UART
    }
}
