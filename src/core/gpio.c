#include "../include/gpio.h"

void GPIOPinTypeGPIOOutput(uint32_t port, uint8_t pins) {
    // Настраиваем пины на выход
    *((volatile uint32_t *)(port + 0x400)) = pins;  // GPIO_O_DIR
    // Разрешаем цифровой ввод-вывод
    *((volatile uint32_t *)(port + 0x51C)) = pins;  // GPIO_O_DEN
}

void SysCtlPeripheralEnable(uint32_t peripheral) {
    // Включаем тактирование периферии
    SYSCTL_RCGC2_R |= (peripheral & 0x000000FF);
}

bool SysCtlPeripheralReady(uint32_t peripheral) {
    // Проверяем готовность периферии
    return (SYSCTL_RCGC2_R & (peripheral & 0x000000FF)) != 0;
}

void SysCtlClockSet(uint32_t config) {
    // Настройка системной частоты
    // TODO: Реализовать если потребуется точная настройка частоты
    // Сейчас просто включаем PLL
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;
}
