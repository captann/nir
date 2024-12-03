#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>

// GPIO пины
#define GPIO_PIN_0              0x00000001  // GPIO pin 0
#define GPIO_PIN_1              0x00000002  // GPIO pin 1
#define GPIO_PIN_2              0x00000004  // GPIO pin 2
#define GPIO_PIN_3              0x00000008  // GPIO pin 3
#define GPIO_PIN_4              0x00000010  // GPIO pin 4
#define GPIO_PIN_5              0x00000020  // GPIO pin 5
#define GPIO_PIN_6              0x00000040  // GPIO pin 6
#define GPIO_PIN_7              0x00000080  // GPIO pin 7

// Базовые адреса GPIO портов
#define GPIO_PORTA_BASE        0x40004000
#define GPIO_PORTB_BASE        0x40005000
#define GPIO_PORTC_BASE        0x40006000
#define GPIO_PORTD_BASE        0x40007000
#define GPIO_PORTE_BASE        0x40024000
#define GPIO_PORTF_BASE        0x40025000

// System Control определения
#define SYSCTL_RCGC2_R         (*((volatile uint32_t *)0x400FE108))
#define SYSCTL_RCGC2_GPIOF     0x00000020  // Port F Clock Gating Control

// System Control bits
#define SYSCTL_SYSDIV_2_5      0x04000000
#define SYSCTL_USE_PLL         0x00000000
#define SYSCTL_OSC_MAIN        0x00000000
#define SYSCTL_XTAL_16MHZ      0x00000540

// Peripheral defines
#define SYSCTL_PERIPH_GPIOF    0xf0000805  // GPIO F peripheral
#define SYSCTL_PERIPH_GPIOE    0xf0000804  // GPIO E peripheral
#define SYSCTL_PERIPH_GPIOD    0xf0000803  // GPIO D peripheral
#define SYSCTL_PERIPH_GPIOC    0xf0000802  // GPIO C peripheral
#define SYSCTL_PERIPH_GPIOB    0xf0000801  // GPIO B peripheral
#define SYSCTL_PERIPH_GPIOA    0xf0000800  // GPIO A peripheral

// Функции для работы с GPIO
void GPIOPinTypeGPIOOutput(uint32_t port, uint8_t pins);
void SysCtlPeripheralEnable(uint32_t peripheral);
bool SysCtlPeripheralReady(uint32_t peripheral);
void SysCtlClockSet(uint32_t config);

#endif // GPIO_H
