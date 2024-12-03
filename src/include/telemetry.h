#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "navigation.h"
#include "communication.h"  // Для структуры TelemetryData

// Функции телеметрии
void Telemetry_Init(void);
void Telemetry_Update(void);
void Telemetry_Send(void);

#endif // TELEMETRY_H
