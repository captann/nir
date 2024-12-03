#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>

// Структура для хранения GPS координат
typedef struct {
    float latitude;   // Широта в градусах
    float longitude;  // Долгота в градусах
    float altitude;   // Высота в метрах
    bool valid;       // Флаг валидности данных GPS
} GPSPosition;

// Структура для точки маршрута
typedef struct {
    GPSPosition position;
    float speed;           // Желаемая скорость в м/с
    float hover_time;      // Время зависания в секундах
    bool reached;          // Флаг достижения точки
} Waypoint;

// Структура для хранения текущего состояния навигации
typedef struct {
    GPSPosition current_position;
    float heading;              // Текущий курс в градусах (0-360)
    float ground_speed;         // Скорость относительно земли в м/с
    float vertical_speed;       // Вертикальная скорость в м/с
    float distance_to_target;   // Расстояние до целевой точки в метрах
    float bearing_to_target;    // Направление на целевую точку в градусах
    bool navigation_active;     // Флаг активности навигации
} NavigationState;

// Структура для навигационных данных
typedef struct {
    NavigationState state;
    GPSPosition homePosition;
    Waypoint currentWaypoint;
    uint8_t waypointCount;
    bool returnToHomeActive;
} NavigationData;

// Константы для навигации
#define MAX_WAYPOINTS 20
#define NAV_WAYPOINT_RADIUS    3.0f   // Радиус достижения точки (метры)
#define NAV_MIN_ALTITUDE       5.0f    // Минимальная безопасная высота
#define NAV_MAX_ALTITUDE       120.0f  // Максимальная разрешенная высота
#define NAV_DEFAULT_SPEED      5.0f    // Скорость по умолчанию (м/с)

// Основные функции навигации
void Navigation_Init(void);
void Navigation_Update(void);
void Navigation_SetHomePosition(GPSPosition* position);
void Navigation_Start(void);
void Navigation_Stop(void);
void Navigation_Pause(void);
void Navigation_Resume(void);
bool Navigation_IsActive(void);

// Функции управления маршрутом
bool Navigation_AddWaypoint(const Waypoint* waypoint);
bool Navigation_RemoveWaypoint(uint8_t index);
bool Navigation_GetWaypoint(uint8_t index, Waypoint* waypoint);
void Navigation_ClearWaypoints(void);

// Функции получения данных
void Navigation_GetCurrentPosition(GPSPosition* position);
float Navigation_GetCurrentAltitude(void);
float Navigation_GetCurrentHeading(void);
float Navigation_GetDistanceToTarget(void);
float Navigation_GetBearingToTarget(void);
bool Navigation_IsValidPosition(const GPSPosition* position);
bool Navigation_IsInSafeZone(const GPSPosition* position);

// Функции управления домашней позицией
void Navigation_UpdateHomePosition(const GPSPosition* position);

// Эмуляция для тестирования
void Navigation_EmulateGPSPosition(float latitude, float longitude, float altitude);
void Navigation_EmulateMovement(float speed, float heading);
void Navigation_EmulateAltitudeChange(float vertical_speed);

#endif // NAVIGATION_H
