#include <math.h>
#include <string.h>
#include "../include/navigation.h"

// Константа для преобразования градусов в радианы
#define DEG_TO_RAD(x) ((x) * 0.017453292519943295)
#define RAD_TO_DEG(x) ((x) * 57.295779513082323)

// Радиус Земли в метрах
#define EARTH_RADIUS 6371000.0f

// Внутренние переменные модуля
static NavigationState nav_state = {0};
static Waypoint waypoints[MAX_WAYPOINTS];
static uint8_t waypoint_count = 0;
static uint8_t current_waypoint_index = 0;
static GPSPosition home_position = {0};
static bool home_position_set = false;

// Вспомогательные функции
static void UpdateNavigationState(void);
static float ClampHeading(float heading);

void Navigation_Init(void) {
    memset(&nav_state, 0, sizeof(NavigationState));
    memset(waypoints, 0, sizeof(waypoints));
    waypoint_count = 0;
    current_waypoint_index = 0;
    nav_state.navigation_active = false;
    
    // Установка начальных значений
    nav_state.current_position.altitude = NAV_MIN_ALTITUDE;
    nav_state.heading = 0.0f;
    nav_state.ground_speed = 0.0f;
    nav_state.vertical_speed = 0.0f;
}

void Navigation_Reset(void) {
    Navigation_Stop();
    Navigation_ClearWaypoints();
    Navigation_Init();
}

bool Navigation_AddWaypoint(const Waypoint* waypoint) {
    if (waypoint == NULL || waypoint_count >= MAX_WAYPOINTS) {
        return false;
    }
    
    if (!Navigation_IsValidPosition(&waypoint->position)) {
        return false;
    }
    
    waypoints[waypoint_count] = *waypoint;
    waypoints[waypoint_count].reached = false;
    waypoint_count++;
    
    return true;
}

bool Navigation_RemoveWaypoint(uint8_t index) {
    if (index >= waypoint_count) {
        return false;
    }
    
    // Сдвигаем все последующие точки
    for (uint8_t i = index; i < waypoint_count - 1; i++) {
        waypoints[i] = waypoints[i + 1];
    }
    
    waypoint_count--;
    
    // Корректируем текущий индекс если необходимо
    if (current_waypoint_index > index) {
        current_waypoint_index--;
    }
    
    return true;
}

void Navigation_ClearWaypoints(void) {
    memset(waypoints, 0, sizeof(waypoints));
    waypoint_count = 0;
    current_waypoint_index = 0;
}

uint8_t Navigation_GetWaypointCount(void) {
    return waypoint_count;
}

bool Navigation_GetWaypoint(uint8_t index, Waypoint* waypoint) {
    if (index >= waypoint_count || waypoint == NULL) {
        return false;
    }
    
    *waypoint = waypoints[index];
    return true;
}

void Navigation_Start(void) {
    if (waypoint_count > 0) {
        nav_state.navigation_active = true;
        current_waypoint_index = 0;
    }
}

void Navigation_Pause(void) {
    nav_state.navigation_active = false;
}

void Navigation_Resume(void) {
    if (waypoint_count > 0) {
        nav_state.navigation_active = true;
    }
}

void Navigation_Stop(void) {
    nav_state.navigation_active = false;
    current_waypoint_index = 0;
}

bool Navigation_IsActive(void) {
    return nav_state.navigation_active;
}

void Navigation_GetCurrentState(NavigationState* state) {
    if (state != NULL) {
        *state = nav_state;
    }
}

void Navigation_GetCurrentPosition(GPSPosition* position) {
    if (position != NULL) {
        *position = nav_state.current_position;
    }
}

float Navigation_GetCurrentAltitude(void) {
    return nav_state.current_position.altitude;
}

float Navigation_GetCurrentHeading(void) {
    return nav_state.heading;
}

float Navigation_GetDistanceToTarget(void) {
    return nav_state.distance_to_target;
}

float Navigation_GetBearingToTarget(void) {
    return nav_state.bearing_to_target;
}

// Функции эмуляции
void Navigation_EmulateGPSPosition(float latitude, float longitude, float altitude) {
    nav_state.current_position.latitude = latitude;
    nav_state.current_position.longitude = longitude;
    nav_state.current_position.altitude = altitude;
    nav_state.current_position.valid = true;
    
    UpdateNavigationState();
}

void Navigation_EmulateMovement(float speed, float heading) {
    // Преобразуем скорость из м/с в градусы/с (приближенно)
    float speed_deg = speed / (EARTH_RADIUS * DEG_TO_RAD(1.0f));
    
    // Обновляем позицию
    float heading_rad = DEG_TO_RAD(heading);
    nav_state.current_position.latitude += speed_deg * cosf(heading_rad);
    nav_state.current_position.longitude += speed_deg * sinf(heading_rad);
    
    nav_state.ground_speed = speed;
    nav_state.heading = heading;
    
    UpdateNavigationState();
}

void Navigation_EmulateAltitudeChange(float vertical_speed) {
    nav_state.current_position.altitude += vertical_speed;
    
    // Ограничиваем высоту
    if (nav_state.current_position.altitude < NAV_MIN_ALTITUDE) {
        nav_state.current_position.altitude = NAV_MIN_ALTITUDE;
        vertical_speed = 0;
    } else if (nav_state.current_position.altitude > NAV_MAX_ALTITUDE) {
        nav_state.current_position.altitude = NAV_MAX_ALTITUDE;
        vertical_speed = 0;
    }
    
    nav_state.vertical_speed = vertical_speed;
}

float Navigation_CalculateDistance(const GPSPosition* pos1, const GPSPosition* pos2) {
    if (!pos1->valid || !pos2->valid) {
        return 0.0f;
    }
    
    // Используем формулу гаверсинусов
    float lat1 = DEG_TO_RAD(pos1->latitude);
    float lat2 = DEG_TO_RAD(pos2->latitude);
    float dlon = DEG_TO_RAD(pos2->longitude - pos1->longitude);
    float dlat = lat2 - lat1;
    
    float a = sinf(dlat/2) * sinf(dlat/2) + 
              cosf(lat1) * cosf(lat2) * sinf(dlon/2) * sinf(dlon/2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    
    return EARTH_RADIUS * c;
}

float Navigation_CalculateBearing(const GPSPosition* pos1, const GPSPosition* pos2) {
    if (!pos1->valid || !pos2->valid) {
        return 0.0f;
    }
    
    float lat1 = DEG_TO_RAD(pos1->latitude);
    float lat2 = DEG_TO_RAD(pos2->latitude);
    float dlon = DEG_TO_RAD(pos2->longitude - pos1->longitude);
    
    float y = sinf(dlon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon);
    float bearing = RAD_TO_DEG(atan2f(y, x));
    
    return ClampHeading(bearing);
}

bool Navigation_IsValidPosition(const GPSPosition* position) {
    if (position == NULL) {
        return false;
    }
    
    // Проверяем диапазоны координат
    if (position->latitude < -90.0f || position->latitude > 90.0f ||
        position->longitude < -180.0f || position->longitude > 180.0f ||
        position->altitude < NAV_MIN_ALTITUDE || position->altitude > NAV_MAX_ALTITUDE) {
        return false;
    }
    
    return true;
}

bool Navigation_IsInSafeZone(const GPSPosition* position) {
    if (!position->valid || !home_position_set) {
        return false;
    }
    
    // Проверяем расстояние до домашней точки
    float distance = Navigation_CalculateDistance(&home_position, position);
    return distance <= 1000.0f; // Безопасная зона - 1 км от дома
}

void Navigation_UpdateHomePosition(const GPSPosition* position) {
    if (position != NULL && Navigation_IsValidPosition(position)) {
        home_position = *position;
        home_position_set = true;
    }
}

float Navigation_GetDistanceToHome(void) {
    if (!home_position_set) {
        return 0.0f;
    }
    return Navigation_CalculateDistance(&nav_state.current_position, &home_position);
}

// Заглушка для функции навигации
void Navigation_Update(void) { }

// Внутренние вспомогательные функции
static void UpdateNavigationState(void) {
    if (!nav_state.navigation_active || waypoint_count == 0 || 
        current_waypoint_index >= waypoint_count) {
        return;
    }
    
    // Обновляем расстояние и направление до текущей точки
    const Waypoint* target = &waypoints[current_waypoint_index];
    nav_state.distance_to_target = Navigation_CalculateDistance(
        &nav_state.current_position, &target->position);
    nav_state.bearing_to_target = Navigation_CalculateBearing(
        &nav_state.current_position, &target->position);
    
    // Проверяем достижение точки
    if (nav_state.distance_to_target <= NAV_WAYPOINT_RADIUS) {
        waypoints[current_waypoint_index].reached = true;
        if (current_waypoint_index < waypoint_count - 1) {
            current_waypoint_index++;
        } else {
            nav_state.navigation_active = false; // Достигли последней точки
        }
    }
}

static float ClampHeading(float heading) {
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    return heading;
}
