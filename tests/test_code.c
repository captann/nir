// Глобальные переменные
int global_var = 42;
static int static_global_var = 10;

// Глобальная функция
void global_function(void) {
    // Локальные переменные
    int local_var = 0;
    static int static_local_var = 5;
    
    // Локальная функция
    void local_function(void) {
        int nested_local_var = 1;
    }
    
    // Использование переменных
    local_var = global_var;
}

// Тестирование констант
const int GLOBAL_CONST = 100;
#define MACRO_CONST 200

// Структура с разными типами полей
typedef struct TestStruct {
    int regular_field;
    static int static_field;
    const int const_field;
} TestStruct;

// Функция с параметрами разных типов
int complex_function(int param1, char* param2, void* param3) {
    // Локальные переменные разных типов
    int* pointer_var = NULL;
    static char static_char = 'A';
    const float const_local = 3.14f;
    
    return param1;
}
