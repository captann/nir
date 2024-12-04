// Глобальные переменные и константы
int global_var = 42;
static int static_global = 10;
const int GLOBAL_CONST = 100;
#define MACRO_CONST 200

// Глобальная функция
int global_function(int param1) {
    // Локальные переменные
    int local_var = 0;
    static int static_local = 5;
    const float local_const = 3.14f;
    
    // Вложенная функция
    void nested_function(void) {
        int nested_var = 1;
    }
    
    return local_var;
}

// Тестовая структура
typedef struct {
    int field1;
    static int static_field;
} TestStruct;
