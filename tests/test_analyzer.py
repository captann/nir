import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from code_analyzer import CodeAnalyzer

def test_scope_analysis():
    analyzer = CodeAnalyzer("tests")
    analyzer.analyze_file("tests/test_scope.c")
    
    # Проверяем глобальные переменные
    global_vars = [v for v in analyzer.variables if v.scope == 'global']
    print("\nГлобальные переменные:")
    for var in global_vars:
        print(f"- {var.id}: {var.name} (static: {var.is_static})")
    
    # Проверяем локальные переменные
    local_vars = [v for v in analyzer.variables if v.scope == 'local']
    print("\nЛокальные переменные:")
    for var in local_vars:
        print(f"- {var.id}: {var.name} (static: {var.is_static})")
    
    # Проверяем функции
    global_funcs = [f for f in analyzer.functions if f.scope == 'global']
    print("\nГлобальные функции:")
    for func in global_funcs:
        print(f"- {func.id}: {func.name}")
    
    local_funcs = [f for f in analyzer.functions if f.scope == 'local']
    print("\nЛокальные функции:")
    for func in local_funcs:
        print(f"- {func.id}: {func.name}")
    
    # Проверяем константы
    print("\nКонстанты:")
    for const in analyzer.constants:
        print(f"- {const.id}: {const.name} (define: {const.is_define})")

if __name__ == '__main__':
    test_scope_analysis()
