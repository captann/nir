from flask import Flask, render_template, jsonify, request
import json
import os
import subprocess
import re
from datetime import datetime
from pathlib import Path
import argparse

app = Flask(__name__)
BACKUP_DIR = 'backups'
VERSIONS_DIR = 'versions'
VERSION = None  # Глобальная переменная для хранения запрошенной версии

def get_latest_version_file():
    """Получает путь к файлу с последней версией данных"""
    versions_dir = Path(VERSIONS_DIR)
    if not versions_dir.exists():
        raise FileNotFoundError("Папка versions не найдена")
        
    # Получаем список всех файлов версий
    version_files = list(versions_dir.glob('code_data_*.json'))
    if not version_files:
        raise FileNotFoundError("Файлы версий не найдены")
    
    # Извлекаем номера версий и находим максимальный
    latest_file = max(
        version_files,
        key=lambda f: int(f.stem.split('_')[-1])
    )
    
    return str(latest_file)

def get_version_file(version=None):
    """Получает путь к файлу конкретной версии или последней версии"""
    versions_dir = Path(VERSIONS_DIR)
    if not versions_dir.exists():
        raise FileNotFoundError("Папка versions не найдена")
        
    # Если указана конкретная версия
    if version is not None:
        specific_file = versions_dir / f'code_data_{version}.json'
        if specific_file.exists():
            return str(specific_file)
        else:
            print(f"Версия {version} не найдена, будет использована последняя версия")
    
    # Получаем список всех файлов версий
    version_files = list(versions_dir.glob('code_data_*.json'))
    if not version_files:
        raise FileNotFoundError("Файлы версий не найдены")
    
    # Извлекаем номера версий и находим максимальный
    latest_file = max(
        version_files,
        key=lambda f: int(f.stem.split('_')[-1])
    )
    
    return str(latest_file)

# Путь к JSON файлу


def create_backup():
    """Create a backup of the current JSON file"""
    if not os.path.exists(BACKUP_DIR):
        os.makedirs(BACKUP_DIR)
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    backup_file = os.path.join(BACKUP_DIR, f'code_data_{timestamp}.json')
    
    try:
        if os.path.exists(get_json_file()):
            with open(get_json_file(), 'r', encoding='utf-8') as src:
                with open(backup_file, 'w', encoding='utf-8') as dst:
                    dst.write(src.read())
        return True
    except Exception as e:
        print(f"Ошибка при создании резервной копии: {str(e)}")
        return False


# Путь к файлу данных теперь будет динамическим
def get_json_file():
    try:
        return get_version_file(VERSION)
    except FileNotFoundError as e:
        print(f"Ошибка: {e}")
        return 'code_data.json'  # Fallback к старому варианту

def load_json_data():
    """Загрузка данных из JSON файла"""
    try:
        json_file = get_json_file()
        with open(json_file, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception as e:
        print(f"Ошибка при загрузке данных: {e}")
        return {
            'functions': [],
            'constants': [],
            'variables': [],
            'structures': []
        }

def save_json_data(data):
    """Сохранение данных в JSON файл"""
    try:

        with open(get_json_file(), 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
            
        print(f'Данные сохранены в {get_json_file()}')
        return True
    except Exception as e:
        print(f"Ошибка при сохранении данных: {e}")
        return False

@app.route('/')
def index():
    try:
        data = load_json_data()
        version_text = f"Версия {VERSION}" if VERSION is not None else "Последняя версия"
        current_file = os.path.abspath(get_json_file())  # Получаем абсолютный путь
        return render_template('index.html', data=data, version_info=version_text, current_file=current_file)
    except Exception as e:
        return f"Ошибка при загрузке данных: {str(e)}"

@app.route('/api/data')
def get_data():
    data = load_json_data()
    return jsonify(data)

@app.route('/api/update', methods=['POST'])
def update_item():
    data = request.json
    item_id = data.get('id')
    try:
        # Проверяем наличие id
        if not item_id:
            return jsonify({
                'success': False,
                'error': 'Отсутствует ID объекта'
            })

        # Загружаем текущие данные
        current_data = load_json_data()
        item_type = data.get('type')
        if not item_type:
            return jsonify({
                'success': False,
                'error': 'Не указан тип объекта'
            })

        prefix_map = {
            'functions': 'func',
            'variables': 'var',
            'constants': 'const',
            'structures': 'struct'
        }
        prefix = prefix_map.get(item_type, 'item')
        
        # Убедимся, что категория существует в словаре
        if item_type not in current_data:
            current_data[item_type] = []

        if item_id == 'new':
            # Находим максимальный номер для данного типа
            max_num = 0
            pattern = f"{prefix}_"
            for item in current_data[item_type]:
                curr_id = item.get('id', '')
                if curr_id.startswith(pattern):
                    try:
                        num = int(curr_id.split('_')[1])
                        max_num = max(max_num, num)
                    except (IndexError, ValueError):
                        continue
            
            # Генерируем новый id
            new_id = f"{prefix}_{str(max_num + 1).zfill(3)}"
            
            # Устанавливаем новый id в данные
            data['id'] = new_id
            item_id = new_id
            # Добавляем новый элемент в соответствующую категорию
            current_data[item_type].append(data)

        
        # Находим и обновляем объект в соответствующей категории
        item_updated = False
        categories = ['functions', 'constants', 'variables', 'structures']
        for category in categories:
            items = current_data.get(category, [])
            for i, item in enumerate(items):
                if item.get('id') == item_id:
                    # Обновляем существующие поля, сохраняя неизменные
                    items[i] = {**item, **data}
                    item_updated = True
                    break
            if item_updated:
                break
        
        if not item_updated:
            return jsonify({
                'success': False,
                'error': f'Объект с ID {item_id} не найден'
            })
            
        # Сохраняем обновленные данные
        if save_json_data(current_data):
            return jsonify({
                'success': True,
                'message': f'Объект с ID {item_id} успешно обновлен'
            })
        else:
            return jsonify({
                'success': False,
                'error': 'Ошибка при сохранении данных'
            })

    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@app.route('/api/validate', methods=['POST'])
def validate_data():
    """Validate data before updating"""
    try:
        data = request.json
        item_type = data.get('type')
        item_data = data.get('data')

        if not all([item_type, item_data]):
            return jsonify({
                'valid': False,
                'error': 'Отсутствуют необходимые данные'
            })

        # Базовая валидация для разных типов
        if item_type == 'functions':
            required_fields = ['name', 'description', 'scope']
            if not all(field in item_data for field in required_fields):
                return jsonify({
                    'valid': False,
                    'error': 'Отсутствуют обязательные поля для функции'
                })
        elif item_type == 'variables':
            if 'name' not in item_data or 'type' not in item_data:
                return jsonify({
                    'valid': False,
                    'error': 'Отсутствуют обязательные поля для переменной'
                })
        elif item_type == 'constants':
            if 'name' not in item_data or 'value' not in item_data:
                return jsonify({
                    'valid': False,
                    'error': 'Отсутствуют обязательные поля для константы'
                })
        elif item_type == 'structures':
            if 'name' not in item_data or 'fields' not in item_data:
                return jsonify({
                    'valid': False,
                    'error': 'Отсутствуют обязательные поля для структуры'
                })

        return jsonify({
            'valid': True,
            'message': 'Данные прошли валидацию'
        })

    except Exception as e:
        return jsonify({
            'valid': False,
            'error': str(e)
        })

@app.route('/api/delete', methods=['POST'])
def delete_item():
    try:
        data = request.get_json()
        item_id = data.get('id')
        
        if not item_id:
            return jsonify({
                'success': False,
                'error': 'ID не указан'
            }), 400
            
        # Загружаем текущие данные
        current_data = load_json_data()
        
        # Ищем объект в каждой категории
        item_found = False
        for category in current_data:
            for item in current_data[category]:
                if item.get('id') == item_id:
                    item['is_deleted'] = True
                    item_found = True
                    break
            if item_found:
                break
        
        if not item_found:
            return jsonify({
                'success': False,
                'error': f'Элемент с id {item_id} не найден'
            }), 404
            
        # Создаем бэкап перед сохранением
        create_backup()
        
        # Сохраняем обновленные данные
        with open(get_json_file(), 'w', encoding='utf-8') as f:
            json.dump(current_data, f, ensure_ascii=False, indent=4)
            
        return jsonify({
            'success': True,
            'message': 'Объект помечен как удаленный'
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/open_file')
def open_file():
    try:
        file_path = request.args.get('file')
        line_number = request.args.get('line', 1, type=int)
        
        print('1. Полученный путь:', file_path)
        
        # Если путь начинается с диска, используем его как есть
        if re.match(r'^[a-zA-Z]:', file_path):
            abs_path = file_path
        else:
            # Иначе считаем путь относительным от корня проекта
            abs_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), file_path)
        
        # Нормализуем путь
        abs_path = os.path.normpath(abs_path)
        print('2. Абсолютный путь:', abs_path)
        print('3. Путь существует:', os.path.exists(abs_path))
        
        # Проверяем существование файла
        if not os.path.exists(abs_path):
            return jsonify({
                'success': False, 
                'error': f'Файл не найден: {abs_path}\nТекущая директория: {os.getcwd()}'
            })
        
        # Пути к VSCode в разных местах установки
        vscode_paths = [
            os.path.expandvars(r'%LOCALAPPDATA%\Programs\Microsoft VS Code\Code.exe'),
            os.path.expandvars(r'%ProgramFiles%\Microsoft VS Code\Code.exe'),
            os.path.expandvars(r'%ProgramFiles(x86)%\Microsoft VS Code\Code.exe'),
            r'C:\Program Files\Microsoft VS Code\Code.exe',
            r'C:\Program Files (x86)\Microsoft VS Code\Code.exe'
        ]
        
        # Ищем существующий путь к VSCode
        vscode_path = None
        for path in vscode_paths:
            if os.path.exists(path):
                vscode_path = path
                break
        
        if not vscode_path:
            return jsonify({
                'success': False,
                'error': 'VSCode не найден. Убедитесь, что он установлен в одной из стандартных директорий.'
            })
        
        print('4. Путь к VSCode:', vscode_path)
        
        # Формируем команду для VSCode
        vscode_command = [vscode_path, '--goto', f'{abs_path}:{line_number}']
        print('5. Команда VSCode:', ' '.join(vscode_command))
        
        # Запускаем VSCode с параметрами
        result = subprocess.run(vscode_command, capture_output=True, text=True)
        if result.returncode != 0:
            return jsonify({
                'success': False,
                'error': f'Ошибка запуска VSCode:\nStdout: {result.stdout}\nStderr: {result.stderr}'
            })
        
        return jsonify({'success': True})
    except Exception as e:
        import traceback
        return jsonify({
            'success': False, 
            'error': f'Ошибка при открытии файла:\n{str(e)}\n\nStack trace:\n{traceback.format_exc()}'
        })

if __name__ == '__main__':
    # Настраиваем парсер аргументов командной строки
    parser = argparse.ArgumentParser(description='Запуск веб-сервера для анализа кода')
    parser.add_argument('-v', '--version', type=int, help='Номер версии для загрузки')
    args = parser.parse_args()
    
    # Устанавливаем глобальную версию
    VERSION = args.version
    
    if VERSION is not None:
        try:
            json_file = get_version_file(VERSION)
            print(f"Используется файл версии {VERSION}: {json_file}")
        except FileNotFoundError:
            print(f"Версия {VERSION} не найдена, будет использована последняя версия")
            VERSION = None
    
    app.run(debug=True, port=5000)