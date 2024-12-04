from flask import Flask, render_template, jsonify, request
import json
import os
import subprocess
import re
from datetime import datetime

app = Flask(__name__)

# Путь к JSON файлу
JSON_FILE = 'code_data.json'
BACKUP_DIR = 'backups'

def create_backup():
    """Create a backup of the current JSON file"""
    if not os.path.exists(BACKUP_DIR):
        os.makedirs(BACKUP_DIR)
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    backup_file = os.path.join(BACKUP_DIR, f'code_data_{timestamp}.json')
    
    try:
        if os.path.exists(JSON_FILE):
            with open(JSON_FILE, 'r', encoding='utf-8') as src:
                with open(backup_file, 'w', encoding='utf-8') as dst:
                    dst.write(src.read())
        return True
    except Exception as e:
        print(f"Ошибка при создании резервной копии: {str(e)}")
        return False

def load_json_data():
    """Load data from JSON file with error handling"""
    try:
        with open(JSON_FILE, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        return {
            'functions': [],
            'constants': [],
            'variables': [],
            'structures': []
        }
    except json.JSONDecodeError:
        if os.path.exists(JSON_FILE):
            # Create backup of corrupted file
            create_backup()
        return {
            'functions': [],
            'constants': [],
            'variables': [],
            'structures': []
        }

def save_json_data(data):
    """Save data to JSON file with backup"""
    try:
        create_backup()  # Create backup before saving
        with open(JSON_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        print(f"Ошибка при сохранении данных: {str(e)}")
        return False

@app.route('/')
def index():
    return render_template('index.html')

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
    app.run(debug=True, port=5000)
