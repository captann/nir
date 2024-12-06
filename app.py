from flask import Flask, render_template, jsonify, request, send_from_directory
import json
import os
import subprocess
import re
from datetime import datetime
from pathlib import Path
import argparse
import urllib.parse
from werkzeug.middleware.proxy_fix import ProxyFix

from flask_cors import CORS
app = Flask(__name__)

app.wsgi_app = ProxyFix(
    app.wsgi_app,
    x_for=1,      # Количество прокси для X-Forwarded-For
    x_proto=1,    # Количество прокси для X-Forwarded-Proto
    x_host=1,     # Количество прокси для X-Forwarded-Host
    x_prefix=1    # Количество прокси для X-Forwarded-Prefix
)

CORS(app)
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
        if not os.path.exists(json_file):
            raise FileNotFoundError(f"Файл не найден: {json_file}")
            
        with open(json_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
            if not isinstance(data, dict):
                raise ValueError("Некорректный формат JSON")
            return data
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

def get_latest_version_number():
    """Получает номер последней версии"""
    try:
        versions_dir = Path(VERSIONS_DIR)
        if not versions_dir.exists():
            return None
        
        version_files = list(versions_dir.glob('code_data_*.json'))
        if not version_files:
            return None
        
        return max(
            int(f.stem.split('_')[-1])
            for f in version_files
        )
    except Exception:
        return None

@app.route('/')
def index():
    try:
        data = load_json_data()
        latest_version = get_latest_version_number()
        
        # Проверяем, является ли версия допустимой
        if VERSION is not None and VERSION <= 0:
            print(f"Указана недопустимая версия {VERSION}, будет использована последняя версия")
            return render_template('index.html', data=data, 
                                version_info=f"Версия файла: последняя", 
                                current_file=os.path.abspath(get_json_file()))
        
        # Проверяем, является ли текущая версия последней
        is_latest = VERSION is None or (latest_version is not None and VERSION >= latest_version)
        version_text = "Версия файла: последняя" if is_latest else f"Версия файла: {VERSION} из {latest_version}"
        
        current_file = os.path.abspath(get_json_file())
        return render_template('index.html', data=data, version_info=version_text, current_file=current_file)
    except Exception as e:
        return f"Ошибка при загрузке данных: {str(e)}"

@app.route('/api/data')
def get_data():
    try:
        data = load_json_data()
        response = jsonify(data)
        response.headers.add('Access-Control-Allow-Origin', '*')  # Разрешаем CORS
        return response
    except Exception as e:
        error_response = jsonify({
            'error': str(e),
            'message': 'Ошибка при загрузке данных'
        })
        error_response.headers.add('Access-Control-Allow-Origin', '*')
        error_response.status_code = 500
        return error_response

@app.route('/api/update', methods=['POST'])
def update_item():
    try:
        data = request.json
        item_id = data.get('id')
        
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
                    
                    # Преобразуем формат links_in_code
                    if 'links_in_code' in data:

                        items[i]['links_in_code'] = [{
                            'file_path': link.get('file', ''),  # кодируем путь, сохраняя слеши
                            'line_number': link.get('line', 1)
                        } for link in data['links_in_code']]
                    
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
        print(e.args)
        return jsonify({
            'success': False,
            'error': str(e)
        })


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




@app.route('/favicon.ico')
def favicon():
    # Определяем предпочтительную цветовую схему из заголовка запроса
    prefers_dark = request.headers.get('Sec-CH-Prefers-Color-Scheme', 'light') == 'dark'
    
    # Выбираем соответствующий файл иконки
    icon_file = 'favicon-dark.ico' if prefers_dark else 'favicon-light.ico'
    
    return send_from_directory(os.path.join(app.root_path, 'static'),
                             icon_file, mimetype='image/vnd.microsoft.icon')

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
    app.run(debug=True, port=5000, host='0.0.0.0',threaded=True )