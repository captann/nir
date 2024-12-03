from flask import Flask, render_template, jsonify, request
import json
import os
import subprocess
import re

app = Flask(__name__)

# Путь к JSON файлу
JSON_FILE = 'code_data.json'

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/data')
def get_data():
    try:
        with open(JSON_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return jsonify(data)
    except FileNotFoundError:
        return jsonify({
            'error': 'Файл code_data.json не найден',
            'functions': [],
            'constants': [],
            'variables': [],
            'structures': []
        })
    except json.JSONDecodeError:
        return jsonify({
            'error': 'Ошибка при чтении JSON файла',
            'functions': [],
            'constants': [],
            'variables': [],
            'structures': []
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
