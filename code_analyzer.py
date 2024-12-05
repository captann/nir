import os
import re
import json
from typing import Dict, List, Any
from dataclasses import dataclass, asdict
from pathlib import Path

@dataclass
class CodeLocation:
    file_path: str
    line_number: int
    context: str

@dataclass
class BaseItem:
    id: str
    name: str
    description: str
    file_path: str
    links_in_code: List[CodeLocation]
    comments: str
    warnings: str
    deprecated: bool
    tags: List[str]
    documentation_link: str
    is_deleted: bool  # Добавляем свойство is_deleted

@dataclass
class Function(BaseItem):
    input_params: List[Dict[str, str]]
    output_params: List[Dict[str, str]]
    is_static: bool
    scope: str
    dependencies: List[str]
    called_by: List[str]

@dataclass
class Constant(BaseItem):
    type: str
    value: str
    is_define: bool
    scope: str
    dependencies: List[str]

@dataclass
class Variable(BaseItem):
    type: str
    scope: str
    is_static: bool
    valid_range: Dict[str, Any]

@dataclass
class Structure(BaseItem):
    fields: List[Dict[str, str]]
    size: int
    alignment: int
    used_in_functions: List[str]
    inheritance: str
    packed: bool

class CodeAnalyzer:
    def __init__(self, root_dir: str):
        self.root_dir = Path(root_dir)
        self.functions: List[Function] = []
        self.constants: List[Constant] = []
        self.variables: List[Variable] = []
        self.structures: List[Structure] = []
        
        # Счетчики для генерации ID
        self._function_counter = 1
        self._constant_counter = 1
        self._variable_counter = 1
        self._structure_counter = 1
        
        # Множества для хранения уникальных идентификаторов
        self._function_ids = set()
        self._constant_ids = set()
        self._variable_ids = set()
        self._structure_ids = set()
        
        # Регулярные выражения для поиска определений
        self.re_function = re.compile(
            r'(?P<static>static\s+)?(?P<type>\w+[\s*]+)(?P<name>\w+)\s*\((?P<params>[^)]*)\)'
        )
        self.re_define = re.compile(
            r'#define\s+(?P<name>\w+)\s+(?P<value>[^\n]*)'
        )
        self.re_constant = re.compile(
            r'const\s+(?P<type>\w+)\s+(?P<name>\w+)\s*=\s*(?P<value>[^;]+);'
        )
        self.re_variable = re.compile(
            r'(?P<static>static\s+)?(?P<type>\w+[\s*]+)(?P<name>\w+)\s*;'
        )
        self.re_struct = re.compile(
            r'typedef\s+struct\s*(?:(?P<n>\w+)\s*)?{(?P<body>.*?)}(?:\s*(?P<alias>\w+))?;',
            re.DOTALL
        )
        
    def _is_unique_function(self, func: Function) -> bool:
        """Проверка уникальности функции"""
        func_id = f"{func.file_path}:{func.name}"
        if func_id in self._function_ids:
            return False
        self._function_ids.add(func_id)
        return True

    def _is_unique_constant(self, const: Constant) -> bool:
        """Проверка уникальности константы"""
        const_id = f"{const.file_path}:{const.name}"
        if const_id in self._constant_ids:
            return False
        self._constant_ids.add(const_id)
        return True

    def _is_unique_variable(self, var: Variable) -> bool:
        """Проверка уникальности переменной"""
        var_id = f"{var.file_path}:{var.name}"
        if var_id in self._variable_ids:
            return False
        self._variable_ids.add(var_id)
        return True

    def _is_unique_structure(self, struct: Structure) -> bool:
        """Проверка уникальности структуры"""
        struct_id = f"{struct.file_path}:{struct.name}"
        if struct_id in self._structure_ids:
            return False
        self._structure_ids.add(struct_id)
        return True
    
    def _generate_function_id(self) -> str:
        """Генерация ID для функции"""
        id = f"func_{self._function_counter:03d}"
        self._function_counter += 1
        return id

    def _generate_constant_id(self) -> str:
        """Генерация ID для константы"""
        id = f"const_{self._constant_counter:03d}"
        self._constant_counter += 1
        return id

    def _generate_variable_id(self) -> str:
        """Генерация ID для переменной"""
        id = f"var_{self._variable_counter:03d}"
        self._variable_counter += 1
        return id

    def _generate_structure_id(self) -> str:
        """Генерация ID для структуры"""
        id = f"struct_{self._structure_counter:03d}"
        self._structure_counter += 1
        return id

    def analyze_file(self, file_path: Path) -> None:
        """Анализ одного файла"""
        if not file_path.suffix in ['.c', '.h']:
            return
            
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
            lines = content.split('\n')
            
        # Анализ функций
        for match in self.re_function.finditer(content):
            if ';' not in match.group():  # Исключаем прототипы
                func = self._parse_function(match, file_path, lines, content)
                if func and self._is_unique_function(func):
                    self.functions.append(func)
        
        # Анализ констант
        for match in self.re_define.finditer(content):
            const = self._parse_define(match, file_path, lines, content)
            if const and self._is_unique_constant(const):
                self.constants.append(const)
                
        for match in self.re_constant.finditer(content):
            const = self._parse_constant(match, file_path, lines, content)
            if const and self._is_unique_constant(const):
                self.constants.append(const)
        
        # Анализ переменных
        for match in self.re_variable.finditer(content):
            var = self._parse_variable(match, file_path, lines, content)
            if var and self._is_unique_variable(var):
                self.variables.append(var)
        
        # Анализ структур
        for match in self.re_struct.finditer(content):
            struct = self._parse_struct(match, file_path, lines, content)
            if struct and self._is_unique_structure(struct):
                self.structures.append(struct)
    
    def _parse_function(self, match: re.Match, file_path: Path, lines: List[str], content: str) -> Function:
        """Парсинг функции"""
        name = match.group('name')
        static = bool(match.group('static'))
        return_type = match.group('type').strip()
        params_str = match.group('params')
        
        # Парсинг параметров
        input_params = []
        for param in params_str.split(','):
            param = param.strip()
            if param:
                parts = param.split()
                if len(parts) >= 2:
                    input_params.append({
                        'name': parts[-1].strip('*'),
                        'type': ' '.join(parts[:-1])
                    })
        
        # Поиск комментариев
        line_num = content[:match.start()].count('\n') + 1
        comments = self._extract_comments(lines, line_num)
        
        return Function(
            id=self._generate_function_id(),
            name=name,
            description=comments,
            file_path=str(file_path),
            links_in_code=self._find_references(name),
            comments=comments,
            warnings="",
            deprecated='deprecated' in comments.lower(),
            tags=self._extract_tags(comments),
            documentation_link="",
            input_params=input_params,
            output_params=[{'name': 'return', 'type': return_type}] if return_type != 'void' else [],
            is_static=static,
            scope='static' if static else 'global',
            dependencies=[],
            called_by=[],
            is_deleted=False  # Добавляем свойство is_deleted
        )
    
    def _parse_define(self, match: re.Match, file_path: Path, lines: List[str], content: str) -> Constant:
        """Парсинг #define константы"""
        name = match.group('name')
        value = match.group('value').strip()
        
        line_num = content[:match.start()].count('\n') + 1
        comments = self._extract_comments(lines, line_num)
        
        return Constant(
            id=self._generate_constant_id(),
            name=name,
            description=comments,
            file_path=str(file_path),
            links_in_code=self._find_references(name),
            comments=comments,
            warnings="",
            deprecated='deprecated' in comments.lower(),
            tags=self._extract_tags(comments),
            documentation_link="",
            type='define',
            value=value,
            is_define=True,
            scope='global',
            dependencies=[],
            is_deleted=False  # Добавляем свойство is_deleted
        )
    
    def _parse_constant(self, match: re.Match, file_path: Path, lines: List[str], content: str) -> Constant:
        """Парсинг const переменной"""
        name = match.group('name')
        type_name = match.group('type')
        value = match.group('value')
        
        line_num = content[:match.start()].count('\n') + 1
        comments = self._extract_comments(lines, line_num)
        
        return Constant(
            id=self._generate_constant_id(),
            name=name,
            description=comments,
            file_path=str(file_path),
            links_in_code=self._find_references(name),
            comments=comments,
            warnings="",
            deprecated='deprecated' in comments.lower(),
            tags=self._extract_tags(comments),
            documentation_link="",
            type=type_name,
            value=value,
            is_define=False,
            scope='global',
            dependencies=[],
            is_deleted=False  # Добавляем свойство is_deleted
        )
    
    def _parse_variable(self, match: re.Match, file_path: Path, lines: List[str], content: str) -> Variable:
        """Парсинг переменной"""
        name = match.group('name')
        type_name = match.group('type')
        static = bool(match.group('static'))
        
        line_num = content[:match.start()].count('\n') + 1
        comments = self._extract_comments(lines, line_num)
        
        return Variable(
            id=self._generate_variable_id(),
            name=name,
            description=comments,
            file_path=str(file_path),
            links_in_code=self._find_references(name),
            comments=comments,
            warnings="",
            deprecated='deprecated' in comments.lower(),
            tags=self._extract_tags(comments),
            documentation_link="",
            type=type_name.strip(),
            scope='static' if static else 'global',
            is_static=static,
            valid_range={},
            is_deleted=False  # Добавляем свойство is_deleted
        )
    
    def _parse_struct(self, match: re.Match, file_path: Path, lines: List[str], content: str) -> Structure:
        """Парсинг структуры"""
        name = match.group('n') or match.group('alias') or 'Unnamed'
        body = match.group('body')
        
        # Парсинг полей структуры
        fields = []
        line_num = content[:match.start()].count('\n') + 1
        current_comment = ''
        
        # Разбиваем тело структуры на строки для поиска комментариев
        body_lines = body.split('\n')
        for line in body_lines:
            line = line.strip()
            # Проверяем, является ли строка комментарием
            if '//' in line:
                current_comment = line.split('//')[-1].strip()
                continue
            
            # Если это объявление поля (содержит ;)
            if ';' in line:
                field_def = line.split(';')[0].strip()
                if field_def and '{' not in field_def and '}' not in field_def:
                    parts = field_def.split()
                    if len(parts) >= 2:
                        field_name = parts[-1].strip('*')
                        field_type = ' '.join(parts[:-1])
                        fields.append({
                            'name': field_name,
                            'type': field_type,
                            'description': current_comment
                        })
                current_comment = ''  # Сбрасываем текущий комментарий
        
        comments = self._extract_comments(lines, line_num)
        
        return Structure(
            id=self._generate_structure_id(),
            name=name,
            description=comments,
            file_path=str(file_path),
            links_in_code=self._find_references(name),
            comments=comments,
            warnings="",
            deprecated='deprecated' in comments.lower(),
            tags=self._extract_tags(comments),
            documentation_link="",
            fields=fields,
            size=0,
            alignment=0,
            used_in_functions=[],
            inheritance="",
            packed=False,
            is_deleted=False  # Добавляем свойство is_deleted
        )
    
    def _find_references(self, name: str) -> List[CodeLocation]:
        """Поиск использования идентификатора в коде"""
        references = []
        for file_path in self.root_dir.rglob('*.[ch]'):
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
                for i, line in enumerate(lines, 1):
                    if name in line and '//' not in line.split(name)[0]:
                        references.append(CodeLocation(
                            file_path=str(file_path),
                            line_number=i,
                            context=line.strip()
                        ))
        return references
    
    def _extract_comments(self, lines: List[str], line_num: int) -> str:
        """Извлечение комментариев перед определением"""
        comments = []
        i = line_num - 1
        while i >= 0 and (lines[i].strip().startswith('//') or lines[i].strip().startswith('/*')):
            comments.insert(0, lines[i].strip('/ *'))
            i -= 1
        return '\n'.join(comments)
    
    def _extract_tags(self, comments: str) -> List[str]:
        """Извлечение тегов из комментариев"""
        tags = []
        for line in comments.split('\n'):
            if '@tag' in line:
                tag = line.split('@tag')[-1].strip()
                if tag:
                    tags.append(tag)
        return tags
    
    def analyze_directory(self) -> None:
        """Анализ всей директории"""
        for file_path in self.root_dir.rglob('*.[ch]'):
            self.analyze_file(file_path)
    
    def _get_next_version_number(self) -> int:
        # Создаем папку versions, если она не существует
        versions_dir = Path('versions')
        versions_dir.mkdir(exist_ok=True)
        
        # Получаем список всех файлов в папке versions
        version_files = list(versions_dir.glob('code_data_*.json'))
        
        if not version_files:
            return 1
            
        # Извлекаем номера версий из имен файлов
        version_numbers = []
        for file in version_files:
            try:
                # Извлекаем номер версии из имени файла
                version = int(file.stem.split('_')[-1])
                version_numbers.append(version)
            except (ValueError, IndexError):
                continue
                
        # Возвращаем следующий номер версии
        return max(version_numbers, default=0) + 1

    def generate_json(self):
        """Генерация JSON с результатами анализа"""
        # Получаем следующий номер версии
        next_version = self._get_next_version_number()
        
        # Формируем имя файла
        versions_dir = Path('versions')
        json_file = versions_dir / f'code_data_{next_version}.json'
        
        # Создаем словарь с данными
        data = {
            'functions': [asdict(f) for f in self.functions],
            'constants': [asdict(c) for c in self.constants],
            'variables': [asdict(v) for v in self.variables],
            'structures': [asdict(s) for s in self.structures]
        }
        
        # Сохраняем в файл версии
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
            
        # Копируем последнюю версию в основной файл
        
        
        print(f'Данные сохранены в {json_file}')
        return data

def main():
    analyzer = CodeAnalyzer('d:/keil/prog/src2')
    analyzer.analyze_directory()
    
    # Сохранение результатов в файл
    analyzer.generate_json()

if __name__ == '__main__':
    main()