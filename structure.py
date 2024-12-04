class Func:
    """Класс для описания функций в проекте
    
    Attributes:
        # Поля, видимые в списке:
        id (str): Уникальный идентификатор
        name (str): Имя функции
        description (str): Описание назначения функции
        scope (str): Область видимости (public/private/static)
        deprecated (bool): Флаг устаревшей функции
        
        # Поля, видимые только в детальном просмотре:
        input_params (list): Список входных параметров [{"name": "type"}]
        output_params (list): Список выходных параметров [{"name": "type"}]
        links_in_code (list): Ссылки на места использования в коде
        file_path (str): Путь к файлу, где определена функция
        is_static (bool): Флаг статической функции
        dependencies (list): Список вызываемых функций
        called_by (list): Список функций, которые вызывают эту функцию
        comments (str): Важные комментарии из кода
        warnings (str): Предупреждения или особые замечания
        tags (list): Теги для группировки и поиска
        documentation_link (str): Ссылка на документацию
    """
    def __init__(self, id="", name="", description="",
                 input_params=[{"name": "type"}], output_params=[{"name": "type"}], 
                 links_in_code=[], file_path="", is_static=False, scope="",
                 dependencies=[], called_by=[], comments="", warnings="",
                 deprecated=False, tags=[], documentation_link=""):
        self.id = id
        self.name = name
        self.description = description
        self.input_params = input_params
        self.output_params = output_params
        self.links_in_code = links_in_code
        self.file_path = file_path
        self.is_static = is_static
        self.scope = scope
        self.dependencies = dependencies
        self.called_by = called_by
        self.comments = comments
        self.warnings = warnings
        self.deprecated = deprecated
        self.tags = tags
        self.documentation_link = documentation_link


class Consts:
    """Класс для описания констант в проекте
    
    Attributes:
        # Поля, видимые в списке:
        id (str): Уникальный идентификатор
        name (str): Имя константы
        description (str): Описание назначения константы
        type (str): Тип данных константы
        value (any): Значение константы
        scope (str): Область видимости
        deprecated (bool): Флаг устаревшей константы
        
        # Поля, видимые только в детальном просмотре:
        links_in_code (list): Ссылки на места использования в коде
        file_path (str): Путь к файлу, где определена константа
        is_define (bool): Определена через #define или как const
        dependencies (list): Зависимые константы
        comments (str): Важные комментарии из кода
        warnings (str): Предупреждения или особые замечания
        tags (list): Теги для группировки и поиска
        documentation_link (str): Ссылка на документацию
    """
    def __init__(self, id="", name="", description="", type="", value=0,
                 links_in_code=[], file_path="", is_define=True, scope="",
                 dependencies=[], comments="", warnings="",
                 deprecated=False, tags=[], documentation_link=""):
        self.id = id
        self.name = name
        self.description = description
        self.type = type
        self.value = value
        self.links_in_code = links_in_code
        self.file_path = file_path
        self.is_define = is_define
        self.scope = scope
        self.dependencies = dependencies
        self.comments = comments
        self.warnings = warnings
        self.deprecated = deprecated
        self.tags = tags
        self.documentation_link = documentation_link


class Peremennaya:
    """Класс для описания переменных в проекте
    
    Attributes:
        # Поля, видимые в списке:
        id (str): Уникальный идентификатор
        name (str): Имя переменной
        description (str): Описание назначения переменной
        type (str): Тип данных переменной
        scope (str): Область видимости
        deprecated (bool): Флаг устаревшей переменной
        
        # Поля, видимые только в детальном просмотре:
        links_in_code (list): Ссылки на места использования в коде
        file_path (str): Путь к файлу
        is_static (bool): Флаг static
        valid_range (dict): Допустимый диапазон значений {min: value, max: value}
        comments (str): Важные комментарии из кода
        warnings (str): Предупреждения или особые замечания
        tags (list): Теги для группировки и поиска
        documentation_link (str): Ссылка на документацию
    """
    def __init__(self, id="", name="", description="", type="",
                 links_in_code=[], file_path="", scope="", is_static=False,
                 valid_range={}, comments="", warnings="",
                 deprecated=False, tags=[], documentation_link=""):
        self.id = id
        self.name = name
        self.description = description
        self.type = type
        self.links_in_code = links_in_code
        self.file_path = file_path
        self.scope = scope
        self.is_static = is_static
        self.valid_range = valid_range
        self.comments = comments
        self.warnings = warnings
        self.deprecated = deprecated
        self.tags = tags
        self.documentation_link = documentation_link


class Structure:
    """Класс для описания структур в проекте
    
    Attributes:
        # Поля, видимые в списке:
        id (str): Уникальный идентификатор
        name (str): Имя структуры
        description (str): Описание назначения структуры
        size (int): Размер структуры в байтах
        deprecated (bool): Флаг устаревшей структуры
        
        # Поля, видимые только в детальном просмотре:
        fields (list): Список полей [{"name": ["type", "description"]}]
        links_in_code (list): Ссылки на места использования в коде
        file_path (str): Путь к файлу
        alignment (int): Выравнивание структуры
        used_in_functions (list): Список использующих функций
        inheritance (str): Родительская структура
        packed (bool): Флаг packed структуры
        comments (str): Важные комментарии из кода
        warnings (str): Предупреждения или особые замечания
        tags (list): Теги для группировки и поиска
        documentation_link (str): Ссылка на документацию
    """
    def __init__(self, id="", name="", description="", 
                 fields=[{"name": ["type", "description"]}],
                 links_in_code=[], file_path="", size=0, alignment=0,
                 used_in_functions=[], inheritance="", packed=False,
                 comments="", warnings="", deprecated=False,
                 tags=[], documentation_link=""):
        self.id = id
        self.name = name
        self.description = description
        self.fields = fields
        self.links_in_code = links_in_code
        self.file_path = file_path
        self.size = size
        self.alignment = alignment
        self.used_in_functions = used_in_functions
        self.inheritance = inheritance
        self.packed = packed
        self.comments = comments
        self.warnings = warnings
        self.deprecated = deprecated
        self.tags = tags
        self.documentation_link = documentation_link
