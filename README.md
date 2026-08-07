# robot-area-painter
**Автономный робот для закрашивания прямоугольной области по ArUco-маркерам.**
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.5+-green.svg)](https://opencv.org/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

---

## 📸 Демонстрация
<a href="docs/videos/demo.mp4">🎥 Видео</a>

<img src="docs/images/setup.jpg" alt="Настройка" width="300"/>
<img src="docs/images/process.jpg" alt="Процесс" width="300"/>
<img src="docs/images/result.jpg" alt="Результат" width="300"/>
<img src="docs/images/demo.gif" alt="GIF" width="500"/>

---

## 📋 Описание
Робот определяет область по 4 ArUco-маркерам (ID 1-4) и закрашивает её зигзагом. Позиция по маркеру ID 0.
**Возможности:**
- Автоопределение области
- Зигзагообразное движение
- Остановка по чёрной границе
- Пауза (Space)

---

## 📁 Структура
robot-area-painter/
├── arduino/ # Прошивка Arduino Uno
├── esp32/ # Прошивка ESP32-C3
├── python/ # Python код
└── docs/
├── images/ # 📸 Фото
├── videos/ # 🎥 Видео
└── presentation/ # 📊 Презентация

## 🎮 Управление
Клавиша |	Действие
Space |	Пауза
Esc| 	Остановка

## 🔧 Параметры
Параметр	Значение	Описание
LINEAR_SPEED_MM_S	250	Скорость движения
ANGULAR_SPEED_MRAD_S	3500	Скорость поворота
TARGET_TOLERANCE_PX	55	Допуск цели
STEP_PX	60	Шаг линий
MARGIN_PX	25	Отступ от краёв

## 🐛 Устранение проблем
Проблема	Решение

Камера не работает -> Изменить CAMERA_INDEX = 1 \

Робот не подключается	-> Проверить IP: ROBOT_ADDRESS = ("192.168.4.1", 8888) \
Маркеры не видны -> Проверить освещение, контрастность \
Arduino не отвечает	-> Проверить UART подключение \


## 📸 Скриншоты интерфейса
<img src="docs/images/ui_main.jpg" alt="Интерфейс" width="600"/> <img src="docs/images/ui_path.jpg" alt="Маршрут" width="600"/>

## 👨‍💻 Автор
[Ваше имя] - [email]
