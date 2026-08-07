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

## 📸 Интерфейс
ROBOT_ADDRESS = ("192.168.4.1", 8888)  # IP ESP32
LINEAR_SPEED_MM_S = 250                # Скорость
STEP_PX = 60                           # Шаг линий

## 👨‍💻 Автор
[Ваше имя] - [email]
