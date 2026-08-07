#!/usr/bin/env python3
"""Робот закрашивает прямоугольную область, образованную четырьмя ArUco-маркерами.

Робот определяет своё положение по маркеру ID 0. Четыре маркера (ID 1, 2, 3, 4)
задают углы прямоугольной области. Робот движется зигзагом внутри области,
закрашивая её полностью.

Управление:
    Space — пауза / продолжение;
    Esc   — безопасно остановить робота и завершить программу.

Перед запуском:
    1. Подключить компьютер к Wi-Fi сети ESP32.
    2. Проверить ROBOT_ADDRESS.
    3. Откалибровать пороги датчиков линии.
    4. Закрепить маркер ID 0 стороной P0–P1 вперёд по корпусу.
    5. Разместить маркеры ID 1, 2, 3, 4 по углам прямоугольной области.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import socket
import time

import cv2
import numpy as np


CAMERA_INDEX = 0
ROBOT_MARKER_ID = 0
CORNER_MARKER_IDS = (1, 2, 3, 4)
ROBOT_ADDRESS = ("192.168.4.1", 8888)

# Оптимальные скорости для плавного движения
LINEAR_SPEED_MM_S = 333
ANGULAR_SPEED_MRAD_S = 4000
TARGET_TOLERANCE_PX = 25
ANGLE_TOLERANCE_RAD = math.radians(5)  # Уменьшен для точных поворотов
SEND_PERIOD_SECONDS = 0.05
TELEMETRY_TIMEOUT_SECONDS = 0.8

LEFT_LINE_THRESHOLD = 5
RIGHT_LINE_THRESHOLD = 5

INFO_PANEL_HEIGHT = 150
TEXT_FONT = cv2.FONT_HERSHEY_COMPLEX


@dataclass
class SafetyData:
    """Поля телеметрии, которые используются для аварийной остановки."""
    line_left: int = 1023
    line_right: int = 1023
    ir_cm: int = 60


@dataclass
class RobotState:
    """Состояние робота и миссии."""
    position: np.ndarray | None = None
    heading: np.ndarray | None = None
    target_index: int = 0
    path_points: list[np.ndarray] = field(default_factory=list)
    paused: bool = True
    mission_complete: bool = False
    state: str = "WAITING FOR MARKERS"
    is_turning: bool = False  # Флаг поворота


def parse_safety(line: str) -> SafetyData | None:
    """Из полной строки TEL извлекает датчики линии."""
    parts = line.split()
    if len(parts) != 12 or parts[0] != "TEL":
        return None

    try:
        return SafetyData(
            line_left=int(parts[8]),
            line_right=int(parts[9]),
            ir_cm=int(parts[10]),
        )
    except ValueError:
        return None


def send(connection: socket.socket, command: str) -> None:
    """Отправляет одну строковую команду роботу."""
    try:
        connection.sendall((command + "\n").encode("ascii"))
    except (BlockingIOError, OSError):
        pass


def receive_telemetry(
    connection: socket.socket,
    buffer: bytes,
    current: SafetyData,
) -> tuple[bytes, SafetyData, bool]:
    """Читает все доступные байты и сохраняет неполную строку в buffer."""
    received = False

    while True:
        try:
            packet = connection.recv(2048)
        except BlockingIOError:
            break

        if not packet:
            raise ConnectionError("TCP-соединение закрыто")
        buffer += packet

    while b"\n" in buffer:
        raw_line, buffer = buffer.split(b"\n", 1)
        parsed = parse_safety(
            raw_line.decode("ascii", errors="replace").strip()
        )
        if parsed is not None:
            current = parsed
            received = True

    return buffer, current, received


def marker_geometry(corners: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Возвращает центр маркера и единичный вектор его стороны P0–P1."""
    points = corners.reshape(4, 2)
    center = points.mean(axis=0)

    front = 0.5 * (points[0] + points[1])
    heading = front - center
    heading /= max(float(np.linalg.norm(heading)), 1.0)

    return center, heading


def signed_angle(heading: np.ndarray, vector: np.ndarray) -> float:
    """Возвращает угол от heading к vector в диапазоне [-pi, pi]."""
    hx, hy = float(heading[0]), -float(heading[1])
    vx, vy = float(vector[0]), -float(vector[1])

    cross = hx * vy - hy * vx
    dot = hx * vx + hy * vy
    return math.atan2(cross, dot)


def get_corner_points(
    markers: dict[int, np.ndarray],
    corner_ids: tuple[int, ...]
) -> list[np.ndarray] | None:
    """Получает центры угловых маркеров в порядке ID."""
    points = []
    for marker_id in corner_ids:
        if marker_id not in markers:
            return None
        center, _ = marker_geometry(markers[marker_id])
        points.append(center)
    return points


def get_rectangle_bounds(points: list[np.ndarray]) -> tuple[float, float, float, float]:
    """Возвращает минимальные и максимальные координаты прямоугольника."""
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    return min(x_coords), max(x_coords), min(y_coords), max(y_coords)


def generate_zigzag_path(
    bounds: tuple[float, float, float, float],
    step_px: float = 60,
    margin_px: float = 25
) -> list[np.ndarray]:
    """Генерирует зигзагообразный путь для покрытия всей прямоугольной области."""
    x_min, x_max, y_min, y_max = bounds
    x_min += margin_px
    x_max -= margin_px
    y_min += margin_px
    y_max -= margin_px

    path = []
    
    width = x_max - x_min
    height = y_max - y_min
    
    if width >= height:
        y = y_min
        direction = 1
        
        while y <= y_max:
            if direction == 1:
                path.append(np.array([x_min, y], dtype=np.float32))
                path.append(np.array([x_max, y], dtype=np.float32))
            else:
                path.append(np.array([x_max, y], dtype=np.float32))
                path.append(np.array([x_min, y], dtype=np.float32))
            
            y += step_px
            direction *= -1
            
            if y <= y_max:
                if direction == 1:
                    path.append(np.array([x_min, y], dtype=np.float32))
                else:
                    path.append(np.array([x_max, y], dtype=np.float32))
    else:
        x = x_min
        direction = 1
        
        while x <= x_max:
            if direction == 1:
                path.append(np.array([x, y_min], dtype=np.float32))
                path.append(np.array([x, y_max], dtype=np.float32))
            else:
                path.append(np.array([x, y_max], dtype=np.float32))
                path.append(np.array([x, y_min], dtype=np.float32))
            
            x += step_px
            direction *= -1
            
            if x <= x_max:
                if direction == 1:
                    path.append(np.array([x, y_min], dtype=np.float32))
                else:
                    path.append(np.array([x, y_max], dtype=np.float32))

    return path


def draw_safety_info(
    frame: np.ndarray,
    safety: SafetyData,
    command: str,
    state: str,
    paused: bool,
    points_complete: int,
    total_points: int
) -> None:
    """Рисует информационную панель на кадре."""
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (frame.shape[1], 90), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

    status_color = (80, 230, 80) if command != "STOP" else (80, 120, 255)
    pause_text = "PAUSED" if paused else "RUNNING"

    cv2.putText(
        frame,
        f"STATE: {state}  {pause_text}",
        (18, 32),
        TEXT_FONT,
        0.65,
        status_color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"COMMAND: {command}  WAYPOINTS: {points_complete}/{total_points}",
        (18, 58),
        TEXT_FONT,
        0.50,
        (240, 240, 240),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"SENSORS: L={safety.line_left} R={safety.line_right}",
        (18, 80),
        TEXT_FONT,
        0.50,
        (240, 240, 240),
        1,
        cv2.LINE_AA,
    )


def main() -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )
    detector = cv2.aruco.ArucoDetector(
        dictionary,
        cv2.aruco.DetectorParameters(),
    )

    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("Камера не открылась")

    connection = socket.create_connection(ROBOT_ADDRESS, timeout=3.0)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.setblocking(False)

    telemetry_buffer = b""
    safety = SafetyData()
    last_telemetry_time = 0.0
    previous_send = 0.0

    state = RobotState()
    corner_points: list[np.ndarray] | None = None
    bounds: tuple[float, float, float, float] | None = None
    zigzag_path: list[np.ndarray] = []
    waypoint_index: int = 0
    
    # Переменные для управления поворотом
    is_turning = False
    turn_start_time = 0
    turn_error = 0
    turn_angular_speed = 0

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Не удалось получить кадр камеры")

            telemetry_buffer, safety, telemetry_received = receive_telemetry(
                connection,
                telemetry_buffer,
                safety,
            )
            if telemetry_received:
                last_telemetry_time = time.monotonic()

            # Обнаружение ArUco маркеров
            corners, ids, _rejected = detector.detectMarkers(frame)
            markers: dict[int, np.ndarray] = {}

            if ids is not None:
                markers = {
                    int(marker_id): marker_corners
                    for marker_corners, marker_id in zip(
                        corners,
                        ids.flatten(),
                    )
                }
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Проверяем наличие угловых маркеров
            if corner_points is None:
                corner_points = get_corner_points(markers, CORNER_MARKER_IDS)
                if corner_points is not None:
                    bounds = get_rectangle_bounds(corner_points)
                    zigzag_path = generate_zigzag_path(bounds)
                    state.path_points = zigzag_path
                    state.state = "RECTANGLE DETECTED"

                    pts = np.array(corner_points, dtype=np.int32)
                    cv2.polylines(frame, [pts], True, (0, 255, 255), 3)

                    state.state = "PRESS SPACE TO START"
                    state.paused = True

                else:
                    state.state = "WAITING FOR CORNER MARKERS"
            else:
                pts = np.array(corner_points, dtype=np.int32)
                cv2.polylines(frame, [pts], True, (0, 255, 255), 2)

            # Рисуем маршрут
            if len(zigzag_path) > 1:
                path_points = np.array(zigzag_path, dtype=np.int32)
                cv2.polylines(frame, [path_points], False, (255, 150, 0), 2)

                for i in range(waypoint_index):
                    pt = tuple(np.rint(zigzag_path[i]).astype(int))
                    cv2.circle(frame, pt, 5, (100, 255, 100), -1)

            # Управление роботом
            command = "STOP"
            current_target = None

            telemetry_stale = (
                time.monotonic() - last_telemetry_time
                > TELEMETRY_TIMEOUT_SECONDS
            )
            black_border = (
                safety.line_left < LEFT_LINE_THRESHOLD
                or safety.line_right < RIGHT_LINE_THRESHOLD
            )

            if telemetry_stale:
                state.state = "WAITING FOR TELEMETRY"
            elif black_border:
                state.state = "BLACK BORDER: STOP"
                state.paused = True
            elif state.paused:
                pass
            elif ROBOT_MARKER_ID not in markers:
                state.state = f"WAITING FOR ROBOT MARKER {ROBOT_MARKER_ID}"
            elif corner_points is None:
                state.state = "WAITING FOR CORNER MARKERS"
            elif waypoint_index >= len(zigzag_path):
                state.state = "MISSION COMPLETE"
                state.mission_complete = True
                state.paused = True
            else:
                robot_pos, robot_heading = marker_geometry(
                    markers[ROBOT_MARKER_ID]
                )
                state.position = robot_pos
                state.heading = robot_heading

                target = zigzag_path[waypoint_index]
                current_target = target

                if current_target is not None:
                    vector = current_target - robot_pos
                    distance = float(np.linalg.norm(vector))
                    error = signed_angle(robot_heading, vector)

                    robot_pixel = tuple(np.rint(robot_pos).astype(int))
                    target_pixel = tuple(np.rint(current_target).astype(int))
                    cv2.arrowedLine(frame, robot_pixel, target_pixel, (0, 170, 255), 3)

                    # Обработка поворота
                    if distance <= TARGET_TOLERANCE_PX:
                        # Достигли цели
                        waypoint_index += 1
                        is_turning = False
                        state.state = f"WAYPOINT {waypoint_index}/{len(zigzag_path)}"
                        
                    elif abs(error) > ANGLE_TOLERANCE_RAD:
                        # Включить режим поворота вокруг оси
                        if not is_turning:
                            # Начать поворот
                            is_turning = True
                            turn_start_time = time.monotonic()
                            turn_error = error
                            turn_angular_speed = ANGULAR_SPEED_MRAD_S if error > 0 else -ANGULAR_SPEED_MRAD_S
                            state.state = f"TURN IN PLACE {math.degrees(error):.1f}DEG"
                        
                        # Поворот на месте (скорость вперёд = 0, только угловая)
                        angular = turn_angular_speed
                        
                        # Проверка, довернули ли мы до нужного угла
                        current_error = signed_angle(robot_heading, vector)
                        
                        # Если довернулись или повернулись слишком далеко
                        if abs(current_error) <= ANGLE_TOLERANCE_RAD or abs(current_error) > abs(turn_error) + 10:
                            # Завершаем поворот
                            is_turning = False
                            command = "VEL 0 0"  # Остановка
                            state.state = f"TURN COMPLETE"
                        else:
                            # Продолжаем поворот на месте
                            command = f"VEL 0 {int(angular)}"
                            state.state = f"TURN IN PLACE {math.degrees(current_error):.1f}DEG"
                        
                        # Если поворот затянулся, сбросить
                        if time.monotonic() - turn_start_time > 5.0:
                            is_turning = False
                            state.state = "TURN TIMEOUT"
                            
                    else:
                        # Движение вперёд
                        is_turning = False
                        command = f"VEL {LINEAR_SPEED_MM_S} 0"
                        state.state = f"DRIVE {distance:.0f}PX"
                else:
                    command = "VEL 0 500"

            # Отправка команды
            now = time.monotonic()
            if now - previous_send >= SEND_PERIOD_SECONDS:
                send(connection, command)
                previous_send = now

            # Отображение информации
            draw_safety_info(
                frame,
                safety,
                command,
                state.state,
                state.paused,
                waypoint_index,
                len(zigzag_path)
            )

            cv2.imshow("Rectangle painting", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break

            if key == ord(" "):
                if state.paused:
                    if corner_points is not None:
                        state.paused = False
                        state.state = "MISSION STARTED"
                        is_turning = False
                    else:
                        state.state = "NO RECTANGLE DETECTED"
                else:
                    state.paused = True
                    state.state = "PAUSED"
                    send(connection, "STOP")
                    is_turning = False

    finally:
        send(connection, "STOP")
        connection.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
