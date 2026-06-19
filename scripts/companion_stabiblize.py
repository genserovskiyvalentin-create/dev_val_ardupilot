#!/usr/bin/env python3
"""
COMPANION SCRIPT - единый скрипт управления дроном-перехватчиком

ЭТАП 1: PD-РЕГУЛЯТОР НАКЛОНА + ПОЛНЫЙ ГАЗ (через STABILIZE + RC OVERRIDE)
- Кадр камеры = плоскость управления
- err_x (горизонталь) → Roll дрона (PD-регулятор → RC канал 1)
- err_y (вертикаль) → Pitch дрона (PD-регулятор → RC канал 2)
- Thrust (газ) = 100% через RC канал 3 = 2000
- РЕЖИМ: STABILIZE (ArduPilot думает, что управляют с пульта)

PD-РЕГУЛЯТОР:
- P (пропорциональная): наклоняет дрон к цели
- D (дифференциальная): ТОРМОЗИТ наклон (демпфирование)
  ⚠️ ВАЖНО: D имеет ПРОТИВОПОЛОЖНЫЙ знак, чтобы демпфировать!
"""

import cv2
import numpy as np
import subprocess
import time
import pygame
import math
from pymavlink import mavutil

# ============================================================================
# НАСТРОЙКИ
# ============================================================================

# === ГЕОМЕТРИЯ КАМЕРЫ ===
CAMERA_ANGLE = 90  # 90° = камера вверх (зенит)

# === ВИДЕО ===
WIDTH, HEIGHT = 640, 480

# === PD-РЕГУЛЯТОР НАКЛОНА ===
MAX_ANGLE_DEG = 30         # Макс. угол наклона
MAX_ANGLE_RAD = math.radians(MAX_ANGLE_DEG)  # 0.524 рад

# --- P (ПРОПОРЦИОНАЛЬНАЯ) ---
# Наклоняет дрон к цели.
KP_ROLL = 0.003            # P для roll
KP_PITCH = 0.003           # P для pitch

# --- D (ДИФФЕРЕНЦИАЛЬНАЯ) ---
# ТОРМОЗИТ наклон (демпфирование).
# ⚠️ Знак МИНУС в формуле — это критически важно!
KD_ROLL = 0.5            # D для roll (начни с малого)
KD_PITCH = 0.5           # D для pitch

# --- ОГРАНИЧЕНИЕ D-СОСТАВЛЯЮЩЕЙ ---
# Чтобы D не давала слишком больших всплесков при резких движениях
MAX_D_ROLL = 0.1           # Макс. вклад D для roll (~5.7°)
MAX_D_PITCH = 0.1          # Макс. вклад D для pitch (~5.7°)

# --- МЁРТВАЯ ЗОНА ---
ROLL_DEADZONE = 15         # Мёртвая зона по X (roll)
PITCH_DEADZONE = 15        # Мёртвая зона по Y (pitch)

# === ДЖОЙСТИК ===
DEADZONE = 0.05
SWITCH_THRESHOLD = 0.5
INVERT_PITCH = True

# === ЗАЩИТА ОТ ПОТЕРИ ЦЕЛИ ===
TARGET_LOST_FRAMES = 10

# === СГЛАЖИВАНИЕ ===
ALTITUDE_HISTORY_SIZE = 10
SPEED_HISTORY_SIZE = 5

# ============================================================================
# БАЗОВЫЕ ТОЧКИ
# ============================================================================
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

# ============================================================================
# ФУНКЦИИ ПОДКЛЮЧЕНИЯ И ИНИЦИАЛИЗАЦИИ
# ============================================================================

def connect_mavlink():
    print("🔌 Подключение к ArduPilot SITL (порт 14550)...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    print(f"✅ Связь установлена (система {master.target_system})")
    return master


def init_joystick():
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        print("❌ Джойстик не найден! Запусти virtual_gamepad.py")
        return None
    
    for i in range(count):
        joy = pygame.joystick.Joystick(i)
        joy.init()
        name = joy.get_name().lower()
        print(f"   [{i}] {joy.get_name()}")
        if 'uinput' in name or 'virtual' in name:
            print(f"✅ Выбран виртуальный геймпад: {joy.get_name()}")
            return joy
    
    if count > 1:
        joy = pygame.joystick.Joystick(1)
        joy.init()
        return joy
    
    joy = pygame.joystick.Joystick(0)
    joy.init()
    return joy


# ============================================================================
# ФУНКЦИИ ЧТЕНИЯ ДАННЫХ С ДРОНА
# ============================================================================

def get_barometric_altitude(master, history):
    """Читает высоту с барометра через VFR_HUD."""
    last_alt = None
    while True:
        msg = master.recv_match(type='VFR_HUD', blocking=False)
        if msg is None:
            break
        last_alt = msg.alt
    
    if last_alt is None:
        if history:
            return sum(history) / len(history)
        return None
    
    if last_alt < -1.0 or last_alt > 20.0:
        if history:
            return sum(history) / len(history)
        return None
    
    history.append(last_alt)
    if len(history) > ALTITUDE_HISTORY_SIZE:
        history.pop(0)
    
    return sum(history) / len(history)


def get_drone_speed(master, history):
    """Читает горизонтальную скорость через LOCAL_POSITION_NED."""
    last_speed = None
    while True:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg is None:
            break
        vx = msg.vx
        vy = msg.vy
        last_speed = math.sqrt(vx**2 + vy**2)
    
    if last_speed is None:
        if history:
            return sum(history) / len(history)
        return None
    
    history.append(last_speed)
    if len(history) > SPEED_HISTORY_SIZE:
        history.pop(0)
    
    return sum(history) / len(history)


# ============================================================================
# ФУНКЦИИ УПРАВЛЕНИЯ ДРОНОМ
# ============================================================================

def send_rc_override(master, ch1, ch2, ch3, ch4, ch5):
    """
    Отправка RC_CHANNELS_OVERRIDE.
    
    ArduPilot в STABILIZE режиме:
    - CH1 (roll)  = 1000-2000 → наклон влево/вправо
    - CH2 (pitch) = 1000-2000 → наклон вперёд/назад
    - CH3 (throttle) = 1000-2000 → газ 0-100%
    - CH4 (yaw)   = 1000-2000 → поворот влево/вправо
    - CH5 (mode)  = 1000-2000 → режим полёта
    """
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        ch1, ch2, ch3, ch4, ch5, 0, 0, 0
    )


def force_disarm(master):
    """Принудительный дизарм моторов."""
    print("💥 ПРИНУДИТЕЛЬНЫЙ ДИЗАРМ!")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0
    )


# ============================================================================
# ФУНКЦИИ ДЕТЕКЦИИ ЦЕЛИ
# ============================================================================

def detect_target(frame):
    """Детектирует красную цель в кадре через HSV-фильтр."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask1, mask2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 20:
            x, y, w, h = cv2.boundingRect(c)
            cx, cy = x + w // 2, y + h // 2
            return cx, cy, w * h, (x, y, w, h)
    return None


# ============================================================================
# ФУНКЦИИ ЧТЕНИЯ ДЖОЙСТИКА
# ============================================================================

def read_joystick(joy):
    """Читает оси джойстика и конвертирует в RC значения."""
    pygame.event.pump()
    
    roll_raw = joy.get_axis(3)
    pitch_raw = joy.get_axis(2)
    throttle_raw = joy.get_axis(1)
    yaw_raw = joy.get_axis(0)
    
    def apply_dz(v):
        return 0.0 if abs(v) < DEADZONE else v
    
    roll = apply_dz(roll_raw)
    pitch = apply_dz(pitch_raw)
    throttle = apply_dz(throttle_raw)
    yaw = apply_dz(yaw_raw)
    
    if INVERT_PITCH:
        pitch = -pitch
    
    def axis_to_rc(v):
        return max(1000, min(2000, int(v * 500 + 1500)))
    
    ch1 = axis_to_rc(roll)
    ch2 = axis_to_rc(pitch)
    ch3 = axis_to_rc(throttle)
    ch4 = axis_to_rc(yaw)
    
    switch_on = False
    ch5 = 1500
    if joy.get_numaxes() > 4:
        switch_raw = joy.get_axis(4)
        ch5 = axis_to_rc(switch_raw)
        switch_on = switch_raw > SWITCH_THRESHOLD
    
    return ch1, ch2, ch3, ch4, ch5, switch_on


# ============================================================================
# ОСНОВНАЯ ФУНКЦИЯ
# ============================================================================

def main():
    master = connect_mavlink()
    joy = init_joystick()
    if joy is None:
        return
    
    print("📹 Включение стриминга камеры в Gazebo...")
    subprocess.run([
        "gz", "topic",
        "-t", "/world/iris_runway/model/iris_with_gimbal/link/camera_link/sensor/camera/image/enable_streaming",
        "-m", "gz.msgs.Boolean",
        "-p", "data: 1"
    ], check=False)
    time.sleep(1)
    
    print(f"🎥 Запуск видеопотока ({WIDTH}x{HEIGHT})...")
    gst_cmd = [
        "gst-launch-1.0", "-q",
        "udpsrc", "port=5600", "!",
        "application/x-rtp, encoding-name=H264, payload=96", "!",
        "rtpjitterbuffer", "!", "rtph264depay", "!", "avdec_h264", "!",
        "videoconvert", "!",
        f"video/x-raw, format=BGR, width={WIDTH}, height={HEIGHT}", "!",
        "fdsink", "fd=1"
    ]
    proc = subprocess.Popen(gst_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=10**8)
    frame_size = WIDTH * HEIGHT * 3
    
    # === СОСТОЯНИЯ ===
    last_switch_on = False
    auto_active = False
    target_lost_counter = 0
    altitude_history = []
    speed_history = []
    
    # ========================================================================
    # 🎯 PD-РЕГУЛЯТОР: СОСТОЯНИЕ (ПРЕДЫДУЩИЕ ОШИБКИ ДЛЯ D-СОСТАВЛЯЮЩЕЙ)
    # ========================================================================
    # last_err_x     — предыдущая ошибка по roll (пиксели)
    # last_err_pitch — предыдущая "виртуальная" ошибка по pitch (инвертированная)
    #
    # Для pitch мы работаем с err_pitch = -err_y, чтобы упростить логику.
    # ========================================================================
    last_err_x = 0.0
    last_err_pitch = 0.0
    
    # Для измерения dt (времени между кадрами)
    last_frame_time = time.time()
    
    print("=" * 60)
    print("🤖 COMPANION СКРИПТ — ЭТАП 1: PD-РЕГУЛЯТОР + 100% ГАЗ")
    print("=" * 60)
    print(f"   📐 MAX_ANGLE = {MAX_ANGLE_DEG}° ({MAX_ANGLE_RAD:.3f} рад)")
    print(f"   🎯 KP_ROLL = {KP_ROLL} рад/пиксель (P для roll)")
    print(f"   🎯 KP_PITCH = {KP_PITCH} рад/пиксель (P для pitch)")
    print(f"   🎯 KD_ROLL = {KD_ROLL} рад*с/пиксель (D для roll)")
    print(f"   🎯 KD_PITCH = {KD_PITCH} рад*с/пиксель (D для pitch)")
    print(f"   🎯 MAX_D_ROLL = {MAX_D_ROLL} рад (ограничение D)")
    print(f"   🎯 MAX_D_PITCH = {MAX_D_PITCH} рад (ограничение D)")
    print(f"   ⛽ CH3 = 2000 (100% газа через RC override)")
    print(f"   🎮 РЕЖИМ: STABILIZE (ArduPilot думает, что управляют с пульта)")
    print(f"   Тумблер ВЫКЛ → MANUAL (пилот управляет)")
    print(f"   Тумблер ВКЛ → AUTO (скрипт управляет через RC override)")
    print(f"   AUTO + цель потеряна → MANUAL")
    print(f"   'q' → выход")
    print("=" * 60)
    
    try:
        while True:
            # === ИЗМЕРЕНИЕ dt (время между кадрами) ===
            current_time = time.time()
            dt = current_time - last_frame_time
            last_frame_time = current_time
            
            # Ограничиваем dt, чтобы избежать "всплесков" при паузах
            dt = min(dt, 0.1)  # Максимум 100 мс
            # Защита от деления на ноль
            if dt < 0.001:
                dt = 0.001
            
            # === ЧИТАЕМ КАДР ===
            raw_frame = proc.stdout.read(frame_size)
            if len(raw_frame) != frame_size:
                continue
            frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()
            
            # === ЧИТАЕМ ДЖОЙСТИК ===
            ch1, ch2, ch3, ch4, ch5, switch_on = read_joystick(joy)
            
            # === ДЕТЕКТИРУЕМ ЦЕЛЬ ===
            target = detect_target(frame)
            
            # === ЧИТАЕМ ТЕЛЕМЕТРИЮ ===
            current_altitude = get_barometric_altitude(master, altitude_history)
            current_speed = get_drone_speed(master, speed_history)
            
            if current_altitude is None:
                current_altitude = 0.0
            if current_speed is None:
                current_speed = 0.0
            
            # === РИСУЕМ ПЕРЕКРЕСТИЕ ЦЕНТРА ===
            cv2.line(frame, (CENTER_X, 0), (CENTER_X, HEIGHT), (100, 100, 100), 1)
            cv2.line(frame, (0, CENTER_Y), (WIDTH, CENTER_Y), (100, 100, 100), 1)
            cv2.circle(frame, (CENTER_X, CENTER_Y), 5, (255, 255, 255), 1)
            
            # ====================================================================
            # ЛОГИКА ПЕРЕКЛЮЧЕНИЯ РЕЖИМОВ
            # ====================================================================
            
            # 1. Фронт тумблера (ВЫКЛ → ВКЛ) → AUTO
            if switch_on and not last_switch_on:
                # 🔒 СБРАСЫВАЕМ ПРЕДЫДУЩИЕ ОШИБКИ при включении AUTO
                last_err_x = 0.0
                last_err_pitch = 0.0
                
                print(f"\n🔘 ТУМБЛЕР ВКЛ → AUTO (STABILIZE + RC OVERRIDE)")
                print(f"   ⛽ CH3 = 2000 (100% газа)")
                print(f"   🎮 ArduPilot думает, что управляют с пульта")
                
                auto_active = True
                target_lost_counter = 0
            
            # 2. Тумблер выключен → MANUAL
            if not switch_on and auto_active:
                print("\n🔘 ТУМБЛЕР ВЫКЛ → ВОЗВРАТ В MANUAL")
                auto_active = False
                target_lost_counter = 0
                last_err_x = 0.0
                last_err_pitch = 0.0
            
            # 3. AUTO + цель потеряна → MANUAL
            if auto_active and target is None:
                target_lost_counter += 1
                if target_lost_counter >= TARGET_LOST_FRAMES:
                    print(f"\n🎯 ЦЕЛЬ ПОТЕРЯНА → ПЕРЕХОД В MANUAL")
                    auto_active = False
                    target_lost_counter = 0
                    last_err_x = 0.0
                    last_err_pitch = 0.0
            elif auto_active and target is not None:
                target_lost_counter = 0
            
            last_switch_on = switch_on
            
            # ====================================================================
            # 🎯 ЭТАП 1: PD-РЕГУЛЯТОР + 100% ГАЗ (AUTO)
            # ====================================================================
            
            if auto_active:
                if target is not None:
                    cx, cy, area, (x, y, w, h) = target
                    
                    # Рисуем цель
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.line(frame, (CENTER_X, CENTER_Y), (cx, cy), (0, 255, 0), 2)
                    
                    # ============================================================
                    # ШАГ 1: Вычисление ошибок в пикселях
                    # ============================================================
                    err_x = cx - CENTER_X
                    err_y = cy - CENTER_Y
                    
                    # ============================================================
                    # ШАГ 2: Мёртвая зона (избегаем дрожания в центре)
                    # ============================================================
                    if abs(err_x) < ROLL_DEADZONE:
                        err_x = 0.0
                    if abs(err_y) < PITCH_DEADZONE:
                        err_y = 0.0
                    
                    # ============================================================
                    # ШАГ 3: "Виртуальная" ошибка pitch (инвертированная)
                    # ============================================================
                    # Камера смотрит ВВЕРХ (в зенит).
                    # err_y > 0 → цель ВНИЗУ кадра → это "впереди" дрона
                    # Нужно наклонить нос ВПЕРЁД (вниз) → CH2 < 1500
                    #
                    # Вводим err_pitch = -err_y:
                    #   err_y > 0 → err_pitch < 0 → rc_pitch < 1500 (нос вниз) ✓
                    #   err_y < 0 → err_pitch > 0 → rc_pitch > 1500 (нос вверх) ✓
                    #
                    # Теперь работаем с err_pitch как с обычной ошибкой.
                    # ============================================================
                    err_pitch = -err_y
                    
                    # ============================================================
                    # ШАГ 4: PD-РЕГУЛЯТОР для ROLL
                    # ============================================================
                    # Формула PD:
                    #   output = Kp * error + Kd * d(error)/dt
                    #
                    # ⚠️ ВАЖНО: D-составляющая имеет знак МИНУС!
                    # Это нужно для демпфирования (торможения наклона).
                    #
                    # P-составляющая:
                    #   p_roll = KP_ROLL * err_x
                    #   - err_x > 0 (цель справа) → p_roll > 0 → rc_roll > 1500 (наклон вправо) ✓
                    #
                    # D-составляющая:
                    #   d_err_x = (err_x - last_err_x) / dt
                    #   d_roll = -KD_ROLL * d_err_x  ← МИНУС!
                    #   - Если ошибка растёт (d_err_x > 0) → d_roll < 0 → тормозит наклон ✓
                    #   - Если ошибка уменьшается (d_err_x < 0) → d_roll > 0 → помогает наклону ✓
                    # ============================================================
                    
                    # P-составляющая для roll
                    p_roll = KP_ROLL * err_x
                    
                    # D-составляющая для roll (с минусом для демпфирования!)
                    d_err_x = (err_x - last_err_x) / dt
                    d_roll_raw = -KD_ROLL * d_err_x  # ← МИНУС!
                    d_roll = np.clip(d_roll_raw, -MAX_D_ROLL, MAX_D_ROLL)
                    
                    # Итоговый roll
                    desired_roll = p_roll + d_roll
                    desired_roll = np.clip(desired_roll, -MAX_ANGLE_RAD, MAX_ANGLE_RAD)
                    
                    # ============================================================
                    # ШАГ 5: PD-РЕГУЛЯТОР для PITCH
                    # ============================================================
                    # Работаем с err_pitch (уже инвертированной).
                    #
                    # P-составляющая:
                    #   p_pitch = KP_PITCH * err_pitch
                    #   - err_pitch < 0 (цель впереди) → p_pitch < 0 → rc_pitch < 1500 (нос вниз) ✓
                    #
                    # D-составляющая:
                    #   d_err_pitch = (err_pitch - last_err_pitch) / dt
                    #   d_pitch = -KD_PITCH * d_err_pitch  ← МИНУС!
                    #   - Если ошибка растёт (d_err_pitch < 0, т.к. err_pitch становится более отрицательным)
                    #     → d_pitch > 0 → тормозит наклон вниз ✓
                    # ============================================================
                    
                    # P-составляющая для pitch
                    p_pitch = KP_PITCH * err_pitch
                    
                    # D-составляющая для pitch (с минусом для демпфирования!)
                    d_err_pitch = (err_pitch - last_err_pitch) / dt
                    d_pitch_raw = -KD_PITCH * d_err_pitch  # ← МИНУС!
                    d_pitch = np.clip(d_pitch_raw, -MAX_D_PITCH, MAX_D_PITCH)
                    
                    # Итоговый pitch
                    desired_pitch = p_pitch + d_pitch
                    desired_pitch = np.clip(desired_pitch, -MAX_ANGLE_RAD, MAX_ANGLE_RAD)
                    
                    # ============================================================
                    # ШАГ 6: СОХРАНЕНИЕ ТЕКУЩИХ ОШИБОК ДЛЯ СЛЕДУЮЩЕГО КАДРА
                    # ============================================================
                    last_err_x = err_x
                    last_err_pitch = err_pitch  # Сохраняем инвертированную ошибку
                    
                    # ================================================================
                    # ШАГ 7: ПРЕОБРАЗОВАНИЕ УГЛОВ В RC ЗНАЧЕНИЯ
                    # ================================================================
                    # Формула:
                    #   RC = 1500 + (desired_angle / MAX_ANGLE) * 500
                    #
                    # Для roll:
                    #   desired_roll > 0 → rc_roll > 1500 (наклон вправо)
                    #   desired_roll < 0 → rc_roll < 1500 (наклон влево)
                    #
                    # Для pitch:
                    #   desired_pitch > 0 → rc_pitch > 1500 (нос вверх)
                    #   desired_pitch < 0 → rc_pitch < 1500 (нос вниз)
                    # ================================================================
                    
                    rc_roll = int(1500 + (desired_roll / MAX_ANGLE_RAD) * 500)
                    rc_pitch = int(1500 + (desired_pitch / MAX_ANGLE_RAD) * 500)
                    
                    # Ограничение RC значений
                    rc_roll = max(1000, min(2000, rc_roll))
                    rc_pitch = max(1000, min(2000, rc_pitch))
                    
                    # ================================================================
                    # ШАГ 8: ОТПРАВКА RC OVERRIDE (100% ГАЗ!)
                    # ================================================================
                    send_rc_override(master, rc_roll, rc_pitch, 2000, 1500, 1500)
                    
                    # Приводим к int для корректного форматирования в OSD
                    err_x_int = int(err_x)
                    err_y_int = int(err_y)
                    
                    # === OSD ===
                    cv2.putText(frame, f"🎯 AUTO (STABILIZE) | RC: {rc_roll}/{rc_pitch}/2000/1500", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                    cv2.putText(frame, f"err_x: {err_x_int:+4d} | roll: {math.degrees(desired_roll):+.1f}° (P:{math.degrees(p_roll):+.2f} D:{math.degrees(d_roll):+.2f})", 
                                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                    cv2.putText(frame, f"err_y: {err_y_int:+4d} | pitch: {math.degrees(desired_pitch):+.1f}° (P:{math.degrees(p_pitch):+.2f} D:{math.degrees(d_pitch):+.2f})", 
                                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                    cv2.putText(frame, f"throttle: 2000 (100% газа)", 
                                (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    # Статус
                    if err_x == 0 and err_y == 0:
                        status = "CENTERED"
                    else:
                        status = "TRACKING"
                    
                    cv2.putText(frame, f"⚡ {status}", (10, 135),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                else:
                    # Цель потеряна — нейтральные стики, но газ 100%
                    send_rc_override(master, 1500, 1500, 2000, 1500, 1500)
                    
                    cv2.putText(frame, f"🎯 AUTO | TARGET LOST", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.putText(frame, f"throttle: 2000 (100% газа)", 
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # ====================================================================
            # ЛОГИКА MANUAL
            # ====================================================================
            
            else:
                send_rc_override(master, ch1, ch2, ch3, ch4, ch5)
                
                if not auto_active:
                    cv2.putText(frame, "🕹 MANUAL MODE (STABILIZE)", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                    cv2.putText(frame, f"CH: {ch1}/{ch2}/{ch3}/{ch4}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    if target is not None:
                        x, y, w, h = target[3]
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                        cv2.putText(frame, "Target visible - toggle ON for AUTO", (10, 90),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            
            # === ПОКАЗ КАДРА ===
            cv2.imshow(f"Companion (Stage 1: PD Controller + Full Throttle)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🛑 Остановка...")
        send_rc_override(master, 0, 0, 0, 0, 0)
        proc.terminate()
        cv2.destroyAllWindows()
        print("✅ Скрипт завершен.")


if __name__ == "__main__":
    main()