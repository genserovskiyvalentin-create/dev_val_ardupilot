#!/usr/bin/env python3
"""
COMPANION SCRIPT - ПРОСТОЙ P-ТРЕКЕР

Логика:
- Кадр камеры = плоскость управления
- err_x (горизонталь) → Roll дрона (P-регулятор)
- err_y (вертикаль) → Pitch дрона (P-регулятор)
- Газ = 100% всегда в AUTO
- Режим: STABILIZE (ArduPilot думает, что управляют с пульта)

Параметры:
- KP: насколько сильно наклоняться к цели
- MAX_ANGLE_DEG: максимальный угол наклона
- DEADZONE: мёртвая зона в центре (пиксели)
- THRUST_RC: значение газа (2000 = 100%)
"""

import cv2
import numpy as np
import subprocess
import time
import pygame
import math
from pymavlink import mavutil

# ============================================================================
# ПАРАМЕТРЫ (всего 4 штуки!)
# ============================================================================

# --- P-регулятор ---
KP = 0.008                 # Насколько сильно наклоняться к цели (рад/пиксель)

# --- Максимальный угол ---
MAX_ANGLE_DEG = 30         # Макс. угол наклона (градусы)
MAX_ANGLE_RAD = math.radians(MAX_ANGLE_DEG)

# --- Мёртвая зона ---
DEADZONE = 15              # Если ошибка меньше этого — считаем нулём (пиксели)

# --- Газ ---
THRUST_RC = 2000           # Значение газа в RC (2000 = 100%)

# ============================================================================
# ВИДЕО
# ============================================================================
WIDTH, HEIGHT = 640, 480
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

# ============================================================================
# ДЖОЙСТИК
# ============================================================================
JOY_DEADZONE = 0.05
SWITCH_THRESHOLD = 0.5
INVERT_PITCH = True

# ============================================================================
# ЗАЩИТА ОТ ПОТЕРИ ЦЕЛИ
# ============================================================================
TARGET_LOST_FRAMES = 10

# ============================================================================
# ФУНКЦИИ
# ============================================================================

def connect_mavlink():
    print("🔌 Подключение к ArduPilot SITL (порт 14550)...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    print(f"✅ Связь установлена")
    return master


def init_joystick():
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        print("❌ Джойстик не найден!")
        return None
    for i in range(count):
        joy = pygame.joystick.Joystick(i)
        joy.init()
        name = joy.get_name().lower()
        print(f"   [{i}] {joy.get_name()}")
        if 'uinput' in name or 'virtual' in name:
            return joy
    joy = pygame.joystick.Joystick(0)
    joy.init()
    return joy


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


def read_joystick(joy):
    """Читает оси джойстика и конвертирует в RC значения."""
    pygame.event.pump()
    
    def apply_dz(v):
        return 0.0 if abs(v) < JOY_DEADZONE else v
    
    roll = apply_dz(joy.get_axis(3))
    pitch = apply_dz(joy.get_axis(2))
    throttle = apply_dz(joy.get_axis(1))
    yaw = apply_dz(joy.get_axis(0))
    
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


def send_rc_override(master, ch1, ch2, ch3, ch4, ch5):
    """Отправка RC_CHANNELS_OVERRIDE."""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        ch1, ch2, ch3, ch4, ch5, 0, 0, 0
    )


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
    
    print("=" * 60)
    print("🤖 ПРОСТОЙ P-ТРЕКЕР")
    print("=" * 60)
    print(f"   KP = {KP} рад/пиксель")
    print(f"   MAX_ANGLE = {MAX_ANGLE_DEG}°")
    print(f"   DEADZONE = {DEADZONE} px")
    print(f"   THRUST = {THRUST_RC} (100%)")
    print(f"   Тумблер ВЫКЛ → MANUAL")
    print(f"   Тумблер ВКЛ → AUTO")
    print(f"   'q' → выход")
    print("=" * 60)
    
    try:
        while True:
            # === ЧИТАЕМ КАДР ===
            raw_frame = proc.stdout.read(frame_size)
            if len(raw_frame) != frame_size:
                continue
            frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()
            
            # === ЧИТАЕМ ДЖОЙСТИК ===
            ch1, ch2, ch3, ch4, ch5, switch_on = read_joystick(joy)
            
            # === ДЕТЕКТИРУЕМ ЦЕЛЬ ===
            target = detect_target(frame)
            
            # === РИСУЕМ ПЕРЕКРЕСТИЕ ===
            cv2.line(frame, (CENTER_X, 0), (CENTER_X, HEIGHT), (100, 100, 100), 1)
            cv2.line(frame, (0, CENTER_Y), (WIDTH, CENTER_Y), (100, 100, 100), 1)
            
            # ====================================================================
            # ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ
            # ====================================================================
            
            # Тумблер ВКЛ → AUTO
            if switch_on and not last_switch_on:
                print(f"\n🔘 ТУМБЛЕР ВКЛ → AUTO")
                auto_active = True
                target_lost_counter = 0
            
            # Тумблер ВЫКЛ → MANUAL
            if not switch_on and auto_active:
                print("\n🔘 ТУМБЛЕР ВЫКЛ → MANUAL")
                auto_active = False
                target_lost_counter = 0
            
            # Цель потеряна → MANUAL
            if auto_active and target is None:
                target_lost_counter += 1
                if target_lost_counter >= TARGET_LOST_FRAMES:
                    print(f"\n🎯 ЦЕЛЬ ПОТЕРЯНА → MANUAL")
                    auto_active = False
                    target_lost_counter = 0
            elif auto_active and target is not None:
                target_lost_counter = 0
            
            last_switch_on = switch_on
            
            # ====================================================================
            # 🎯 ТРЕКИНГ (AUTO) — всего 10 строк!
            # ====================================================================
            
            if auto_active:
                if target is not None:
                    cx, cy, area, (x, y, w, h) = target
                    
                    # Рисуем цель
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # === ПРОСТОЙ P-РЕГУЛЯТОР ===
                    # 1. Ошибки
                    err_x = cx - CENTER_X
                    err_y = cy - CENTER_Y
                    
                    # 2. Мёртвая зона
                    if abs(err_x) < DEADZONE:
                        err_x = 0.0
                    if abs(err_y) < DEADZONE:
                        err_y = 0.0
                    
                    # 3. Желаемые углы (P-регулятор)
                    #    Для pitch инвертируем: цель внизу → нос вниз
                    desired_roll = KP * err_x
                    desired_pitch = KP * (-err_y)
                    
                    # 4. Ограничение по MAX_ANGLE
                    desired_roll = np.clip(desired_roll, -MAX_ANGLE_RAD, MAX_ANGLE_RAD)
                    desired_pitch = np.clip(desired_pitch, -MAX_ANGLE_RAD, MAX_ANGLE_RAD)
                    
                    # 5. Преобразование в RC
                    rc_roll = int(1500 + (desired_roll / MAX_ANGLE_RAD) * 500)
                    rc_pitch = int(1500 + (desired_pitch / MAX_ANGLE_RAD) * 500)
                    rc_roll = max(1000, min(2000, rc_roll))
                    rc_pitch = max(1000, min(2000, rc_pitch))
                    
                    # 6. Отправка
                    send_rc_override(master, rc_roll, rc_pitch, THRUST_RC, 1500, 1500)
                    
                    # === OSD ===
                    cv2.putText(frame, f"🎯 AUTO | RC: {rc_roll}/{rc_pitch}/{THRUST_RC}", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(frame, f"err_x: {int(err_x):+4d} | roll: {math.degrees(desired_roll):+.1f}°", 
                                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(frame, f"err_y: {int(err_y):+4d} | pitch: {math.degrees(desired_pitch):+.1f}°", 
                                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    if err_x == 0 and err_y == 0:
                        status = "CENTERED"
                    else:
                        status = "TRACKING"
                    
                    cv2.putText(frame, f"⚡ {status}", (10, 110),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                else:
                    # Цель потеряна — нейтральные стики, но газ 100%
                    send_rc_override(master, 1500, 1500, THRUST_RC, 1500, 1500)
                    cv2.putText(frame, f"🎯 AUTO | TARGET LOST", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # ====================================================================
            # MANUAL
            # ====================================================================
            
            else:
                send_rc_override(master, ch1, ch2, ch3, ch4, ch5)
                
                cv2.putText(frame, "🕹 MANUAL MODE", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                cv2.putText(frame, f"CH: {ch1}/{ch2}/{ch3}/{ch4}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                if target is not None:
                    x, y, w, h = target[3]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, "Target visible - toggle ON", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            
            # === ПОКАЗ КАДРА ===
            cv2.imshow(f"Simple P-Tracker", frame)
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