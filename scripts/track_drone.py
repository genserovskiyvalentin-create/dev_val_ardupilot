#!/usr/bin/env python3
import cv2
import numpy as np
import subprocess
import time
from pymavlink import mavutil

# --- НАСТРОЙКИ ---
WIDTH, HEIGHT = 640, 480
TARGET_ALTITUDE = 2.0  # Метры
KP_YAW = 0.005         # Коэффициент поворота (уменьшил для плавности)
MAX_YAW_RATE = 0.5     # Максимальная скорость поворота (рад/с, ~30 град/с)

def connect_mavlink():
    print("🔌 Подключение к ArduPilot SITL (порт 14550)...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    print("✅ Связь с автопилотом установлена!")
    return master

def arm_and_takeoff(master, altitude):
    print("🔒 Арминг и взлет...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("✅ Моторы запущены.")

    master.set_mode(master.mode_mapping()['GUIDED'])
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude
    )
    
    print(f"📈 Набор высоты до {altitude} м...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            print(f" Высота: {current_alt:.2f} м / {altitude} м")
            if current_alt >= altitude * 0.95:
                break
    print("✅ Взлет завершен. Переход в режим слежения.")

def send_velocity(master, vx, vy, vz, yaw_rate):
    """Отправка команды на изменение скорости в режиме GUIDED"""
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000010111000111,  # ✅ ИСПРАВЛЕНО: бит 11 = 0 (используем yaw_rate)
        0, 0, 0,             # x, y, z (игнорируются)
        vx, vy, vz,          # vx, vy, vz (скорости в м/с)
        0, 0, 0,             # afx, afy, afz (ускорения игнорируются)
        0, yaw_rate          # yaw, yaw_rate (рад/с)
    )

def main():
    master = connect_mavlink()
    arm_and_takeoff(master, TARGET_ALTITUDE)
    time.sleep(2)

    # Запуск видеопотока
    gst_cmd = [
        "gst-launch-1.0", "-q",
        "udpsrc", "port=5600", "!",
        "application/x-rtp, encoding-name=H264, payload=96", "!",
        "rtpjitterbuffer", "!",
        "rtph264depay", "!",
        "avdec_h264", "!",
        "videoconvert", "!",
        f"video/x-raw, format=BGR, width={WIDTH}, height={HEIGHT}", "!",
        "fdsink", "fd=1"
    ]
    proc = subprocess.Popen(gst_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=10**8)
    frame_size = WIDTH * HEIGHT * 3

    center_x, center_y = WIDTH // 2, HEIGHT // 2
    print("🎯 Режим слежения активирован. Дрон будет поворачиваться за кубиком!")
    print("   (Продольное движение отключено. Нажмите 'q' для посадки)")

    try:
        while True:
            raw_frame = proc.stdout.read(frame_size)
            if len(raw_frame) != frame_size:
                continue

            frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Детекция красного цвета
            mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # По умолчанию дрон зависает (все скорости = 0)
            vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0

            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 500:
                    x, y, w, h = cv2.boundingRect(c)
                    cx, cy = x + w // 2, y + h // 2
                    
                    # Рисуем интерфейс
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.line(frame, (center_x, 0), (center_x, HEIGHT), (255, 255, 0), 1)
                    cv2.line(frame, (0, center_y), (WIDTH, center_y), (255, 255, 0), 1)

                    # --- ЛОГИКА УПРАВЛЕНИЯ ---
                    err_x = cx - center_x  # Отклонение по горизонтали (пиксели)

                    # ✅ ИСПРАВЛЕНО: убран минус, чтобы дрон поворачивал В СТОРОНУ цели
                    yaw_rate = err_x * KP_YAW
                    yaw_rate = np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)

                    # Отладочная информация на экране
                    cv2.putText(frame, f"err_x: {err_x:+4d} px", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(frame, f"yaw_rate: {yaw_rate:+.3f} rad/s", (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Отправляем команды автопилоту (даже если кубик не найден — дрон зависнет)
            send_velocity(master, vx, vy, vz, yaw_rate)

            cv2.imshow("Drone Tracking (Yaw Only)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"❌ Ошибка: {e}")
    finally:
        print("🛑 Остановка и посадка...")
        send_velocity(master, 0, 0, 0, 0)
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
        )
        proc.terminate()
        cv2.destroyAllWindows()
        print("✅ Скрипт завершен.")

if __name__ == "__main__":
    main()