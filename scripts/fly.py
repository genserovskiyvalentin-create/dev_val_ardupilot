from pymavlink import mavutil
import time

# Подключаемся к ArduPilot SITL (тот же порт что и MAVProxy)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("Ждём heartbeat...")
master.wait_heartbeat()
print("Подключено!")

# Переводим в режим GUIDED
master.set_mode('GUIDED')
time.sleep(2)

# Армируем
master.arducopter_arm()
master.motors_armed_wait()
print("Армирован!")

# Взлёт на 10 метров
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 10
)

print("Взлёт!")
time.sleep(15)

# Зависаем (просто ждём)
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print(f"\rВысота: {msg.relative_alt/1000:.1f} м", end='', flush=True)
