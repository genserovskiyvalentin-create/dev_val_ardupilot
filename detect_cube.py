#!/usr/bin/env python3
import cv2
import numpy as np
import subprocess

def main():
    # ВАЖНО: Проверьте разрешение вашей камеры в Gazebo!
    # Если картинка "рваная", измените эти значения (часто бывает 1280x720).
    WIDTH, HEIGHT = 640, 480 
    
    print(f"🚀 Запуск GStreamer (разрешение {WIDTH}x{HEIGHT})...")
    
    # Пайплайн через subprocess (работает стабильнее, чем cv2.VideoCapture без GStreamer)
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

    try:
        proc = subprocess.Popen(
            gst_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=10**8
        )
    except Exception as e:
        print(f"Ошибка запуска GStreamer: {e}")
        return

    frame_size = WIDTH * HEIGHT * 3
    print("✅ Ожидание видео... (нажмите 'q' для выхода)")

    while True:
        try:
            # Читаем кадр
            raw_frame = proc.stdout.read(frame_size)
            
            # Если поток прервался
            if len(raw_frame) != frame_size:
                print("⚠️ Сбой потока данных (кадр неполный). Пропускаем...")
                continue

            # Преобразуем в numpy array
            frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()

            # --- Логика детекции ---
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Красный цвет (2 диапазона)
            mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Берем самый большой объект
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 500: # Фильтр шума
                    x, y, w, h = cv2.boundingRect(c)
                    
                    # Рисуем рамку
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    cx, cy = x + w // 2, y + h // 2
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # Выводим текст
                    cv2.putText(frame, f"Target: {cx}, {cy}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # Показываем окно
            cv2.imshow("Drone Camera - Cube Detection", frame)
            
            # Выход по клавише
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            # 🔥 ВОТ ЗДЕСЬ МЫ УВИДИМ ПРИЧИНУ ПАДЕНИЯ
            print(f"КРИТИЧЕСКАЯ ОШИБКА: {e}")
            break

    proc.terminate()
    cv2.destroyAllWindows()
    print(" Скрипт остановлен.")

if __name__ == "__main__":
    main()