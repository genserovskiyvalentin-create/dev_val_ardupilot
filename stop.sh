#!/bin/bash

echo "========================================"
echo "🛑 ПОЛНАЯ ОЧИСТКА СИМУЛЯЦИИ"
echo "========================================"

# === ШАГ 1: Убиваем все процессы ===
echo ""
echo "[1/4] Останавливаем процессы..."

pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gz launch" 2>/dev/null
pkill -9 -f gazebo 2>/dev/null
pkill -9 -f arducopter 2>/dev/null
pkill -9 -f mavproxy 2>/dev/null
pkill -9 -f sim_vehicle 2>/dev/null
pkill -9 -f fly.py 2>/dev/null
pkill -9 -f "gst-launch" 2>/dev/null
pkill -9 -f companion 2>/dev/null
pkill -9 -f "python3.*companion" 2>/dev/null

sleep 1
echo "   ✅ Процессы остановлены"

# === ШАГ 2: Очищаем кэш Gazebo ===
echo ""
echo "[2/4] Очищаем кэш Gazebo..."

rm -rf ~/.gz/sim 2>/dev/null
rm -rf ~/.gz/fuel 2>/dev/null
rm -rf ~/.gz/log 2>/dev/null
rm -rf ~/.ignition 2>/dev/null

echo "   ✅ Кэш очищен"

# === ШАГ 3: Очищаем shared memory ===
echo ""
echo "[3/4] Очищаем shared memory (/dev/shm)..."

# Проверяем сколько занято ДО очистки
echo "   До очистки:"
df -h /dev/shm 2>/dev/null | tail -1 | awk '{print "      Занято: " $3 " / " $2 " (" $5 ")"}'

# Очищаем (может потребоваться sudo)
sudo rm -rf /dev/shm/* 2>/dev/null || rm -rf /dev/shm/* 2>/dev/null

echo "   После очистки:"
df -h /dev/shm 2>/dev/null | tail -1 | awk '{print "      Занято: " $3 " / " $2 " (" $5 ")"}'
echo "   ✅ Shared memory очищена"

# === ШАГ 4: Проверяем остаточные процессы ===
echo ""
echo "[4/4] Проверка остаточных процессов..."

REMAINING=$(ps aux | grep -E "gz sim|gz launch|gazebo|arducopter|mavproxy|sim_vehicle|fly.py|gst-launch|companion" | grep -v grep)

if [ -z "$REMAINING" ]; then
    echo "   ✅ Все процессы остановлены"
else
    echo "   ⚠️  Остались процессы:"
    echo "$REMAINING"
    echo ""
    echo "   Попытка принудительной остановки..."
    pkill -9 -f "gz" 2>/dev/null
    pkill -9 -f "gazebo" 2>/dev/null
    sleep 1
    
    # Повторная проверка
    REMAINING2=$(ps aux | grep -E "gz sim|gazebo|arducopter" | grep -v grep)
    if [ -z "$REMAINING2" ]; then
        echo "   ✅ Теперь все процессы остановлены"
    else
        echo "   ❌ Не удалось остановить все процессы:"
        echo "$REMAINING2"
    fi
fi

echo ""
echo "========================================"
echo "✅ ОЧИСТКА ЗАВЕРШЕНА"
echo "========================================"
echo ""
echo "💡 Совет: если Gazebo всё равно показывает"
echo "   чёрный экран — перезапусти графический сервер:"
echo "   sudo systemctl restart gdm"
echo ""