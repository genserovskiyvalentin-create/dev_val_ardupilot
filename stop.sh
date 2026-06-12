#!/bin/bash

echo "Останавливаем симуляцию..."

pkill -9 -f "gz sim"
pkill -9 -f arducopter
pkill -9 -f mavproxy
pkill -9 -f sim_vehicle
pkill -9 -f fly.py
pkill -9 -f "gst-launch"

sleep 1

echo "Проверка остаточных процессов:"
ps aux | grep -E "gz sim|arducopter|mavproxy|sim_vehicle|fly.py" | grep -v grep

echo "Готово."
