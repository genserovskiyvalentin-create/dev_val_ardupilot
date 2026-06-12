#!/bin/bash

BASE=$(dirname "$(realpath "$0")")

echo "[1/2] Запуск Gazebo..."
gz sim -v4 -r $BASE/worlds/my_world.sdf &
sleep 5

echo "[2/2] Запуск ArduPilot SITL..."
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map
