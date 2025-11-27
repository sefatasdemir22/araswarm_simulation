#!/bin/bash

echo "🧹 Temizlik yapılıyor..."
killall -9 px4 gzserver gzclient python3 2>/dev/null
sleep 2

echo "⚙️ Mağara fizik ayarları (250 Hz) kontrol ediliyor..."
sed -i 's/<real_time_update_rate>100/<real_time_update_rate>250/g' ~/araswarm_ws/src/gas_distribution_sim/worlds/cave.world
sed -i 's/<max_step_size>0.01/<max_step_size>0.004/g' ~/araswarm_ws/src/gas_distribution_sim/worlds/cave.world

echo "🚀 Simülasyon başlatılıyor..."

# 1. MAĞARA DÜNYASI (Terminal 1)
gnome-terminal --tab --title="Cave World" -- bash -c "source ~/araswarm_ws/install/setup.bash; ros2 launch gas_distribution_sim cave_simulation.launch.py; exec bash"

sleep 5

# 2. PX4 BEYNİ (Terminal 2)
# Not: Manuel denememizde çalışan '10015' ID'si ve klasör yolları eklendi.
gnome-terminal --tab --title="PX4 Brain" -- bash -c "cd ~/PX4-Autopilot/build/px4_sitl_default; export PX4_SYS_AUTOSTART=10015; ./bin/px4 -i 0 -d \"\$PWD/etc\" -w sitl_iris_0; exec bash"

sleep 5

# 3. DRONE GÖVDESİ / SPAWN (Terminal 3)
gnome-terminal --tab --title="Spawn Drone" -- bash -c "source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default; ros2 run gazebo_ros spawn_entity.py -entity iris -file ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf -x -2.0 -y 0.0 -z 0.2; exec bash"

echo "✅ SİSTEM HAZIR! Drone mağara girişine yerleşti."
