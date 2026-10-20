# asv_sim
Simulasi ASV untuk KKI (ROS 2 Jazzy, Gazebo Harmonic, Ubuntu 24.04).

## Cara pakai
    cd ~/ros2_ws/src && git clone https://github.com/anip1/asv_sim.git asv_sim
    cd ~/ros2_ws && colcon build --symlink-install --packages-select asv_sim
    source install/setup.bash
    ros2 launch asv_sim sim.launch.py
