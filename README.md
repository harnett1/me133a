# me133a
Final project for ME133a: Squash-playing Robotic Arm Simulation in Gazebo

Please see the wiki for a breakdown of the code structure!

# Building Gazebodemo Package
1. Remove `build`, `install`, and `log` to clear previous build history.
2. While in `133final/`, run `colcon-build --symlink-install && source install/setup.bash`

# Running Gazebodemo Squash Game Package
1. Build package
2. Run `ros2 launch gazebodemo gazebo_bounce.launch.py`. Ensure that `node_gui` is commented out at the bottom of the launch file. It should be already.

# Running Gazebodemo Drivemotion Spline Package
1. Build package
2. In `133final/`, `ros2 launch gazebodemo gazebo_sevenDOF_usegui.launch.py`. Ensure that `node_gui` is commented out at the bottom of the launch file. It should be already.
3. In another terminal in the same directory, run `ros2 run gazebodemo drivemotion`.
