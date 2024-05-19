# conan-slam
EKF-based SLAM for valet, remote parking, and including or nav in GPS-denied situations. 

# This branch provides a SLAM solution based on EKF and Particle Filter.

# Prerequisite (Window):
* Conan
* Visual Studio 2022 (Win)
* CMake (3.28.1)

# Build (Window):
* Run **run_debug.bat** or **run_release.bat**
* *Run*.

## Outcome verification through visualizer (no support for visualizer)
![image](https://github.com/rajiv1977/conan-slam/assets/16018587/1f84ed03-29d7-4f07-b9ec-7c3b9182d81c)
Green dots: landmarks / GSP transceiver locations /wifi spots. 
Orange dots: estimated Features (Perfectly aligned with feature locations).
Yellow line: pathway (connected by waypoints)
White triangle: host vehicle's location. 
Red dots: observations. 
Blue lines: line of sight of landmarks.  
