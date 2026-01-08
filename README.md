# vacuum-robot-ros
# README 
## Objetivo: clonar, compilar y ejecutar el robot aspiradora con LIDAR en Gazebo + RViz.
##  NOTA: este repo asume un workspace en: ~/catkin_ws  (ROS Noetic + Ubuntu 20.04)

# 0) REQUISITOS (una sola vez por VM)

## Instalar ROS Noetic
## (Si ya tienes ros-noetic-desktop-full instalado, pasa a la sección 1)
sudo apt update
sudo apt install -y ros-noetic-desktop-full

## Gazebo + plugins ROS para simulación
sudo apt update
sudo apt install -y ros-noetic-gazebo-ros-pkgs

## Herramientas útiles
sudo apt update
sudo apt install -y git

# 1) CLONAR REPO Y COLOCARLO EN EL WORKSPACE

## Crear estructura base del workspace si no existe
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

## Clonar el repositorio
git clone https://github.com/emilioriofrio/vacuum-robot-ros.git

# 2) COMPILAR EL WORKSPACE
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash

# 3) EJECUCIÓN PRINCIPAL (GAZEBO + ROBOT + MUNDO)
## Lanza Gazebo con el mundo tipo casa + spawnea el robot v2 (incluye LIDAR)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch vacuum_robot_description spawn_in_gazebo_v2.launch

# 4) ABRIR RVIZ CON CONFIG GUARDADA
## En otra terminal:
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rviz -d ~/catkin_ws/src/vacuum_robot_description/rviz/vacuum_robot.rviz~~
### roslaunch vacuum_robot_description display_v2.launch


