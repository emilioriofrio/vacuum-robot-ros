# vacuum-robot-ros
## README
Nota: este repo asume un workspace en `~/catkin_ws` (ROS Noetic + Ubuntu 20.04).
## 0) Requisitos (una sola vez por VM)

### Instalar ROS Noetic
(Si ya se tiene `ros-noetic-desktop-full`, omitir este bloque)

sudo apt update
sudo apt install -y ros-noetic-desktop-full

### Gazebo + plugins ROS para simulación

sudo apt update
sudo apt install -y ros-noetic-gazebo-ros-pkgs
Herramientas útiles

sudo apt update
sudo apt install -y git

### 1) Clonar repo y colocarlo en el workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/emilioriofrio/vacuum-robot-ros.git
### 2) Compilar el workspace
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
### 3) Ejecución principal (Gazebo + robot + mundo
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch vacuum_robot_description spawn_in_gazebo_v2.launch
### 4) Abrir RViz con config guardada
(En otra terminal)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rviz -d ~/catkin_ws/src/vacuum_robot_description/rviz/vacuum_robot.rviz

#### Alternativa: visualizar solo el modelo (sin Gazebo)
roslaunch vacuum_robot_description display_v2.launch
