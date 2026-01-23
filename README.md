# vacuum-robot-ros

Robot aspirador autónomo con navegación en ROS Noetic + Gazebo.

## 📋 Requisitos

**Sistema:** Ubuntu 20.04 + ROS Noetic

### 0) Instalación (una sola vez por VM)

#### Instalar ROS Noetic
```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
```

#### Gazebo + plugins ROS para simulación
```bash
sudo apt update
sudo apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-navigation
```

#### Herramientas útiles
```bash
sudo apt install -y git
```

---

## 🚀 Instalación del Proyecto

### 1) Clonar el repositorio
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/emilioriofrio/vacuum-robot-ros.git
```

### 2) Compilar el workspace
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

---

## 🎮 Modos de Ejecución

### Opción 1: Sistema Completo (Navegación Autónoma) ⭐ RECOMENDADO

Inicia **Gazebo + RViz + Navegación** en un solo comando:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch vacuum_robot_description full_system.launch
```

**Esto incluye:**
- ✅ Simulación Gazebo con el mundo `house_blocked.world`
- ✅ Robot aspirador completamente funcional
- ✅ Sistema de navegación autónoma (AMCL + move_base)
- ✅ RViz con configuración de navegación
- ✅ Mapa pre-cargado

**Cómo usar:**
1. Espera a que se abran Gazebo y RViz
2. En RViz, haz clic en **"2D Nav Goal"** (flecha verde en la barra superior)
3. Haz clic en el mapa donde quieres que vaya el robot
4. ¡El robot navegará automáticamente evitando obstáculos!

---

### Opción 2: Solo Simulación (sin navegación)

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch vacuum_robot_description spawn_in_gazebo_v2.launch
```

Luego, en otra terminal, abre RViz:
```bash
source ~/catkin_ws/devel/setup.bash
rviz -d ~/catkin_ws/src/vacuum_robot_description/rviz/vacuum_robot.rviz
```

---

### Opción 3: Solo visualización del modelo (sin Gazebo)

```bash
roslaunch vacuum_robot_description display_v2.launch
```

---

## ⚡ Parámetros de Navegación Optimizados

El robot está configurado con **parámetros de velocidad 2x más rápidos** para navegación eficiente:

**Archivo:** `src/vacuum_robot_description/param/base_local_planner_params.yaml`

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `max_vel_x` | 1.0 m/s | Velocidad lineal máxima |
| `max_vel_theta` | 2.0 rad/s | Velocidad angular máxima |
| `acc_lim_x` | 2.5 m/s² | Aceleración lineal |
| `acc_lim_theta` | 3.5 rad/s² | Aceleración angular |
| `pdist_scale` | 0.4 | Prioridad de seguir el path |
| `gdist_scale` | 1.0 | Prioridad de llegar al goal |

Estos valores permiten al robot moverse **rápido pero de forma segura**, balanceando velocidad con precisión en la navegación.

---

## 📁 Estructura del Proyecto

```
vacuum_robot_description/
├── launch/          # Archivos de lanzamiento
├── maps/            # Mapas para navegación
├── meshes/          # Modelos 3D del robot
├── param/           # Parámetros de navegación
├── rviz/            # Configuraciones de RViz
├── urdf/            # Descripción del robot
└── worlds/          # Mundos de Gazebo
```

---

## 🔧 Troubleshooting

**Gazebo no abre o crashea:**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch vacuum_robot_description full_system.launch
```

**El robot no se mueve:**
- Verifica que los topics estén publicándose: `rostopic list`
- Revisa que `/cmd_vel` esté activo: `rostopic echo /cmd_vel`

**Warnings de TF_REPEATED_DATA:**
- Son normales durante la navegación, no afectan el funcionamiento

---

## 📝 Notas

- El sistema usa **AMCL** para localización
- **move_base** maneja la planificación de rutas
- Los parámetros están optimizados para velocidad manteniendo seguridad
- El mapa se carga automáticamente desde `maps/my_house.yaml`
