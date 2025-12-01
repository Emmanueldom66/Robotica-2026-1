# Proyecto Final - Manipuladores Robóticos ROS2

Este repositorio contiene la implementación de dos manipuladores robóticos en ROS2, cada uno con arquitecturas de control completas incluyendo cinemática directa e inversa, dinámica mediante Lagrange, y visualización en RViz.

## 📋 Contenido

- [Descripción General](#descripción-general)
- [Arquitectura del Sistema](#arquitectura-del-sistema)
- [Robots Implementados](#robots-implementados)
- [Requisitos](#requisitos)
- [Instalación](#instalación)
- [Uso](#uso)
- [Estructura del Proyecto](#estructura-del-proyecto)
- [Comunicación entre Nodos](#comunicación-entre-nodos)
- [Desarrollo](#desarrollo)

---

## 🤖 Descripción General

Este proyecto implementa dos manipuladores robóticos con diferentes configuraciones cinemáticas:

- **Robot 1**: Manipulador RRR de 3 grados de libertad operando en el plano horizontal (XY)
- **Robot 2**: Manipulador RRR de 3 grados de libertad con base rotatoria y brazos verticales (3D)

Ambos robots implementan:
- ✅ Cinemática directa e inversa
- ✅ Planificación de trayectorias mediante polinomios de 5to orden
- ✅ Dinámica calculada por el método de Lagrange
- ✅ Control en espacio de juntas
- ✅ Visualización en RViz con interacción mediante puntos clickeados

---

## 🏗️ Arquitectura del Sistema

Cada robot está organizado en varios paquetes especializados:

### Paquetes por Robot

1. **`robotX_description`**: Descripción URDF y configuraciones de visualización
2. **`robotX_control`**: Controladores, cinemática y dinámica
3. **`robot_bringup`**: Launch files para iniciar los sistemas completos

### Nodos Principales

Cada robot ejecuta 3 nodos fundamentales:

```
┌─────────────────────────────────────────────────────────┐
│                  Manipulator Controller                  │
│  - Cinemática directa e inversa                         │
│  - Generación de trayectorias                           │
│  - Cálculo de dinámica (Lagrange)                       │
└────────────────┬────────────────────────────────────────┘
                 │ /robotX/joint_goals
                 ↓
┌─────────────────────────────────────────────────────────┐
│                  Controller Manager                      │
│  - Gestión de comandos                                  │
│  - Procesamiento de señales                             │
└────────────────┬────────────────────────────────────────┘
                 │ /robotX/joint_hardware_objectives
                 ↓
┌─────────────────────────────────────────────────────────┐
│                  Hardware Interface                      │
│  - Interfaz con hardware (simulada)                     │
│  - Publicación de estados de juntas                     │
└─────────────────────────────────────────────────────────┘
```

---

## 🦾 Robots Implementados

### Robot 1 - Manipulador RRR Planar (XY)

**Características:**
- 3 juntas rotacionales en el plano horizontal
- Longitud de eslabones: 0.3m cada uno
- Espacio de trabajo: Plano XY
- Sin efectos gravitacionales

**Configuración de Juntas:**
```
Base → Shoulder → Arm → Forearm
(θ₁)     (θ₂)     (θ₃)
```

**Parámetros DH:**
- Todas las rotaciones sobre eje Z
- Movimiento completamente en plano horizontal

### Robot 2 - Manipulador con Base Rotatoria (3D)

**Características:**
- Base rotatoria + 2 juntas RR verticales
- Longitudes: Base 5cm, Eslabón1 20cm, Eslabón2 35cm
- Espacio de trabajo: 3D completo
- Incluye efectos gravitacionales

**Configuración de Juntas:**
```
Base Rotatoria → Shoulder → Arm
    (θ₁)           (θ₂)     (θ₃)
     (Z)          (Z→X)     (Z)
```

**Características especiales:**
- Rotación completa en base (360°)
- Movimiento vertical del manipulador RR
- Dinámica más compleja por gravedad

---

## 📦 Requisitos

### Software Requerido

- **ROS2 Humble** (o superior)
- **Python 3.8+**
- **Ubuntu 22.04** (recomendado)

### Dependencias Python

```bash
pip install sympy matplotlib numpy --break-system-packages
```

### Paquetes ROS2

```bash
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-xacro
```

---

## 🚀 Instalación

### 1. Clonar el Repositorio

```bash
# Opción A: Crear nuevo workspace
mkdir -p ~/proyecto_final_ws/src
cd ~/proyecto_final_ws/src
git clone <url-del-repositorio> .

# Opción B: Usar workspace existente
cd ~/ros2_ws/src
git clone <url-del-repositorio> proyecto_final
```

### 2. Instalar Dependencias

```bash
cd ~/proyecto_final_ws  # O tu workspace
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Compilar

```bash
colcon build --symlink-install
```

### 4. Configurar el Entorno

```bash
source install/setup.bash

# Opcional: Agregar a ~/.bashrc
echo "source ~/proyecto_final_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🎮 Uso

### Ejecutar Robot 1 (Plano XY)

```bash
# En una terminal
ros2 launch robot_bringup robot1.launch.py
```

Esto iniciará:
- ✅ Controlador del manipulador
- ✅ Gestor de control
- ✅ Interfaz de hardware (simulada)
- ✅ RViz con configuración personalizada
- ✅ Robot State Publisher

### Ejecutar Robot 2 (3D con Base Rotatoria)

```bash
# En una terminal
ros2 launch robot_bringup robot2.launch.py
```

### Enviar Comandos de Posición

#### Opción 1: Mediante Publicación de Tópicos

**Robot 1 (x, y, alpha):**
```bash
ros2 topic pub /robot1/end_effector_goal geometry_msgs/msg/Twist \
  "{linear: {x: 0.6, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --once
```

**Robot 2 (x, y, z):**
```bash
ros2 topic pub /robot2/end_effector_goal geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.2, z: 0.3}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --once
```

#### Opción 2: Mediante RViz (Publish Point)

1. Abrir RViz (se abre automáticamente con el launch)
2. Seleccionar la herramienta **"Publish Point"** en la barra superior
3. Hacer clic en el punto deseado del espacio de trabajo
4. El robot calculará la trayectoria y la ejecutará automáticamente

**Nota:** Al usar Publish Point desde RViz, se mostrarán automáticamente las gráficas de:
- Posiciones, velocidades y aceleraciones en espacio de trabajo
- Posiciones, velocidades y aceleraciones en espacio de juntas  
- Esfuerzos/pares calculados en cada junta

---

## 📁 Estructura del Proyecto

```
Proyecto_Final_ws/
├── src/
│   ├── robot1_control/
│   │   ├── robot1_control/
│   │   │   ├── controller_manager.py      # Gestor de comandos
│   │   │   ├── hardware_interface.py      # Interfaz hardware simulada
│   │   │   ├── manipulator_controller.py  # Control principal
│   │   │   ├── kinematics.py              # Cinemática directa/inversa
│   │   │   └── dynamics.py                # Dinámica de Lagrange
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── robot1_description/
│   │   ├── urdf/
│   │   │   └── robot1.urdf                # Modelo URDF Robot 1
│   │   ├── rviz/
│   │   │   └── robot1_config.rviz         # Configuración RViz
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── robot2_control/
│   │   ├── robot2_control/
│   │   │   ├── controller_manager.py      # Gestor de comandos
│   │   │   ├── hardware_interface.py      # Interfaz hardware simulada
│   │   │   ├── manipulator_controller.py  # Control principal
│   │   │   ├── kinematics.py              # Cinemática directa/inversa
│   │   │   └── dynamics.py                # Dinámica de Lagrange
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── robot2_description/
│   │   ├── urdf/
│   │   │   └── robot2.urdf                # Modelo URDF Robot 2
│   │   ├── rviz/
│   │   │   └── robot2_config.rviz         # Configuración RViz
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── robot_bringup/
│       ├── launch/
│       │   ├── robot1.launch.py           # Launch Robot 1
│       │   └── robot2.launch.py           # Launch Robot 2
│       ├── package.xml
│       └── setup.py
│
├── build/      # Archivos de compilación (auto-generado)
├── install/    # Archivos instalados (auto-generado)
└── log/        # Logs de compilación (auto-generado)
```

---

## 📡 Comunicación entre Nodos

### Tópicos Robot 1

| Tópico | Tipo | Descripción |
|--------|------|-------------|
| `/robot1/end_effector_goal` | `geometry_msgs/Twist` | Posición objetivo (x, y, α) |
| `/robot1/clicked_point` | `geometry_msgs/PointStamped` | Punto clickeado en RViz |
| `/robot1/joint_goals` | `sensor_msgs/JointState` | Objetivos de juntas calculados |
| `/robot1/joint_hardware_objectives` | `sensor_msgs/JointState` | Comandos a hardware |
| `/robot1/joint_states` | `sensor_msgs/JointState` | Estado actual de juntas |
| `/robot1/robot_description` | `std_msgs/String` | Descripción URDF |

### Tópicos Robot 2

| Tópico | Tipo | Descripción |
|--------|------|-------------|
| `/robot2/end_effector_goal` | `geometry_msgs/Twist` | Posición objetivo (x, y, z) |
| `/robot2/clicked_point` | `geometry_msgs/PointStamped` | Punto clickeado en RViz |
| `/robot2/joint_goals` | `sensor_msgs/JointState` | Objetivos de juntas calculados |
| `/robot2/joint_hardware_objectives` | `sensor_msgs/JointState` | Comandos a hardware |
| `/robot2/joint_states` | `sensor_msgs/JointState` | Estado actual de juntas |
| `/robot2/robot_description` | `std_msgs/String` | Descripción URDF |

### Visualizar Tópicos Activos

```bash
# Ver todos los tópicos
ros2 topic list

# Monitorear un tópico específico
ros2 topic echo /robot1/joint_states

# Ver información de un tópico
ros2 topic info /robot1/joint_goals
```

---

## 🔧 Desarrollo

### Modificar Parámetros del Robot

**Longitudes de eslabones** (`kinematics.py`):
```python
# Robot 1
self.l1 = 0.3  # metros
self.l2 = 0.3
self.l3 = 0.3

# Robot 2
self.l0 = 0.05  # Altura base
self.l1 = 0.05  # Eslabón 1
self.l2 = 0.20  # Eslabón 2
self.l3 = 0.35  # Eslabón 3
```

**Masas** (`dynamics.py`):
```python
# Valores por defecto [eslabón1, eslabón2, eslabón3] en kg
self.define_dynamics(mass = [0.25, 0.25, 0.25])
```

**Parámetros de trayectoria** (`manipulator_controller.py`):
```python
# Duración de la trayectoria (segundos)
self.robot_kinematics.trajectory_generator(
    self.current_joint_states.position,
    [x_goal, y_goal, z_goal], 
    3  # ← Duración en segundos
)

# Frecuencia de muestreo (kinematics.py)
self.freq = 30  # Hz
```

### Compilar Después de Cambios

```bash
# Compilar todo
colcon build --symlink-install

# Compilar solo un paquete
colcon build --packages-select robot1_control --symlink-install

# Limpiar y recompilar
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Depuración

**Ver logs en tiempo real:**
```bash
ros2 run robot1_control manipulator_controller
```

**Verificar transformaciones TF:**
```bash
ros2 run tf2_tools view_frames
# Genera frames.pdf con el árbol de transformaciones
```

**Inspeccionar modelo URDF:**
```bash
# Ver el modelo parseado
check_urdf src/robot1_description/urdf/robot1.urdf
```

---

## 🧮 Teoría Implementada

### Cinemática Directa

Se utilizan **transformaciones homogéneas** para calcular la posición del efector final:

```
T₀ₚ = T₀₁ × T₁₂ × T₂₃ × T₃ₚ
```

### Cinemática Inversa

Mediante el **Jacobiano inverso** y integración numérica:

```
q̇ = J⁻¹(q) × ξ̇
```

Donde:
- `q̇`: Velocidades en espacio de juntas
- `J⁻¹`: Inversa del Jacobiano
- `ξ̇`: Velocidades en espacio de trabajo

### Dinámica - Método de Lagrange

```
τ = d/dt(∂L/∂q̇) - ∂L/∂q
```

Donde:
- `L = K - U` (Lagrangiano)
- `K`: Energía cinética
- `U`: Energía potencial
- `τ`: Pares en las juntas

### Planificación de Trayectorias

Polinomios de **5to orden** con condiciones de frontera:

```
λ(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
```

Garantizan:
- Posición, velocidad y aceleración continuas
- Velocidad y aceleración cero en inicio/fin

---

## 📊 Características Avanzadas

### Protección contra Comandos Concurrentes

Los controladores rechazan nuevos comandos mientras se ejecuta una trayectoria:

```
Robot X - Trayectoria en progreso. Mensaje rechazado
```

### Visualización Automática de Gráficas

Al usar **Publish Point** en RViz, se generan automáticamente 3 ventanas con gráficas:

1. **Espacio de trabajo**: x, y, z (o x, y, α para Robot 1)
   - Posiciones, velocidades, aceleraciones vs tiempo

2. **Espacio de juntas**: θ₁, θ₂, θ₃
   - Posiciones, velocidades, aceleraciones vs tiempo

3. **Esfuerzos**: τ₁, τ₂, τ₃
   - Pares calculados en cada junta vs tiempo

### Marcas de Tiempo Sincronizadas

Todos los mensajes incluyen marcas de tiempo del reloj de ROS2 para sincronización precisa.

---

## ⚠️ Solución de Problemas

### El robot no se mueve

**Verificar que los nodos estén corriendo:**
```bash
ros2 node list
```

**Debería mostrar:**
```
/robot1/controller_manager
/robot1/hardware_interface
/robot1/manipulator_controller
/robot1/rviz
/robot1/robot_state_publisher
```

### Error de singularidad en Jacobiano

Ocurre cuando el robot alcanza configuraciones singulares. **Soluciones:**
- Evitar puntos objetivo muy cercanos o fuera del espacio de trabajo
- Modificar la posición inicial de las juntas

### RViz no muestra el robot

**Verificar el tópico de descripción:**
```bash
ros2 topic echo /robot1/robot_description --once
```

**Reiniciar RViz:**
```bash
# Matar el proceso
pkill rviz2
# Relanzar
ros2 launch robot_bringup robot1.launch.py
```

### Dependencias de Python faltantes

```bash
pip install sympy matplotlib numpy --break-system-packages
```

---

## 📝 Notas Importantes

### Sobre `--symlink-install`

Se recomienda compilar con `--symlink-install` para desarrollo activo. Esto permite modificar archivos Python sin recompilar:

```bash
colcon build --symlink-install
```

**Excepción:** Cambios en `package.xml`, `setup.py` o código C++ requieren recompilación.

### Archivos a Ignorar en Git

El repositorio incluye un `.gitignore` adecuado que excluye:
- `build/`, `install/`, `log/`
- Archivos Python compilados (`.pyc`, `__pycache__`)
- Archivos de IDEs

---

## 👥 Autores

- **Maintainer**: robousr
- **Email**: emmanueldom007@outlook.com / luisfdopapu@gmail.com

---


---

## 🔗 Referencias

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [RViz User Guide](http://wiki.ros.org/rviz/UserGuide)
- [Robot Kinematics and Dynamics (Craig, 2005)](https://www.pearson.com/store/p/introduction-to-robotics/P100000434453)

---
