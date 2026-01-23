# robot_description

## Übersicht
Das **robot_description** Paket enthält alle notwendigen Dateien zur Beschreibung des *robot_xl* Roboters für ROS 2 Simulation und Visualisierung. Es stellt das URDF/XACRO-Modell, zugehörige 3D-Meshes sowie Konfigurationsdateien bereit, die von `robot_state_publisher`, Gazebo, Webots und den Controller-Setups verwendet werden.

## Paketstruktur
```
robot_description/
├── urdf/                       # XACRO-Dateien zur URDF-Generierung
│   ├── common/                 # Gemeinsame Simulator-Includes
│   │   ├── gazebo.urdf.xacro
│   │   ├── ros2_control.urdf.xacro
│   │   └── webots.urdf.xacro
│   └── robot_xl/               # Robot XL spezifische Modelle
│       ├── body.urdf.xacro
│       ├── wheel.urdf.xacro
│       ├── robot_xl_macro.urdf.xacro
│       └── components/         # Komponenten (Antenne, Kamera, Nerf-Launcher)
├── meshes/                     # 3D-Meshes (DAE/STL)
│   └── robot_xl/
│       ├── body.dae
│       ├── body_colision.stl
│       ├── wheel_a.dae, wheel_b.dae
│       ├── mecanum_a.dae, mecanum_b.dae
│       └── components/         # Komponenten-Meshes
├── config/                     # Konfigurationsdateien
│   └── robot_xl/
│       ├── autonomy.yaml
│       ├── basic.yaml
│       ├── manipulation.yaml
│       ├── telepresence.yaml
│       └── xbox_teleop.yaml
├── launch/                     # Launch-Dateien
│   ├── load_urdf.launch.py    # URDF laden und publishen
│   └── rviz.launch.py          # RViz-Visualisierung starten
├── rviz/                       # RViz-Konfigurationen
├── test/                       # Unit-Tests
├── CMakeLists.txt
└── package.xml
```

## Verzeichnisse im Detail

### `urdf/`
Die XACRO-Dateien beschreiben die kinematische Struktur, Geometrie und physikalischen Eigenschaften des Roboters.

- **`robot_xl.urdf.xacro`** – Haupt-XACRO-Datei, die alle Komponenten zusammenführt
- **`urdf/robot_xl/robot_xl_macro.urdf.xacro`** – Macro-Definition des kompletten Roboters
- **`urdf/robot_xl/body.urdf.xacro`** – Roboter-Chassis (base_link)
- **`urdf/robot_xl/wheel.urdf.xacro`** – Radmodelle (normal und mecanum)
- **`urdf/robot_xl/components/`** – Optionale Komponenten:
  - `antenna.urdf.xacro` – Antenne
  - `camera_mount.urdf.xacro` – Kamera-Halterung
  - `nerf_launcher.urdf.xacro` – Nerf-Launcher
- **`urdf/common/`** – Simulator-Integrationen:
  - `gazebo.urdf.xacro` – Gazebo-Plugins (differential drive, sensors, etc.)
  - `ros2_control.urdf.xacro` – ros2_control Hardware-Interfaces
  - `webots.urdf.xacro` – Webots-spezifische Anpassungen

### `meshes/`
3D-Modelle für visuelle Darstellung und Kollisionsgeometrie:

- **Visual Meshes (.dae):** Hochauflösende Collada-Dateien für Visualisierung
- **Collision Meshes (.stl):** Vereinfachte STL-Geometrie für Physiksimulation
- Unterverzeichnis `robot_xl/components/` enthält Meshes für optionale Komponenten

### `config/`
YAML-Konfigurationen definieren verschiedene Hardware-Setups:

- **`basic.yaml`** – Basis-Konfiguration
- **`autonomy.yaml`** – Autonomie-Setup (Lidar, IMU)
- **`manipulation.yaml`** – Manipulator-Konfiguration
- **`telepresence.yaml`** – Telepräsenz mit Kamera/Audio
- **`xbox_teleop.yaml`** – Xbox-Controller-Mapping

Diese Dateien werden in `load_urdf.launch.py` verwendet, um das finale URDF dynamisch zu generieren.

### `launch/`
- **`load_urdf.launch.py`** – Lädt URDF basierend auf gewählter Konfiguration, startet `robot_state_publisher`
- **`rviz.launch.py`** – Startet RViz mit vordefinierter Konfiguration

## Verwendung

### URDF manuell generieren
```bash
# Basis-Roboter ohne Komponenten
ros2 run xacro xacro urdf/robot_xl.urdf.xacro > /tmp/robot.urdf

# Mit spezifischer Konfiguration
ros2 run xacro xacro urdf/robot_xl.urdf.xacro \
  components_config_path:=$(ros2 pkg prefix robot_description)/share/robot_description/config/robot_xl/autonomy.yaml \
  > /tmp/robot_autonomy.urdf
```

### URDF via Launch-File laden
```bash
# Standard-Konfiguration
ros2 launch robot_description load_urdf.launch.py

# Mit spezifischer Konfiguration
ros2 launch robot_description load_urdf.launch.py \
  components_config:=autonomy
```

### Visualisierung in RViz
```bash
ros2 launch robot_description rviz.launch.py
```

## Wichtige Hinweise

- **Joint-Namen:** Müssen exakt mit den Namen in `robot_controllers/config/*` übereinstimmen
- **Inertial-Daten:** Für realistische Gazebo-Simulation aktuell halten (siehe `body.urdf.xacro` und `wheel.urdf.xacro`)
- **Neue Komponenten hinzufügen:**
  1. XACRO-Datei in `urdf/robot_xl/components/` erstellen
  2. Meshes in `meshes/robot_xl/components/` ablegen
  3. Komponente in der entsprechenden Config-YAML (z.B. `autonomy.yaml`) aktivieren
- **Mesh-Formate:**
  - `.dae` für visuelle Darstellung (mit Farben/Texturen)
  - `.stl` für Kollisionsgeometrie (einfache Geometrie für Performance)

## Tests
Unit-Tests im `test/` Verzeichnis prüfen URDF-Validität:
```bash
colcon test --packages-select robot_description
```
