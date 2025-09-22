
```markdown name=src/robot_description/README.md
```markdown
# robot_description

Enthält URDF / XACRO, Meshes und components_config für das my_steel Chassis.

Wichtige Dateien
- urdf/*.xacro — Haupt‑XACRO Dateien
- meshes/ — 3D Modelle (STL/DAE)
- config/components/ — Komponenten‑Includes (lidar, imu, nerf)

Übliche Nutzung
- URDF erzeugen:
  ros2 run xacro xacro urdf/multi_drive_robot.urdf.xacro > /tmp/robot.urdf
- Test:
  ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=`cat /tmp/robot.urdf`

Hinweis
- Joint‑Namen müssen exakt mit robot_controllers/config/* übereinstimmen.
- Pflege Inertial‑Daten für realistische Simulation.