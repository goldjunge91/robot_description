# robot_description Architecture

## Overview

This package contains the URDF/xacro robot model definitions, meshes, and component configurations for the robot_xl platform.

## Package Structure

```mermaid
graph TB
    subgraph "robot_description Package"
        direction TB
        
        subgraph "URDF/Xacro Files"
            MAIN[robot.urdf.xacro<br/>Main Entry Point]
            CORE[robot_core.xacro<br/>Base Structure]
            COMP[Component Xacros<br/>camera, lidar, imu, etc.]
            RC[ros2_control.urdf.xacro<br/>Hardware Interface]
        end
        
        subgraph "Meshes"
            STL[STL Files<br/>Visual & Collision]
            DAE[DAE Files<br/>Textured Models]
        end
        
        subgraph "Configuration"
            CC[components_config/<br/>YAML Configs]
        end
        
        MAIN --> CORE
        MAIN --> COMP
        MAIN --> RC
        CORE --> STL
        CORE --> DAE
        MAIN --> CC
    end
    
    style MAIN fill:#4CAF50
    style CORE fill:#2196F3
    style RC fill:#FF9800
```

## URDF Structure

```mermaid
graph TB
    subgraph "robot.urdf.xacro"
        ARGS[Arguments<br/>robot_model<br/>components_config]
        
        INCLUDES[Include Files]
        
        ARGS --> INCLUDES
    end
    
    subgraph "Included Files"
        CORE[robot_core.xacro<br/>Base + Wheels]
        RC[ros2_control.urdf.xacro<br/>Hardware Interface]
        
        subgraph "Optional Components"
            CAM[camera.xacro]
            LIDAR[lidar.xacro]
            IMU[imu.xacro]
            MANIP[manipulator.xacro]
        end
    end
    
    subgraph "Generated URDF"
        LINKS[Links<br/>base_link, wheels, sensors]
        JOINTS[Joints<br/>Fixed, Continuous]
        GAZEBO[Gazebo Plugins]
        HW[ros2_control Tags]
    end
    
    INCLUDES --> CORE
    INCLUDES --> RC
    INCLUDES --> CAM
    INCLUDES --> LIDAR
    INCLUDES --> IMU
    INCLUDES --> MANIP
    
    CORE --> LINKS
    CORE --> JOINTS
    RC --> HW
    CAM --> GAZEBO
    LIDAR --> GAZEBO
    
    style CORE fill:#4CAF50
    style RC fill:#FF9800
```

## Link Hierarchy

```mermaid
graph TB
    BL[base_link<br/>Main Body]
    
    subgraph "Wheels"
        FLW[front_left_wheel_link]
        FRW[front_right_wheel_link]
        RLW[rear_left_wheel_link]
        RRW[rear_right_wheel_link]
    end
    
    subgraph "Sensors"
        IL[imu_link<br/>ICM20948]
        LL[laser_link<br/>RPLiDAR]
        CL[camera_link<br/>USB Camera]
    end
    
    subgraph "Optional"
        ML[manipulator_base_link<br/>Open Manipulator X]
    end
    
    BL --> FLW
    BL --> FRW
    BL --> RLW
    BL --> RRW
    BL --> IL
    BL --> LL
    BL --> CL
    BL --> ML
    
    style BL fill:#4CAF50
```

## Joint Types

```mermaid
graph LR
    subgraph "Joint Types"
        direction TB
        
        CONT[Continuous Joints<br/>4x Wheel Joints]
        FIXED[Fixed Joints<br/>Sensor Mounts]
        REV[Revolute Joints<br/>Manipulator Optional]
    end
    
    subgraph "Properties"
        direction TB
        
        AXIS[Axis<br/>Rotation Axis]
        LIMITS[Limits<br/>Position, Velocity, Effort]
        DYNAMICS[Dynamics<br/>Damping, Friction]
    end
    
    CONT --> AXIS
    CONT --> LIMITS
    CONT --> DYNAMICS
    
    REV --> AXIS
    REV --> LIMITS
    REV --> DYNAMICS
    
    style CONT fill:#4CAF50
    style FIXED fill:#2196F3
    style REV fill:#FF9800
```

## ros2_control Configuration

```mermaid
graph TB
    subgraph "ros2_control Tag"
        direction TB
        
        HW[hardware<br/>name: RobotSystem]
        
        subgraph "Plugin"
            PLUGIN[plugin: robot_hardware_interfaces/RobotSystem]
        end
        
        subgraph "Parameters"
            PARAMS[connection_timeout_ms<br/>connection_check_period_ms<br/>wheel_radius<br/>wheel_base<br/>velocity_command_joint_order]
        end
        
        subgraph "Joints"
            direction LR
            J1[front_left_wheel_joint<br/>command: velocity<br/>state: position, velocity]
            J2[front_right_wheel_joint<br/>command: velocity<br/>state: position, velocity]
            J3[rear_left_wheel_joint<br/>command: velocity<br/>state: position, velocity]
            J4[rear_right_wheel_joint<br/>command: velocity<br/>state: position, velocity]
        end
        
        subgraph "Sensors"
            IMU[imu_sensor<br/>state: orientation, angular_velocity, linear_acceleration]
        end
        
        HW --> PLUGIN
        HW --> PARAMS
        HW --> J1
        HW --> J2
        HW --> J3
        HW --> J4
        HW --> IMU
    end
    
    style HW fill:#9C27B0
    style PLUGIN fill:#4CAF50
```

## Coordinate Frames (TF Tree)

```mermaid
graph TB
    ODOM[odom<br/>Odometry Frame]
    BF[base_footprint<br/>Ground Projection]
    BL[base_link<br/>Robot Center]
    
    subgraph "Wheels"
        FLW[front_left_wheel_link]
        FRW[front_right_wheel_link]
        RLW[rear_left_wheel_link]
        RRW[rear_right_wheel_link]
    end
    
    subgraph "Sensors"
        IL[imu_link]
        LL[laser_link]
        CL[camera_link]
        COL[camera_optical_link]
    end
    
    ODOM -->|dynamic| BF
    BF -->|static| BL
    BL -->|static| FLW
    BL -->|static| FRW
    BL -->|static| RLW
    BL -->|static| RRW
    BL -->|static| IL
    BL -->|static| LL
    BL -->|static| CL
    CL -->|static| COL
    
    style ODOM fill:#4CAF50
    style BL fill:#2196F3
```

## Gazebo Integration

```mermaid
graph TB
    subgraph "Gazebo Plugins"
        direction TB
        
        RC_PLUGIN[gazebo_ros2_control<br/>Hardware Interface]
        
        subgraph "Sensor Plugins"
            LIDAR_PLUGIN[gazebo_ros_ray_sensor<br/>LiDAR]
            CAM_PLUGIN[gazebo_ros_camera<br/>Camera]
            IMU_PLUGIN[gazebo_ros_imu_sensor<br/>IMU]
        end
    end
    
    subgraph "ROS2 Topics"
        SCAN[/scan<br/>LaserScan]
        IMG[/camera/image_raw<br/>Image]
        IMU_DATA[/imu/data<br/>Imu]
    end
    
    RC_PLUGIN -->|simulates| HWI[Hardware Interface]
    LIDAR_PLUGIN --> SCAN
    CAM_PLUGIN --> IMG
    IMU_PLUGIN --> IMU_DATA
    
    style RC_PLUGIN fill:#4CAF50
    style LIDAR_PLUGIN fill:#2196F3
    style CAM_PLUGIN fill:#FF9800
    style IMU_PLUGIN fill:#9C27B0
```

## Component Configuration Flow

```mermaid
flowchart LR
    subgraph "Input"
        YAML[components_config.yaml<br/>Component List]
    end
    
    subgraph "Processing"
        XACRO[Xacro Processing<br/>Conditional Includes]
    end
    
    subgraph "Components"
        direction TB
        CAM{camera?}
        LIDAR{lidar?}
        IMU{imu?}
        MANIP{manipulator?}
    end
    
    subgraph "Output"
        URDF[Final URDF<br/>With Selected Components]
    end
    
    YAML --> XACRO
    XACRO --> CAM
    XACRO --> LIDAR
    XACRO --> IMU
    XACRO --> MANIP
    
    CAM -->|yes| URDF
    LIDAR -->|yes| URDF
    IMU -->|yes| URDF
    MANIP -->|yes| URDF
    
    style YAML fill:#4CAF50
    style URDF fill:#2196F3
```

## Physical Properties

### Robot Dimensions (robot_xl)

```mermaid
graph TB
    subgraph "Top View"
        direction LR
        
        BASE[Base: 270mm x 170mm]
        
        subgraph "Wheel Positions"
            FL[Front Left<br/>+135mm, +85mm]
            FR[Front Right<br/>+135mm, -85mm]
            RL[Rear Left<br/>-135mm, +85mm]
            RR[Rear Right<br/>-135mm, -85mm]
        end
    end
    
    subgraph "Side View"
        HEIGHT[Height: ~150mm]
        WHEEL_R[Wheel Radius: 47mm]
        CLEARANCE[Ground Clearance: ~20mm]
    end
    
    style BASE fill:#4CAF50
```

### Mass Properties

| Component | Mass (kg) | Inertia |
|-----------|-----------|---------|
| Base Link | 2.5 | Calculated from box |
| Wheel (each) | 0.1 | Calculated from cylinder |
| IMU | 0.01 | Negligible |
| Camera | 0.05 | Negligible |
| LiDAR | 0.2 | Calculated from cylinder |

## Xacro Macros

```mermaid
graph TB
    subgraph "Reusable Macros"
        direction TB
        
        WHEEL[wheel_macro<br/>Creates wheel link + joint]
        SENSOR[sensor_macro<br/>Creates sensor mount]
        INERTIA[inertial_macro<br/>Calculates inertia]
    end
    
    subgraph "Parameters"
        direction TB
        
        WP[Wheel Parameters<br/>radius, width, position]
        SP[Sensor Parameters<br/>position, orientation]
        IP[Inertial Parameters<br/>mass, dimensions]
    end
    
    subgraph "Generated Elements"
        direction TB
        
        LINKS[Links<br/>Visual, Collision, Inertial]
        JOINTS[Joints<br/>Parent, Child, Axis]
    end
    
    WP --> WHEEL
    SP --> SENSOR
    IP --> INERTIA
    
    WHEEL --> LINKS
    WHEEL --> JOINTS
    SENSOR --> LINKS
    SENSOR --> JOINTS
    INERTIA --> LINKS
    
    style WHEEL fill:#4CAF50
    style SENSOR fill:#2196F3
    style INERTIA fill:#FF9800
```

## URDF Validation

```mermaid
flowchart TD
    START[URDF Source Files] --> XACRO[Xacro Processing]
    
    XACRO --> CHECK1{Valid XML?}
    CHECK1 -->|No| ERROR1[Error: XML Syntax]
    CHECK1 -->|Yes| CHECK2{Valid URDF?}
    
    CHECK2 -->|No| ERROR2[Error: URDF Schema]
    CHECK2 -->|Yes| CHECK3{TF Tree Valid?}
    
    CHECK3 -->|No| ERROR3[Error: TF Loops/Disconnected]
    CHECK3 -->|Yes| CHECK4{Collision Geometry?}
    
    CHECK4 -->|No| WARN1[Warning: No Collision]
    CHECK4 -->|Yes| CHECK5{Inertial Properties?}
    
    CHECK5 -->|No| WARN2[Warning: No Inertia]
    CHECK5 -->|Yes| VALID[Valid URDF]
    
    WARN1 --> VALID
    WARN2 --> VALID
    
    VALID --> OUTPUT[robot_description Parameter]
    
    style ERROR1 fill:#f44336
    style ERROR2 fill:#f44336
    style ERROR3 fill:#f44336
    style WARN1 fill:#FF9800
    style WARN2 fill:#FF9800
    style VALID fill:#4CAF50
```

## Usage Examples

### Load URDF
```bash
ros2 launch robot_description load_urdf.launch.py \
  robot_model:=robot_xl \
  components_config:=$(ros2 pkg prefix robot_description)/config/robot_xl/basic.yaml
```

### Visualize in RViz
```bash
ros2 launch robot_description view_robot.launch.py
```

### Check URDF
```bash
check_urdf robot.urdf
```

### View TF Tree
```bash
ros2 run tf2_tools view_frames
```

### Generate URDF from Xacro
```bash
xacro robot.urdf.xacro robot_model:=robot_xl > robot.urdf
```

## Configuration Files

### components_config.yaml

**Purpose**: Define which components to include in robot model

**Example**:
```yaml
components:
  - type: camera
    enabled: true
    position: [0.1, 0.0, 0.15]
  
  - type: lidar
    enabled: true
    position: [0.0, 0.0, 0.2]
  
  - type: imu
    enabled: true
    position: [0.0, 0.0, 0.05]
  
  - type: manipulator
    enabled: false
```

## Dependencies

- **xacro**: URDF macro processing
- **robot_state_publisher**: TF tree publishing
- **joint_state_publisher**: Joint state management (for testing)
- **urdf**: URDF parsing and validation
- **husarion_components_description**: Component xacro files (optional)

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| URDF fails to load | Syntax error in xacro | Check xacro syntax, validate XML |
| TF tree disconnected | Missing joint | Check all links have parent joints |
| Robot appears wrong in RViz | Wrong mesh paths | Verify mesh file paths are correct |
| Gazebo crashes | Invalid inertial properties | Check mass and inertia values |
| Controllers fail | Wrong joint names | Verify joint names match controller config |
| No visualization | Missing meshes | Check mesh files exist in package |
