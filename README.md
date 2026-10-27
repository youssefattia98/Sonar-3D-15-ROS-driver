# Sonar 3D-15 ROS Driver

## Overview

This package provides a ROS 2 driver for the **Water Linked Sonar 3D-15**, a real-time multibeam imaging sonar. The sonar streams 3D range images over UDP multicast, which are decoded and published as standard ROS messages:

- 3D point clouds: `sensor_msgs/PointCloud2` on `/sonar_point_cloud`, with
  `x`, `y`, `z`, and `intensity` fields stamped at the sonar acquisition time
- Raw internal IMU samples: `sensor_msgs/Imu` on `/sonr/imu/data_raw`, stamped per sample

The driver listens to RIP1 multicast packets, extracts and parses `RangeImage` protobuf messages, and converts the sonar data into formats usable by standard ROS visualization and processing tools.

---

## Features

- Receives and decodes **RIP1/RIP2** packets via UDP multicast
- Publishes point clouds at real-time rates using the sensor acquisition timestamp
- Enables and publishes the sonar's 100 Hz `ImuBatch` output in ROS FLU coordinates
- Publishes the static `sonar_frame` to `sonar_imu_frame` transform
- Does not advertise or publish a 2D sonar image topic
- Automatically enables sonar acoustics and udp multicast on startup
- Compatible with ROS 2 (tested on **Jazzy**)

---

## Topics

| Topic               | Message Type              | Description                        |
|--------------------|---------------------------|------------------------------------|
| `/sonar_point_cloud` | `sensor_msgs/PointCloud2` | 3D point cloud in ROS frame with 8-bit logarithmic signal strength in the floating-point `intensity` field |
| `/sonr/imu/data_raw` | `sensor_msgs/Imu` | Raw specific force and rate of turn at 100 Hz |

The internal IMU is located at `(-0.022, +0.046, +0.003)` metres from the
point-cloud origin after converting Water Linked FRD coordinates to ROS FLU.
The raw packet has no orientation estimate, so the ROS message marks orientation
as unavailable.

---

## Usage

### 1. Clone the repository into your ros2 workspace

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/waterlinked/Sonar-3D-15-ROS-driver.git
```

### 2. Install requirements

Install requirements:

```bash
pip install -r requirements.txt
```


### 3. Set IP-address of Sonar 3D-15

Change to your sonars IP-address in the sonar3d.launch.py file:

```python
    {'IP': '192.168.194.96'},  # Change to your sonar IP, '192.168.194.96' is the fallback ip.
```
Alternatively, you can modify the default parameter in `multicast_listener.py` directly.

```python
    self.declare_parameter('IP', '192.168.194.96')#  <-- your sonar's IP here, '192.168.194.96' is the fallback ip.
```

### 4. Build the package from the root of your ros project

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sonar3d
source install/local_setup.bash
```

### 5. Run the package

```bash
ros2 launch sonar3d sonar3d.launch.py
```

### 6. License

This package is distributed under the MIT License.
