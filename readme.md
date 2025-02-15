# YouTube Publisher for ROS1 and ROS2

This repository provides two packages for publishing video frames from a YouTube link as `sensor_msgs/Image`. It uses the `youtube_dl` Python library to extract the direct video stream URL, then opens the stream with OpenCV and publishes frames at 30 Hz.

- **ROS1** package: `youtube_publisher_ros1`
- **ROS2** package: `youtube_publisher_ros2`

Both packages let you configure the YouTube link via a parameter (ROS1 `_youtube_link`, ROS2 `youtube_link`). By default, they publish frames on the `/rgb/image_raw` topic.

---

## 1. ROS1: youtube_publisher_ros1

### Overview
- Located in the `youtube_publisher_ros1` folder.
- Publishes frames at 30 Hz on `/rgb/image_raw`.
- Uses `rospy` and `cv_bridge`.

### Prerequisites
- ROS1 (e.g., ROS Noetic)
- Python 2.7 or 3.x (matching your ROS distribution)
- `cv_bridge`, `sensor_msgs`
- `youtube_dl` (install via pip, e.g. `pip install youtube_dl`)

### Installation and Usage

1. **Copy or symlink** the `youtube_publisher_ros1` folder into your ROS workspace’s `src` directory.
2. **Build** the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
3. **Source** your workspace:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```
4. **Run** the node:
   ```bash
   rosrun youtube_publisher_ros1 youtube_publisher_node.py _youtube_link:=https://www.youtube.com/watch?v=YOUR_VIDEO_ID
   ```
   By default, the node publishes frames on `/rgb/image_raw`. You can view them via tools like `rqt_image_view` or `rviz`.

---

## 2. ROS2: youtube_publisher_ros2

### Overview
- Located in the `youtube_publisher_ros2` folder.
- Publishes frames at 30 Hz on `/rgb/image_raw`.
- Uses `rclpy` and `cv_bridge`.

### Prerequisites
- ROS2 (e.g., Foxy, Galactic, or Humble)
- Python 3.x
- `rclpy`, `sensor_msgs`, `cv_bridge` installed in your ROS2 environment
- `youtube_dl` (install via pip, e.g. `pip install youtube_dl`)

### Installation and Usage

1. **Place** the `youtube_publisher_ros2` folder in your ROS2 workspace’s `src` directory.
2. **Build** the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select youtube_publisher_ros2
   ```
3. **Source** your workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
4. **Launch** the node with the default YouTube link:
   ```bash
   ros2 launch youtube_publisher_ros2 youtube_publisher_launch.py
   ```
5. **Override** the default link on the command line:
   ```bash
   ros2 launch youtube_publisher_ros2 youtube_publisher_launch.py youtube_link:=https://www.youtube.com/watch?v=YOUR_VIDEO_ID
   ```

You can view the published images (by default on `/rgb/image_raw`) with ROS2 visualization tools or `rqt`.

---

## Parameter Details

- **ROS1**: Pass the link as `_youtube_link:=your_link`.  
- **ROS2**: Pass the link as a parameter `youtube_link:=your_link`.

---

## Maintainer

Developed and maintained by **Panagiotis Filntisis** (@filby89).  
License: **Apache 2.0**  
