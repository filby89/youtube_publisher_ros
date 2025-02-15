# YouTube Publisher ROS2 Package

This ROS2 package, **youtube_publisher_ros2**, publishes video frames from a YouTube link as `sensor_msgs/Image`. It uses the `youtube_dl` package to extract the video URL from a provided link, opens the stream using OpenCV, and publishes the frames at 30 Hz. The YouTube link can be configured via a parameter (`youtube_link`).

## Repository Structure

```
.
├── package.xml                 # Package manifest file.
├── setup.py                    # Python package setup file for ament_python.
├── youtube_publisher_node.py   # Main ROS2 node that extracts the video URL, opens the stream, and publishes frames.
└── launch/
    └── youtube_publisher_launch.py   # Launch file for starting the node with default parameters.
```

## Features

- Extracts the video URL from a provided YouTube link using `youtube_dl`.
- Opens the video stream with OpenCV.
- Publishes video frames on the `/rgb/image_raw` topic at 30 Hz.
- Configurable via the `youtube_link` parameter.

## Prerequisites

- ROS2 (e.g., Foxy, Galactic, or Humble)
- Python 3.x
- OpenCV (`python3-opencv`)
- Required Python packages:
  - `rclpy`
  - `sensor_msgs`
  - `cv_bridge`
  - `youtube_dl`

These dependencies are expected to be installed as part of your ROS2 environment or via pip.

## Installation

1. **Clone/Copy the Package:**  
   Place the `youtube_publisher_ros2` folder into the `src` directory of your ROS2 workspace.

2. **Build the Workspace:**  
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select youtube_publisher_ros2
   ```

3. **Source the Workspace:**  
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Running the Node

To launch the YouTube Publisher node with the default YouTube link, run:

```bash
ros2 launch youtube_publisher_ros2 youtube_publisher_launch.py
```

You can override the default YouTube link by passing a parameter on the command line. For example:

```bash
ros2 launch youtube_publisher_ros2 youtube_publisher_launch.py youtube_link:=https://www.youtube.com/watch?v=YOUR_VIDEO_ID
```

## Parameter Details

- **youtube_link**:  
  A string parameter specifying the YouTube link to extract the video stream from.  
  **Default value:** `https://www.youtube.com/watch?v=SAanKlcupmI`

## License

This package is licensed under the Apache 2.0 License.

## Maintainer

- Panagiotis Filntisis (@filby89)
