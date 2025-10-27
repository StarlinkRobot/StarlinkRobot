# Starlink Robot: A Platform and Dataset for Mobile Satellite Communication

[![GitHub Pages](https://img.shields.io/badge/GitHub-Pages-blue)](https://starlinkrobot.github.io) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository is the companion site for the SenSys 2026 paper: "The Starlink Robot: A Platform and Dataset for Mobile Satellite Communication". It hosts the dataset (via external link), analysis code, video demonstration, and technical documentation. The platform integrates a Unitree Go2 wheeled robot with Starlink Mini, Livox Mid-360 LiDAR, and fisheye camera for studying mobile satellite communication, as detailed in the paper's hardware (Section 3.1) and software architecture (Section 3.2).

## Overview
The Starlink Robot captures synchronized multi-modal data: communication metrics (e.g., RTT, throughput, SNR from Starlink gRPC via LEOViz), motion dynamics (from robot SLAM), environmental context (LiDAR point clouds, fisheye sky images), and satellite tracking. The initial dataset includes 7 hours from London urban environments, with expansions for high-speed mobility and occlusions (per response file Planned Revision 5). It contains over 25K RTT measurements, 630K LiDAR frames, and 378K fisheye images across open areas, tree-covered paths, and urban settings at speeds 0.1-2.0 m/s.

## Installation
1. Clone the repo: `git clone https://github.com/StarlinkRobot/StarlinkRobot.git`
2. Install Python dependencies: `pip install -r requirements.txt`
3. For LEOViz: Clone from https://github.com/clarkzjw/LEOViz; install Docker for containerized runs (per LEOViz docs).
4. ROS Noetic on Ubuntu 18.04: Follow https://wiki.ros.org/noetic/Installation (paper's computing platform).
5. Unitree Go2 ROS Drivers: Clone https://github.com/alexlin2/unitree_go2_ros for ROS1 interface (WebRTC for commands, publishes /slam/odom, /camera, /lidar topics).
6. Livox ROS Driver: Clone https://github.com/Livox-SDK/livox_ros_driver2; build for Mid-360 support.

## Usage Instructions
Detailed steps for hardware setup, data collection, and analysis, based on paper Section 3 and response file implementations.

### Hardware Setup (From Paper Section 3.1)
- Mount Starlink Mini on 3D-printed support (replaces aluminum frame) atop Unitree Go2 wheeled base (15 kg payload, differential drive).
- Connect Starlink to Intel NUC via Ethernet; power via DC-DC converter (24V robot to 12V Starlink).
- Install Livox Mid-360 LiDAR (360° FoV, 25 Hz, 0.05 m accuracy) and fisheye camera (185° FoV, 15 Hz, 1920x1080) upward-facing.
- Use robot's internal SLAM for motion/localization (no separate IMU/GPS; sync via NUC clock).
- Power: 60-110W base + 20-40W Starlink; ~2 hours endurance per deployment.

### Data Collection Steps (Enriched from Paper Section 3.2 and LEOViz/Unitree/Livox Sources)
1. **Power On**: Activate Unitree Go2 and NUC. Ensure battery charged for 2-hour sessions (paper endurance).
2. **Start ROS Noetic**: `roscore` then `source devel/setup.bash` (from Unitree ROS setup).
3. **Start Unitree SLAM/Drivers**: Launch from cloned repo: `roslaunch unitree_go2_ros go2_driver.launch` (publishes /slam/odom for velocity/position, based on https://github.com/alexlin2/unitree_go2_ros).
4. **Start Sensors**: 
   - LiDAR: `roslaunch livox_ros_driver2 msg_MID360_launch.launch` (from https://github.com/Livox-SDK/livox_ros_driver2; publishes /livox/lidar point clouds).
   - Camera: Launch fisheye driver (e.g., `roslaunch usb_cam usb_cam.launch` if using standard ROS cam; publishes /fisheye/image).
5. **Start LEOViz**: Use Docker for gRPC queries (1Hz satellite data): `./scripts/run_leoviz.sh --mobile` (logs to CSV/Parquet; from LEOViz repo).
6. **Start Data Logging**: 
   - ROS bags: `rosbag record -a -O data.bag` (all topics: LiDAR, camera, SLAM).
   - Network probes: Run ping for RTT (e.g., `ping -i 0.01 8.8.8.8 > rtt_log.txt`); log Starlink metrics via LEOViz.
7. **Mobility Control**: Use joystick or SDK for velocity (0.1-2.0 m/s via sport commands from Unitree ROS). Collect in diverse scenarios (open sky, trees, high-speed; per response file Revision 5).
8. **Post-Collection Processing**: 
   - Parse: `./scripts/parse_rosbag.py data.bag`
   - Sync: `./scripts/sync_datasets.py gps.csv [other_csvs]`
   - Visualize: `./scripts/visualize_metrics.py synced.h5` (reproduces paper figures like RTT vs. velocity).

For data access/schema: See [docs/data_structure.md](docs/data_structure.md) and [docs/file_formats.md](docs/file_formats.md).  
For LEOViz: See [docs/leoviz_integration.md](docs/leoviz_integration.md).

## Reproducing Paper Figures (From Response File Revision 6)
Use `./scripts/visualize_metrics.py` on parsed HDF5 to plot RTT means/confidence intervals vs. velocity/obstructions (e.g., Figs. 9,10,12,15 in paper).

## Original Content (Retained)
Thank you for your interest in Starlink Robot! 🌟

We’re excited to share our progress with you.

📍 The project homepage has moved to:

[🌐 https://starlinkrobot.github.io/](https://starlinkrobot.github.io/)

Includes the video, datasets, and paper.

## Visualization Tool: LEOViz

This work uses the visualization tool [LEOViz](https://github.com/clarkzjw/LEOViz), developed by Jinwei Zhang from the Pan Lab, led by Prof. Jianping Pan at the University of Victoria.

🙏 We sincerely acknowledge their contribution.

🧑‍🤝‍🧑 We are planning to work together in future collaborations to further enhance the Starlink Robot project.

## Project Status

Due to the large volume of data, we are continuously improving the system and dataset.

📚 Documentation, 🧑‍💻 source code, and 🛠️ usage instructions will be provided in the next update.

## License
MIT License. Acknowledgments: LEOViz developers; Unitree Robotics; Livox SDK.