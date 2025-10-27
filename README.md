# StarlinkRobot/StarlinkRobot

## 🤖 Starlink Robot

Thank you for your interest in Starlink Robot! 🌟

We’re excited to share our progress with you.

📍 The project homepage has moved to:

[🌐 https://starlinkrobot.github.io/](https://starlinkrobot.github.io/)

Includes the video, datasets, and paper.

## About

The Starlink Robot Dataset

# Enhanced Overview
The Starlink Robot is a mobile platform based on the Unitree Go2 wheeled version, integrating Starlink Mini for satellite connectivity, Livox Mid-360 LiDAR for 3D mapping, an upward-facing fisheye camera for sky visibility, and motion sensors via the robot's internal SLAM. It captures synchronized multi-modal data to study satellite communication under motion and occlusion, as detailed in the SenSys 2026 paper. The dataset includes over 7 hours of data from London urban environments, with 25K+ RTT measurements, 630K LiDAR frames, and 378K fisheye images, expanded with high-speed and late-afternoon sessions for diversity.

- **Collaborators**: Ongoing with Norwegian University of Science and Technology (NTNU) and University of Virginia for geographical diversity.
- **Dataset Access**: Hosted on the companion site (add your link here).
- **Video Demonstration**: Available on the companion site (add your link here).

## Installation
1. Clone the repo: `git clone https://github.com/StarlinkRobot/StarlinkRobot.git`
2. Install Python dependencies: `pip install -r requirements.txt`
3. Install ROS Noetic on Ubuntu 18.04 for bag parsing (as per paper Section 3.2).
4. For LEOViz: Clone from https://github.com/clarkzjw/LEOViz and install its dependencies (Python, gRPC).
5. Hardware requirements: Unitree Go2-W (dimensions 70x43x50cm, payload 8-12kg, weight 18kg), Starlink Mini (weight 1.10kg, IP67, -30°C to 50°C operating temp), Livox Mid-360 (65x65x60mm, 6.5W, 905nm laser).

## Usage Instructions
Detailed steps for hardware setup, data collection, and analysis, based on paper Section 3 (System Design).

### Hardware Setup
- Assemble the platform on Unitree Go2 wheeled base (differential drive, velocity 0.1-2.0 m/s, payload ≈8kg).
- Mount Starlink Mini (weight 1.10kg) on 3D-printed support to maintain upward orientation and reduce vibration.
- Connect Starlink Mini to Intel NUC via Ethernet; use DC-DC converter for power (robot 24V to Starlink 12V).
- Install Livox Mid-360 LiDAR (360° x 59° FOV, 200,000 points/s, 25 Hz custom rate per paper, 0.05m accuracy) and fisheye camera (185° FOV, 15 Hz, 1920x1080 resolution) upward-facing.
- No separate IMU/GPS: Rely on robot's internal SLAM estimator and Starlink's built-in localization for motion data.
- Power: Platform draws 60-110W (base) + 20-40W (Starlink Mini, 15W idle), supporting ~2 hours operation.

### Data Collection Steps
1. **Power On**: Activate Unitree Go2 and Intel NUC (Ubuntu 18.04). Ensure battery is charged for 2-hour endurance.
2. **Start ROS**: Run `roscore` and source workspace. Launch sensor drivers: `roslaunch livox_ros_driver livox_lidar.launch` for LiDAR (25 Hz), and camera node for fisheye (15 Hz).
3. **Start SLAM**: Initialize Unitree internal SLAM via SDK for odometry, velocity, and position tracking (sub-ms alignment via NUC clock).
4. **Start LEOViz**: Execute `./scripts/run_leoviz.sh` to query Starlink gRPC at 1Hz for satellite tracking (positions, elevation, SNR). Enable JSON logging.
5. **Network Probing**: Run tools like ping for RTT (20-400ms range), iperf for throughput, logging to CSV with timestamps.
6. **Data Logging**: Use `rosbag record -a` for raw sensors (LiDAR point clouds, camera images, SLAM odom). Log Starlink/LEOViz to separate CSVs.
7. **Mobility Execution**: Control robot via joystick or SDK for scenarios (steady 0.1-2.0 m/s, variable speeds, open/tree-urban paths). Collect under diverse conditions like high-speed (up to 2 m/s) and occlusions.
8. **Post-Processing**: Parse bags with `./scripts/parse_rosbag.py`, sync via `./scripts/sync_datasets.py` (GPS-based alignment), visualize with `./scripts/visualize_metrics.py` to reproduce paper figures (e.g., RTT vs. velocity).
9. **LEOViz Parsing**: Use `./examples/leoviz_output_parser.py` to align JSON logs with metrics.

For dataset details, see [docs/data_structure.md](docs/data_structure.md) and [docs/file_formats.md](docs/file_formats.md). For LEOViz, see [docs/leoviz_integration.md](docs/leoviz_integration.md).

## 📊 Visualization Tool: LEOViz

This work uses the visualization tool [LEOViz](https://github.com/clarkzjw/LEOViz), developed by Jinwei Zhang from the Pan Lab, led by Prof. Jianping Pan at the University of Victoria.

🙏 We sincerely acknowledge their contribution.

🧑‍🤝‍🧑 We are planning to work together in future collaborations to further enhance the Starlink Robot project.

## 🚧 Project Status

Due to the large volume of data, we are continuously improving the system and dataset.

## License
MIT License. Acknowledgments: LEOViz developers; Unitree, Livox, Starlink vendors.