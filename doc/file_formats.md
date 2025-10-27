# File Formats and Data Fields

From paper Section 4/response Revision 2. Includes units and modalities.

## ROS Bags (.bag)
- Topics: `/livox/lidar` (point clouds: x,y,z in meters, 200,000 points/s), `/fisheye/image` (raw bytes, 1920x1080), `/slam/odom` (twist: linear velocity m/s, angular rad/s).
- Fields: Timestamp (sec/nsec), data blobs. No separate IMU/GPS—integrated via SLAM/Starlink.

## CSVs (.csv)
- Columns: timestamp (Unix ms), rtt (ms, 20-400 range), snr (dB), rssi (dBm), velocity (m/s, 0.1-2.0), acceleration (m/s²), position (lat/lon deg, alt m WGS84), obstruction_state (0-1, from fisheye sky %).
- Units: Time ms, signal dB/dBm, motion m/s/m/s². LEOViz adds: satellite_id, azimuth/elevation (deg), signal_quality (0-1).

## HDF5 (.h5)
- Groups: `/communication` (RTT ms mean/95% CI per response Revision 6, throughput Mbps), `/motion` (6-DoF IMU from SLAM, velocity/accel), `/environment` (LiDAR points as Nx3 arrays, sky_visibility % from fisheye).
- Fields: All synced at sub-ms. Specs: LiDAR accuracy 0.05m (paper), detection 40-70m (Livox site); camera 185° FOV.

Full schema supports correlation: motion-aware networking, occlusion prediction (paper Section 6).