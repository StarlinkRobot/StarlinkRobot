# Dataset Structure and Organization

Based on paper Section 4 and response Planned Revision 2. The dataset is structured for reproducibility, with raw ROS bags for sensors, CSVs for processed metrics, and HDF5 for synchronized analysis.

## Directory Hierarchy
- `dataset/`
  - `sessions/` : Named by date/location/condition (e.g., `2025-10-25_london_open-sky_high-speed/`, expanded for late-afternoon and occlusion diversity per response Revision 5).
    - `raw/` : ROS bags (e.g., `lidar_20251025_1400.bag` for Livox point clouds at 25 Hz; `camera_20251025_1400.bag` for fisheye images at 15 Hz; `slam_20251025_1400.bag` for motion).
    - `processed/` : CSVs (e.g., `rtt_metrics.csv` for RTT/throughput; `satellite_tracking.csv` from LEOViz at 1Hz).
    - `synchronized/` : HDF5 files (e.g., `aligned_data.h5` consolidating all modalities with sub-ms alignment via NUC clock).
  - `metadata/` : JSON per session (e.g., duration 7+ hours total, conditions: velocity 0.1-2.0 m/s, occlusions from trees/urban).

## Naming Conventions
- Files: `<modality>_<session_id>_<timestamp>.<ext>` (e.g., `leoviz_20251025_1400.json` for satellite data).
- Sessions: `<YYYY-MM-DD>_<location>_<condition>_<speed>` (e.g., tree-occlusion_high-speed for expanded data).

## Mappings
- ROS → CSV: Extract via `parse_rosbag.py` (timestamps to Unix ms, aligned to system clock per paper Section 3.2).
- CSV → HDF5: Sync via `sync_datasets.py` (GPS interpolation for heterogeneous streams: LiDAR 25 Hz, camera 15 Hz, Starlink 1 Hz).
- LEOViz JSON: Mapped to communication groups in HDF5 (e.g., satellite positions, elevation angles).
- Total: 25K+ RTT, 630K LiDAR frames, 378K images across open/tree/urban at varying speeds (paper Abstract).