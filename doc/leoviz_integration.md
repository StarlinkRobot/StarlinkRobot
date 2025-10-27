# LEOViz Integration

From paper Section 3.2/response Revision 3. LEOViz[](https://github.com/clarkzjw/LEOViz) parses Starlink gRPC for 1Hz satellite data.

## Setup
1. Clone LEOViz.
2. Install: Python, gRPCio. Configure Starlink IP (local Ethernet, paper Section 3.1).
3. Enable logging: JSON for positions, azimuth/elevation (deg), SNR (dB), connection status.

## Configuration
- Query: 1Hz for real-time visualization/logging.
- Integration: Runs on NUC alongside ROS; aligns with GPS time for sync.
- Pipeline: Launch with `run_leoviz.sh`, parse JSON via `leoviz_output_parser.py` to CSV, sync to HDF5.

Supports precise synchronization between constellation geometry and metrics (e.g., handover detection, paper Section 6).