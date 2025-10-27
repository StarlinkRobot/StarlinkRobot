#!/bin/bash

# Launch LEOViz with logging enabled for 1Hz satellite data (paper Section 3.2)
# Assumes LEOViz cloned to /path/to/LEOViz; adjust as needed
# Config: Starlink IP via Ethernet (192.168.100.1 default)

LEOVIZ_PATH="/path/to/LEOViz"  # Update this
STARLINK_IP="192.168.100.1"    # From paper: Local Ethernet connection
LOG_FILE="leoviz_log_$(date +%Y%m%d_%H%M%S).json"

cd $LEOVIZ_PATH
python main.py --starlink-ip $STARLINK_IP --log-output $LOG_FILE --interval 1 --visualize real-time
echo "LEOViz launched: Querying at 1Hz, logging to $LOG_FILE. Includes azimuth/elevation/SNR."