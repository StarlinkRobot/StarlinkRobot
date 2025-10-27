import rosbag
import pandas as pd
import h5py
import numpy as np
from scipy.interpolate import interp1d  # For minor alignments if needed

def parse_rosbag(bag_path, output_csv='parsed.csv', output_hdf5='parsed.h5'):
    """
    Parses ROS bag files to extract synchronized measurements into CSV/HDF5.
    Handles LiDAR (25 Hz), camera (15 Hz), SLAM odom per paper Section 3.2.
    """
    bag = rosbag.Bag(bag_path)
    data = {'timestamp': [], 'lidar_points': [], 'camera_img': [], 'slam_velocity': [], 'slam_position': []}
    
    for topic, msg, t in bag.read_messages(topics=['/livox/lidar', '/fisheye/image', '/slam/odom']):
        ts = t.to_sec()  # Unified system clock timestamp
        data['timestamp'].append(ts)
        if topic == '/livox/lidar':
            # Extract point cloud (Nx3: x,y,z in meters, accuracy 0.05m)
            points = np.array([[p.x, p.y, p.z] for p in msg.points])  # 200,000 points/s
            data['lidar_points'].append(points)
        elif topic == '/fisheye/image':
            # Raw image data (1920x1080, for sky visibility %)
            data['camera_img'].append(np.frombuffer(msg.data, dtype=np.uint8))
        elif topic == '/slam/odom':
            # Motion from Unitree SLAM (velocity m/s, position lat/lon/alt)
            data['slam_velocity'].append(msg.twist.twist.linear.x)
            data['slam_position'].append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    
    df = pd.DataFrame(data)
    df.to_csv(output_csv, index=False)
    
    with h5py.File(output_hdf5, 'w') as hf:
        hf.create_dataset('timestamps', data=df['timestamp'].values)
        # Add compressed datasets for large point clouds
        for col in ['lidar_points', 'camera_img']:  # Example; handle large data
            hf.create_dataset(col, data=df[col].values, compression='gzip')
    
    bag.close()
    print(f'Parsed {bag_path} to {output_csv} and {output_hdf5}. Total entries: {len(df)}')

# Usage: parse_rosbag('path/to/bag.bag')