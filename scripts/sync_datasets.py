import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

def sync_datasets(gps_csv, other_csvs, output_hdf5='synced.h5'):
    """
    Implements GPS-based timestamp alignment across streams (e.g., LiDAR 25 Hz, Starlink 1 Hz).
    Uses interpolation for sub-ms precision, per paper Section 3.2.
    """
    gps_df = pd.read_csv(gps_csv)  # Columns: timestamp (ms), lat, lon, alt
    synced_data = {'timestamp': gps_df['timestamp'].values}
    
    for csv_path in other_csvs:
        df = pd.read_csv(csv_path)
        for col in df.columns[1:]:  # Interpolate each metric to GPS timestamps
            if len(df) > 1:  # Avoid single-point errors
                interp_func = interp1d(df['timestamp'], df[col], kind='linear', fill_value='extrapolate')
                synced_data[col] = interp_func(gps_df['timestamp'])
            else:
                synced_data[col] = np.full(len(gps_df), df[col].iloc[0])  # Constant if single value
    
    with h5py.File(output_hdf5, 'w') as hf:
        for key, val in synced_data.items():
            hf.create_dataset(key, data=val, compression='gzip')  # Compress for large datasets
    
    print(f'Synced {len(other_csvs)} CSVs to {output_hdf5}. Aligned entries: {len(gps_df)}')

# Usage: sync_datasets('gps.csv', ['rtt.csv', 'lidar.csv', 'leoviz.csv'])