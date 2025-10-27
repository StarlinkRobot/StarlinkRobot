import json
import pandas as pd
from scipy.interpolate import interp1d

def parse_leoviz(json_path, output_csv='leoviz_parsed.csv', align_with_metrics=None):
    """
    Parses LEOViz JSON logs (1Hz: satellite_id, azimuth deg, elevation deg, SNR dB).
    Aligns with other metrics via timestamp interpolation (response Revision 3).
    """
    with open(json_path, 'r') as f:
        data = json.load(f)  # List of dicts per timestamp
    
    df = pd.DataFrame(data)
    df['timestamp'] = pd.to_datetime(df['timestamp']).astype(int) / 10**9  # To Unix sec
    
    if align_with_metrics:
        metrics_df = pd.read_csv(align_with_metrics)
        aligned_dfs = []
        for col in df.columns[1:]:
            if len(df) > 1:
                interp_func = interp1d(df['timestamp'], df[col], kind='nearest', fill_value='extrapolate')
                aligned_dfs.append(interp_func(metrics_df['timestamp']))
            else:
                aligned_dfs.append(np.full(len(metrics_df), df[col].iloc[0]))
        aligned_df = pd.DataFrame(dict(zip(df.columns[1:], aligned_dfs)))
        df = pd.concat([metrics_df['timestamp'], aligned_df], axis=1)
    
    df.to_csv(output_csv, index=False)
    print(f'Parsed {json_path} to {output_csv}. Entries: {len(df)}. Aligned with metrics if provided.')

# Usage: parse_leoviz('leoviz_log.json', align_with_metrics='rtt.csv')