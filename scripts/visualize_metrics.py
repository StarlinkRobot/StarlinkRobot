import pandas as pd
import matplotlib.pyplot as plt
import h5py
import numpy as np

def visualize_metrics(hdf5_path, output_plot='metrics.png'):
    """
    Visualizes communication/motion metrics, reproducing paper figures (e.g., RTT vs. velocity).
    Computes mean/95% CI per response Revision 6.
    """
    with h5py.File(hdf5_path, 'r') as hf:
        timestamps = hf['timestamp'][:]
        rtt = hf['rtt'][:] if 'rtt' in hf else None
        velocity = hf['velocity'][:] if 'velocity' in hf else None
        obstruction = hf['obstruction_state'][:] if 'obstruction_state' in hf else None
    
    if rtt is not None and velocity is not None:
        fig, ax1 = plt.subplots()
        ax1.plot(timestamps, rtt, 'b-', label='RTT (ms)')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('RTT (ms)', color='b')
        
        ax2 = ax1.twinx()
        ax2.plot(timestamps, velocity, 'r-', label='Velocity (m/s)')
        ax2.set_ylabel('Velocity (m/s)', color='r')
        
        if obstruction is not None:
            ax2.plot(timestamps, obstruction, 'g--', label='Obstruction (0-1)')
        
        # Compute stats: Mean RTT with 95% CI
        mean_rtt = np.mean(rtt)
        ci_rtt = 1.96 * np.std(rtt) / np.sqrt(len(rtt))  # 95% CI
        plt.title(f'RTT: {mean_rtt:.2f} ± {ci_rtt:.2f} ms')
        
        fig.legend()
        plt.savefig(output_plot)
        plt.show()
        print(f'Visualized to {output_plot}. Stats: RTT mean {mean_rtt:.2f} ms (95% CI ±{ci_rtt:.2f})')

# Usage: visualize_metrics('synced.h5')