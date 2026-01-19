"""
Log Analysis Tool for FCU IMU Telemetry Data
Analyzes saved CSV log files and generates comprehensive plots:
Measured stability metrics:
1. Mean Absolute Error (MAE)
2. Root Mean Square Error (RMSE)
3. Settling Time (2% and 5% tolerance)

Author: Patrick Andrasena T.
Version: 1.0
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# Configuration
DATA_LOGS_DIR = "data_logs"
CSV_FILENAME = "imu_data.csv"


def scan_log_directories():
    """Scan data_logs directory for timestamped folders"""
    if not os.path.exists(DATA_LOGS_DIR):
        print(f"Error: Directory '{DATA_LOGS_DIR}' not found.")
        return []
    
    # Find all timestamped directories (format: YYYYMMDD_HHMMSS)
    log_dirs = []
    for item in os.listdir(DATA_LOGS_DIR):
        item_path = os.path.join(DATA_LOGS_DIR, item)
        if os.path.isdir(item_path):
            csv_path = os.path.join(item_path, CSV_FILENAME)
            if os.path.exists(csv_path):
                log_dirs.append(item)
    
    # Sort by timestamp (newest first)
    log_dirs.sort(reverse=True)
    return log_dirs


def list_log_files():
    """List available log files with details"""
    log_dirs = scan_log_directories()
    
    if not log_dirs:
        print("No log files found in data_logs directory.")
        return None
    
    print("\n" + "=" * 70)
    print("Available Log Files:")
    print("=" * 70)
    
    log_info = []
    for idx, log_dir in enumerate(log_dirs, 1):
        csv_path = os.path.join(DATA_LOGS_DIR, log_dir, CSV_FILENAME)
        
        # Get file stats
        try:
            file_size = os.path.getsize(csv_path)
            file_size_mb = file_size / (1024 * 1024)
            
            # Count lines in CSV
            with open(csv_path, 'r') as f:
                line_count = sum(1 for line in f) - 1  # Subtract header
            
            # Parse timestamp
            try:
                dt = datetime.strptime(log_dir, "%Y%m%d_%H%M%S")
                timestamp_str = dt.strftime("%Y-%m-%d %H:%M:%S")
            except:
                timestamp_str = log_dir
            
            log_info.append({
                'index': idx,
                'directory': log_dir,
                'path': csv_path,
                'timestamp': timestamp_str,
                'size_mb': file_size_mb,
                'rows': line_count
            })
            
            print(f"  [{idx:2d}] {timestamp_str} | {line_count:6d} rows | {file_size_mb:.2f} MB")
            
        except Exception as e:
            print(f"  [{idx:2d}] {log_dir} | Error reading file: {e}")
    
    print("=" * 70)
    return log_info


def select_log_file():
    """Interactive log file selection"""
    log_info = list_log_files()
    
    if not log_info:
        return None
    
    while True:
        try:
            choice = input(f"\nSelect log file [1-{len(log_info)}] (or 'q' to quit): ").strip()
            
            if choice.lower() == 'q':
                return None
            
            choice_idx = int(choice)
            if 1 <= choice_idx <= len(log_info):
                selected = log_info[choice_idx - 1]
                print(f"\nSelected: {selected['timestamp']} ({selected['rows']} rows)")
                return selected['path']
            else:
                print(f"Invalid choice. Please enter a number between 1 and {len(log_info)}.")
                
        except ValueError:
            print("Invalid input. Please enter a number or 'q' to quit.")
        except KeyboardInterrupt:
            print("\nCancelled.")
            return None


def load_csv_data(csv_path):
    """Load CSV data into pandas DataFrame"""
    try:
        print(f"\nLoading data from: {csv_path}")
        df = pd.read_csv(csv_path)
        
        # Convert timestamp_us to relative time in seconds
        if 'timestamp_us' in df.columns and len(df) > 0:
            first_timestamp = df['timestamp_us'].iloc[0]
            df['time_s'] = (df['timestamp_us'] - first_timestamp) / 1e6
        
        print(f"Loaded {len(df)} data points")
        print(f"Time span: {df['time_s'].iloc[-1]:.2f} seconds")
        
        return df
        
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return None


def create_analysis_plots(df):
    """Create comprehensive analysis plots"""
    if df is None or len(df) == 0:
        print("No data to plot.")
        return
    
    # Create figure with 2 rows, 2 columns (same layout as UDPDataClient)
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle('FCU IMU Telemetry - Log Analysis', fontsize=16, fontweight='bold')
    
    time_data = df['time_s'].values
    
    # Plot 1: Attitude Comparison (top-left)
    ax1 = axes[0, 0]
    ax1.set_title('Attitude Comparison (Roll, Pitch, Yaw)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Time (s)', fontsize=10)
    ax1.set_ylabel('Angle (deg)', fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    ax1.plot(time_data, df['ctrl_roll'], 'r-', label='Control Roll', linewidth=1.5)
    ax1.plot(time_data, df['ctrl_pitch'], 'g-', label='Control Pitch', linewidth=1.5)
    ax1.plot(time_data, df['ctrl_yaw'], 'b-', label='Control Yaw', linewidth=1.5)
    ax1.plot(time_data, df['mon_roll'], 'r--', label='Monitor Roll', linewidth=1.5, alpha=0.7)
    ax1.plot(time_data, df['mon_pitch'], 'g--', label='Monitor Pitch', linewidth=1.5, alpha=0.7)
    ax1.plot(time_data, df['mon_yaw'], 'b--', label='Monitor Yaw', linewidth=1.5, alpha=0.7)
    ax1.legend(loc='upper right', fontsize=8)
    
    # Plot 2: Attitude Error (top-right)
    ax2 = axes[0, 1]
    ax2.set_title('Attitude Error (Control - Monitor)', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Time (s)', fontsize=10)
    ax2.set_ylabel('Error (deg)', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    ax2.plot(time_data, df['err_roll'], 'r-', label='Roll Error', linewidth=1.5)
    ax2.plot(time_data, df['err_pitch'], 'g-', label='Pitch Error', linewidth=1.5)
    ax2.plot(time_data, df['err_yaw'], 'b-', label='Yaw Error', linewidth=1.5)
    ax2.legend(loc='upper right', fontsize=8)
    
    # Calculate and display error statistics
    err_roll_std = df['err_roll'].std()
    err_pitch_std = df['err_pitch'].std()
    err_yaw_std = df['err_yaw'].std()
    err_roll_max = df['err_roll'].abs().max()
    err_pitch_max = df['err_pitch'].abs().max()
    err_yaw_max = df['err_yaw'].abs().max()
    
    # Calculate stability metrics
    metrics = calculate_stability_metrics(df)
    
    stats_text = f"Roll:  σ={err_roll_std:.3f}°, max={err_roll_max:.3f}°\n"
    stats_text += f"       MAE={metrics['roll']['mae']:.3f}°, RMSE={metrics['roll']['rmse']:.3f}°\n"
    stats_text += f"Pitch: σ={err_pitch_std:.3f}°, max={err_pitch_max:.3f}°\n"
    stats_text += f"       MAE={metrics['pitch']['mae']:.3f}°, RMSE={metrics['pitch']['rmse']:.3f}°\n"
    stats_text += f"Yaw:   σ={err_yaw_std:.3f}°, max={err_yaw_max:.3f}°\n"
    stats_text += f"       MAE={metrics['yaw']['mae']:.3f}°, RMSE={metrics['yaw']['rmse']:.3f}°"
    
    # Add settling time if available
    settling_times = []
    for axis in ['roll', 'pitch', 'yaw']:
        st = metrics[axis]['settling_time_2pct']
        if st is not None:
            settling_times.append(f"{axis[0].upper()}:{st:.2f}s")
    
    if settling_times:
        stats_text += f"\n\nSettling (2%): {', '.join(settling_times)}"
    
    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes,
             fontsize=7, verticalalignment='top', bbox=dict(boxstyle='round', 
             facecolor='wheat', alpha=0.5))
    
    # Draw settling time markers on error plot
    tolerance_2pct = 0.02
    tolerance_5pct = 0.05
    for axis, color in [('roll', 'r'), ('pitch', 'g'), ('yaw', 'b')]:
        error_data = df[f'err_{axis}'].values
        max_abs_error = np.abs(error_data).max()
        tol_2pct = tolerance_2pct * max_abs_error
        tol_5pct = tolerance_5pct * max_abs_error
        
        # Draw tolerance bands
        ax2.axhline(y=tol_2pct, color=color, linestyle=':', alpha=0.3, linewidth=1)
        ax2.axhline(y=-tol_2pct, color=color, linestyle=':', alpha=0.3, linewidth=1)
        
        # Mark settling time
        st_2pct = metrics[axis]['settling_time_2pct']
        if st_2pct is not None:
            ax2.axvline(x=st_2pct, color=color, linestyle='--', alpha=0.5, linewidth=1.5)
    
    # Plot 3: Accelerometer Data (bottom-left)
    ax3 = axes[1, 0]
    ax3.set_title('Accelerometer Data (g)', fontsize=12, fontweight='bold')
    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('Acceleration (g)', fontsize=10)
    ax3.grid(True, alpha=0.3)
    
    ax3.plot(time_data, df['ctrl_acc_x'], 'r-', label='Control X', linewidth=1.5)
    ax3.plot(time_data, df['ctrl_acc_y'], 'g-', label='Control Y', linewidth=1.5)
    ax3.plot(time_data, df['ctrl_acc_z'], 'b-', label='Control Z', linewidth=1.5)
    ax3.plot(time_data, df['mon_acc_x'], 'r--', label='Monitor X', linewidth=1.5, alpha=0.7)
    ax3.plot(time_data, df['mon_acc_y'], 'g--', label='Monitor Y', linewidth=1.5, alpha=0.7)
    ax3.plot(time_data, df['mon_acc_z'], 'b--', label='Monitor Z', linewidth=1.5, alpha=0.7)
    ax3.legend(loc='upper right', fontsize=8)
    
    # Plot 4: Gyroscope Data (bottom-right)
    ax4 = axes[1, 1]
    ax4.set_title('Gyroscope Data (deg/s)', fontsize=12, fontweight='bold')
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Angular Rate (deg/s)', fontsize=10)
    ax4.grid(True, alpha=0.3)
    
    ax4.plot(time_data, df['ctrl_gyro_x'], 'r-', label='Control X', linewidth=1.5)
    ax4.plot(time_data, df['ctrl_gyro_y'], 'g-', label='Control Y', linewidth=1.5)
    ax4.plot(time_data, df['ctrl_gyro_z'], 'b-', label='Control Z', linewidth=1.5)
    ax4.plot(time_data, df['mon_gyro_x'], 'r--', label='Monitor X', linewidth=1.5, alpha=0.7)
    ax4.plot(time_data, df['mon_gyro_y'], 'g--', label='Monitor Y', linewidth=1.5, alpha=0.7)
    ax4.plot(time_data, df['mon_gyro_z'], 'b--', label='Monitor Z', linewidth=1.5, alpha=0.7)
    ax4.legend(loc='upper right', fontsize=8)
    
    plt.tight_layout()
    
    return fig


def calculate_mae(series):
    """Calculate Mean Absolute Error"""
    return series.abs().mean()


def calculate_rmse(series):
    """Calculate Root Mean Square Error"""
    return np.sqrt((series ** 2).mean())


def calculate_settling_time(time_data, error_data, tolerance_percent=2.0, window_size=50):
    """
    Calculate settling time - time for error to reach and stay within tolerance
    
    Args:
        time_data: Time array
        error_data: Error signal array
        tolerance_percent: Percentage tolerance (default 2%)
        window_size: Number of samples to check for stability
    
    Returns:
        Settling time in seconds, or None if not settled
    """
    if len(error_data) < window_size:
        return None
    
    # Calculate tolerance based on maximum absolute error
    max_abs_error = np.abs(error_data).max()
    tolerance = tolerance_percent / 100.0 * max_abs_error
    
    # Minimum tolerance to avoid false positives with very small errors
    min_tolerance = 0.01  # degrees
    tolerance = max(tolerance, min_tolerance)
    
    # Find when error enters and stays within tolerance
    # Check if error stays within tolerance for the entire window
    for i in range(len(error_data) - window_size):
        window = error_data[i:i + window_size]
        if np.all(np.abs(window) <= tolerance):
            return time_data[i]
    
    return None


def calculate_stability_metrics(df):
    """Calculate stability validation metrics for attitude errors"""
    time_data = df['time_s'].values
    
    metrics = {}
    
    for axis in ['roll', 'pitch', 'yaw']:
        error_col = f'err_{axis}'
        error_data = df[error_col].values
        
        metrics[axis] = {
            'mae': calculate_mae(df[error_col]),
            'rmse': calculate_rmse(df[error_col]),
            'settling_time_2pct': calculate_settling_time(time_data, error_data, tolerance_percent=2.0),
            'settling_time_5pct': calculate_settling_time(time_data, error_data, tolerance_percent=5.0),
        }
    
    return metrics


def print_data_statistics(df):
    """Print statistical summary of the data"""
    print("\n" + "=" * 70)
    print("Data Statistics:")
    print("=" * 70)
    
    # Attitude statistics
    print("\nAttitude (Control IMU):")
    print(f"  Roll:  mean={df['ctrl_roll'].mean():7.3f}°, std={df['ctrl_roll'].std():7.3f}°, "
          f"min={df['ctrl_roll'].min():7.3f}°, max={df['ctrl_roll'].max():7.3f}°")
    print(f"  Pitch: mean={df['ctrl_pitch'].mean():7.3f}°, std={df['ctrl_pitch'].std():7.3f}°, "
          f"min={df['ctrl_pitch'].min():7.3f}°, max={df['ctrl_pitch'].max():7.3f}°")
    print(f"  Yaw:   mean={df['ctrl_yaw'].mean():7.3f}°, std={df['ctrl_yaw'].std():7.3f}°, "
          f"min={df['ctrl_yaw'].min():7.3f}°, max={df['ctrl_yaw'].max():7.3f}°")
    
    print("\nAttitude (Monitor IMU):")
    print(f"  Roll:  mean={df['mon_roll'].mean():7.3f}°, std={df['mon_roll'].std():7.3f}°, "
          f"min={df['mon_roll'].min():7.3f}°, max={df['mon_roll'].max():7.3f}°")
    print(f"  Pitch: mean={df['mon_pitch'].mean():7.3f}°, std={df['mon_pitch'].std():7.3f}°, "
          f"min={df['mon_pitch'].min():7.3f}°, max={df['mon_pitch'].max():7.3f}°")
    print(f"  Yaw:   mean={df['mon_yaw'].mean():7.3f}°, std={df['mon_yaw'].std():7.3f}°, "
          f"min={df['mon_yaw'].min():7.3f}°, max={df['mon_yaw'].max():7.3f}°")
    
    print("\nAttitude Errors (Control - Monitor):")
    print(f"  Roll:  mean={df['err_roll'].mean():7.3f}°, std={df['err_roll'].std():7.3f}°, "
          f"max_abs={df['err_roll'].abs().max():7.3f}°")
    print(f"  Pitch: mean={df['err_pitch'].mean():7.3f}°, std={df['err_pitch'].std():7.3f}°, "
          f"max_abs={df['err_pitch'].abs().max():7.3f}°")
    print(f"  Yaw:   mean={df['err_yaw'].mean():7.3f}°, std={df['err_yaw'].std():7.3f}°, "
          f"max_abs={df['err_yaw'].abs().max():7.3f}°")
    
    print("\nAccelerometer (Control IMU):")
    print(f"  X: mean={df['ctrl_acc_x'].mean():7.3f}g, std={df['ctrl_acc_x'].std():7.3f}g")
    print(f"  Y: mean={df['ctrl_acc_y'].mean():7.3f}g, std={df['ctrl_acc_y'].std():7.3f}g")
    print(f"  Z: mean={df['ctrl_acc_z'].mean():7.3f}g, std={df['ctrl_acc_z'].std():7.3f}g")
    
    print("\nGyroscope (Control IMU):")
    print(f"  X: mean={df['ctrl_gyro_x'].mean():7.3f}°/s, std={df['ctrl_gyro_x'].std():7.3f}°/s")
    print(f"  Y: mean={df['ctrl_gyro_y'].mean():7.3f}°/s, std={df['ctrl_gyro_y'].std():7.3f}°/s")
    print(f"  Z: mean={df['ctrl_gyro_z'].mean():7.3f}°/s, std={df['ctrl_gyro_z'].std():7.3f}°/s")
    
    print("=" * 70)


def print_stability_metrics(df):
    """Print stability validation metrics"""
    print("\n" + "=" * 70)
    print("Stability Validation Metrics:")
    print("=" * 70)
    
    metrics = calculate_stability_metrics(df)
    
    print("\nMean Absolute Error (MAE):")
    print(f"  Roll:  {metrics['roll']['mae']:7.3f}°")
    print(f"  Pitch: {metrics['pitch']['mae']:7.3f}°")
    print(f"  Yaw:   {metrics['yaw']['mae']:7.3f}°")
    
    print("\nRoot Mean Square Error (RMSE):")
    print(f"  Roll:  {metrics['roll']['rmse']:7.3f}°")
    print(f"  Pitch: {metrics['pitch']['rmse']:7.3f}°")
    print(f"  Yaw:   {metrics['yaw']['rmse']:7.3f}°")
    
    print("\nSettling Time (2% tolerance):")
    for axis in ['roll', 'pitch', 'yaw']:
        st = metrics[axis]['settling_time_2pct']
        if st is not None:
            print(f"  {axis.capitalize():5s}: {st:7.3f} s")
        else:
            print(f"  {axis.capitalize():5s}: Not settled (within 2% tolerance)")
    
    print("\nSettling Time (5% tolerance):")
    for axis in ['roll', 'pitch', 'yaw']:
        st = metrics[axis]['settling_time_5pct']
        if st is not None:
            print(f"  {axis.capitalize():5s}: {st:7.3f} s")
        else:
            print(f"  {axis.capitalize():5s}: Not settled (within 5% tolerance)")
    
    print("=" * 70)
    
    return metrics


def save_plot(fig, csv_path):
    """Save plot to file in same directory as CSV"""
    if fig is None:
        return
    
    csv_dir = os.path.dirname(csv_path)
    plot_path = os.path.join(csv_dir, "analysis_plot.png")
    
    try:
        fig.savefig(plot_path, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved to: {plot_path}")
    except Exception as e:
        print(f"Error saving plot: {e}")


def save_stability_report(csv_path, metrics, df):
    """Save stability metrics to a text file"""
    csv_dir = os.path.dirname(csv_path)
    report_path = os.path.join(csv_dir, "stability_report.txt")
    
    try:
        with open(report_path, 'w') as f:
            f.write("=" * 70 + "\n")
            f.write("Stability Validation Report\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Data file: {os.path.basename(csv_path)}\n")
            f.write(f"Total samples: {len(df)}\n")
            f.write(f"Time span: {df['time_s'].iloc[-1]:.2f} seconds\n\n")
            
            f.write("=" * 70 + "\n")
            f.write("Stability Metrics\n")
            f.write("=" * 70 + "\n\n")
            
            f.write("Mean Absolute Error (MAE):\n")
            for axis in ['roll', 'pitch', 'yaw']:
                f.write(f"  {axis.capitalize():5s}: {metrics[axis]['mae']:7.3f}°\n")
            
            f.write("\nRoot Mean Square Error (RMSE):\n")
            for axis in ['roll', 'pitch', 'yaw']:
                f.write(f"  {axis.capitalize():5s}: {metrics[axis]['rmse']:7.3f}°\n")
            
            f.write("\nSettling Time (2% tolerance):\n")
            for axis in ['roll', 'pitch', 'yaw']:
                st = metrics[axis]['settling_time_2pct']
                if st is not None:
                    f.write(f"  {axis.capitalize():5s}: {st:7.3f} s\n")
                else:
                    f.write(f"  {axis.capitalize():5s}: Not settled\n")
            
            f.write("\nSettling Time (5% tolerance):\n")
            for axis in ['roll', 'pitch', 'yaw']:
                st = metrics[axis]['settling_time_5pct']
                if st is not None:
                    f.write(f"  {axis.capitalize():5s}: {st:7.3f} s\n")
                else:
                    f.write(f"  {axis.capitalize():5s}: Not settled\n")
            
            f.write("\n" + "=" * 70 + "\n")
            f.write("Additional Statistics\n")
            f.write("=" * 70 + "\n\n")
            
            f.write("Attitude Error Statistics:\n")
            for axis in ['roll', 'pitch', 'yaw']:
                err_col = f'err_{axis}'
                f.write(f"  {axis.capitalize():5s}: ")
                f.write(f"mean={df[err_col].mean():7.3f}°, ")
                f.write(f"std={df[err_col].std():7.3f}°, ")
                f.write(f"max_abs={df[err_col].abs().max():7.3f}°\n")
        
        print(f"Stability report saved to: {report_path}")
    except Exception as e:
        print(f"Error saving stability report: {e}")


def main():
    """Main analysis function"""
    print("=" * 70)
    print("FCU IMU Telemetry - Log Analysis Tool")
    print("=" * 70)
    
    # Select log file
    csv_path = select_log_file()
    
    if csv_path is None:
        print("No file selected. Exiting.")
        return
    
    # Load data
    df = load_csv_data(csv_path)
    
    if df is None or len(df) == 0:
        print("No data to analyze.")
        return
    
    # Print statistics
    print_data_statistics(df)
    
    # Print stability metrics
    stability_metrics = print_stability_metrics(df)
    
    # Create plots
    print("\nGenerating plots...")
    fig = create_analysis_plots(df)
    
    if fig is not None:
        # Save plot
        save_plot(fig, csv_path)
        
        # Save stability report
        save_stability_report(csv_path, stability_metrics, df)
        
        # Show plot
        print("\nDisplaying plot. Close the window to exit.")
        plt.show()
    else:
        print("Failed to create plots.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nAnalysis cancelled by user.")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
