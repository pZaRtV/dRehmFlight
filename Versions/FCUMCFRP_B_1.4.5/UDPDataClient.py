"""

UDP Data Client for FCU Madgwick Control Filter Research Platform
Receives IMU telemetry data via UDP and creates live plots + CSV datasets

Author: Patrick Andrasena T.
Version: 1.0

"""

import socket
import struct
import csv
import os
from datetime import datetime
import time
import threading
import queue
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# UDP Configuration (must match quad.h settings)
UDP_LOCAL_PORT = 8888  # Port to listen on
UDP_BUFFER_SIZE = 1500  # Standard UDP MTU

# Packet structure (matches IMUDataPacket in imuMonitor.ino)
# Format: '<I' (uint32_t timestamp) + '27f' (27 floats)
# Total: 4 + (27 * 4) = 112 bytes
PACKET_FORMAT = '<I27f'  # Little-endian, 1 uint32, 27 floats
PACKET_SIZE = 112  # bytes

# Data field names (in order of struct definition)
FIELD_NAMES = [
    'timestamp_us',
    # Control IMU raw sensors (MPU6050)
    'ctrl_acc_x', 'ctrl_acc_y', 'ctrl_acc_z',
    'ctrl_gyro_x', 'ctrl_gyro_y', 'ctrl_gyro_z',
    'ctrl_mag_x', 'ctrl_mag_y', 'ctrl_mag_z',
    # Monitor IMU raw sensors (MPU9250)
    'mon_acc_x', 'mon_acc_y', 'mon_acc_z',
    'mon_gyro_x', 'mon_gyro_y', 'mon_gyro_z',
    'mon_mag_x', 'mon_mag_y', 'mon_mag_z',
    # Control IMU attitude (from Madgwick filter)
    'ctrl_roll', 'ctrl_pitch', 'ctrl_yaw',
    # Monitor IMU attitude (from Madgwick filter)
    'mon_roll', 'mon_pitch', 'mon_yaw',
    # Attitude comparison errors
    'err_roll', 'err_pitch', 'err_yaw'
]

# Plotting configuration
MAX_DATA_POINTS = 1000  # Number of points to keep in rolling buffer
PLOT_UPDATE_INTERVAL = 50  # ms between plot updates


class UDPDataClient:
    def __init__(self):
        self.sock = None
        self.data_queue = queue.Queue()
        self.running = False
        self.packet_count = 0
        self.start_time = None
        self.first_timestamp_us = None  # Arduino timestamp of first packet
        
        # Create timestamped output directory
        self.session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join("data_logs", self.session_timestamp)
        os.makedirs(self.output_dir, exist_ok=True)
        
        # CSV file paths
        self.csv_file = os.path.join(self.output_dir, "imu_data.csv")
        self.csv_writer = None
        self.csv_file_handle = None
        
        # Data buffers for plotting (rolling buffers)
        self.time_buffer = deque(maxlen=MAX_DATA_POINTS)
        self.data_buffers = {field: deque(maxlen=MAX_DATA_POINTS) for field in FIELD_NAMES[1:]}  # Skip timestamp
        
        # Initialize CSV writer
        self._init_csv()
        
        print(f"Data logging initialized:")
        print(f"  Output directory: {self.output_dir}")
        print(f"  CSV file: {self.csv_file}")
    
    def _init_csv(self):
        """Initialize CSV file with headers"""
        self.csv_file_handle = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file_handle, fieldnames=FIELD_NAMES)
        self.csv_writer.writeheader()
        self.csv_file_handle.flush()
    
    def start(self):
        """Start UDP listener and data processing"""
        try:
            # Create UDP socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('', UDP_LOCAL_PORT))
            self.sock.settimeout(1.0)  # 1 second timeout for graceful shutdown
            
            self.running = True
            self.start_time = time.time()
            
            print(f"UDP listener started on port {UDP_LOCAL_PORT}")
            print("Waiting for data packets...")
            print("Press Ctrl+C to stop\n")
            
            # Start data processing thread
            processing_thread = threading.Thread(target=self._process_data, daemon=True)
            processing_thread.start()
            
            # Main receive loop
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(UDP_BUFFER_SIZE)
                    
                    if len(data) == PACKET_SIZE:
                        self.data_queue.put((data, addr))
                    else:
                        print(f"Warning: Received packet of size {len(data)} bytes, expected {PACKET_SIZE}")
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"Error receiving data: {e}")
                    
        except KeyboardInterrupt:
            print("\nShutting down...")
        except Exception as e:
            print(f"Error starting UDP client: {e}")
        finally:
            self.stop()
    
    def _process_data(self):
        """Process received packets and write to CSV"""
        while self.running:
            try:
                data, addr = self.data_queue.get(timeout=0.1)
                
                # Unpack packet
                try:
                    unpacked = struct.unpack(PACKET_FORMAT, data)
                    
                    # Create dictionary from unpacked data
                    packet_dict = dict(zip(FIELD_NAMES, unpacked))
                    
                    # Store first Arduino timestamp as reference
                    if self.first_timestamp_us is None:
                        self.first_timestamp_us = packet_dict['timestamp_us']
                    
                    # Calculate time relative to first packet (in seconds)
                    relative_time_s = (packet_dict['timestamp_us'] - self.first_timestamp_us) / 1e6
                    
                    # Write to CSV (with original Arduino timestamp)
                    self.csv_writer.writerow(packet_dict)
                    self.csv_file_handle.flush()
                    
                    # Update data buffers for plotting (use Arduino timestamp for consistency)
                    self.time_buffer.append(relative_time_s)
                    
                    for field in FIELD_NAMES[1:]:  # Skip timestamp_us
                        self.data_buffers[field].append(packet_dict[field])
                    
                    self.packet_count += 1
                    
                    if self.packet_count % 100 == 0:
                        print(f"Received {self.packet_count} packets...")
                        
                except struct.error as e:
                    print(f"Error unpacking packet: {e}")
                    
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    print(f"Error processing data: {e}")
    
    def stop(self):
        """Stop UDP listener and close files"""
        self.running = False
        
        if self.sock:
            self.sock.close()
        
        if self.csv_file_handle:
            self.csv_file_handle.close()
        
        print(f"\nSession complete:")
        print(f"  Total packets received: {self.packet_count}")
        print(f"  Data saved to: {self.output_dir}")


class LivePlotter:
    def __init__(self, data_client):
        self.data_client = data_client
        self.fig = None
        self.axes = None
        self.lines = {}
        
    def setup_plots(self):
        """Setup matplotlib figure with subplots"""
        # Create figure with 2 rows, 2 columns
        self.fig, self.axes = plt.subplots(2, 2, figsize=(16, 10))
        self.fig.suptitle('FCU IMU Telemetry - Live Data', fontsize=16, fontweight='bold')
        
        # Attitude plots (top row)
        ax1 = self.axes[0, 0]  # Control vs Monitor Attitude
        ax1.set_title('Attitude Comparison (Roll, Pitch, Yaw)')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Angle (deg)')
        ax1.grid(True, alpha=0.3)
        ax1.legend(['Control Roll', 'Control Pitch', 'Control Yaw', 
                    'Monitor Roll', 'Monitor Pitch', 'Monitor Yaw'], 
                   loc='upper right')
        
        # Attitude error plot
        ax2 = self.axes[0, 1]
        ax2.set_title('Attitude Error (Control - Monitor)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error (deg)')
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        
        # Raw sensor plots (bottom row)
        ax3 = self.axes[1, 0]  # Accelerometer
        ax3.set_title('Accelerometer Data (g)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Acceleration (g)')
        ax3.grid(True, alpha=0.3)
        
        ax4 = self.axes[1, 1]  # Gyroscope
        ax4.set_title('Gyroscope Data (deg/s)')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Angular Rate (deg/s)')
        ax4.grid(True, alpha=0.3)
        
        # Initialize plot lines
        # Attitude comparison
        self.lines['ctrl_roll'] = ax1.plot([], [], 'r-', label='Ctrl Roll', linewidth=1.5)[0]
        self.lines['ctrl_pitch'] = ax1.plot([], [], 'g-', label='Ctrl Pitch', linewidth=1.5)[0]
        self.lines['ctrl_yaw'] = ax1.plot([], [], 'b-', label='Ctrl Yaw', linewidth=1.5)[0]
        self.lines['mon_roll'] = ax1.plot([], [], 'r--', label='Mon Roll', linewidth=1.5, alpha=0.7)[0]
        self.lines['mon_pitch'] = ax1.plot([], [], 'g--', label='Mon Pitch', linewidth=1.5, alpha=0.7)[0]
        self.lines['mon_yaw'] = ax1.plot([], [], 'b--', label='Mon Yaw', linewidth=1.5, alpha=0.7)[0]
        
        # Attitude errors
        self.lines['err_roll'] = ax2.plot([], [], 'r-', label='Roll Error', linewidth=1.5)[0]
        self.lines['err_pitch'] = ax2.plot([], [], 'g-', label='Pitch Error', linewidth=1.5)[0]
        self.lines['err_yaw'] = ax2.plot([], [], 'b-', label='Yaw Error', linewidth=1.5)[0]
        
        # Accelerometer
        self.lines['ctrl_acc_x'] = ax3.plot([], [], 'r-', label='Ctrl X', linewidth=1.5)[0]
        self.lines['ctrl_acc_y'] = ax3.plot([], [], 'g-', label='Ctrl Y', linewidth=1.5)[0]
        self.lines['ctrl_acc_z'] = ax3.plot([], [], 'b-', label='Ctrl Z', linewidth=1.5)[0]
        self.lines['mon_acc_x'] = ax3.plot([], [], 'r--', label='Mon X', linewidth=1.5, alpha=0.7)[0]
        self.lines['mon_acc_y'] = ax3.plot([], [], 'g--', label='Mon Y', linewidth=1.5, alpha=0.7)[0]
        self.lines['mon_acc_z'] = ax3.plot([], [], 'b--', label='Mon Z', linewidth=1.5, alpha=0.7)[0]
        
        # Gyroscope
        self.lines['ctrl_gyro_x'] = ax4.plot([], [], 'r-', label='Ctrl X', linewidth=1.5)[0]
        self.lines['ctrl_gyro_y'] = ax4.plot([], [], 'g-', label='Ctrl Y', linewidth=1.5)[0]
        self.lines['ctrl_gyro_z'] = ax4.plot([], [], 'b-', label='Ctrl Z', linewidth=1.5)[0]
        self.lines['mon_gyro_x'] = ax4.plot([], [], 'r--', label='Mon X', linewidth=1.5, alpha=0.7)[0]
        self.lines['mon_gyro_y'] = ax4.plot([], [], 'g--', label='Mon Y', linewidth=1.5, alpha=0.7)[0]
        self.lines['mon_gyro_z'] = ax4.plot([], [], 'b--', label='Mon Z', linewidth=1.5, alpha=0.7)[0]
        
        # Add legends
        ax1.legend(loc='upper right', fontsize=8)
        ax2.legend(loc='upper right', fontsize=8)
        ax3.legend(loc='upper right', fontsize=8)
        ax4.legend(loc='upper right', fontsize=8)
        
        plt.tight_layout()
    
    def update_plots(self, frame):
        """Update plot data"""
        if not self.data_client.running or len(self.data_client.time_buffer) == 0:
            return self.lines.values()
        
        # Get current data
        time_data = np.array(self.data_client.time_buffer)
        
        # Update attitude plots
        if len(time_data) > 0:
            self.lines['ctrl_roll'].set_data(time_data, self.data_client.data_buffers['ctrl_roll'])
            self.lines['ctrl_pitch'].set_data(time_data, self.data_client.data_buffers['ctrl_pitch'])
            self.lines['ctrl_yaw'].set_data(time_data, self.data_client.data_buffers['ctrl_yaw'])
            self.lines['mon_roll'].set_data(time_data, self.data_client.data_buffers['mon_roll'])
            self.lines['mon_pitch'].set_data(time_data, self.data_client.data_buffers['mon_pitch'])
            self.lines['mon_yaw'].set_data(time_data, self.data_client.data_buffers['mon_yaw'])
            
            # Update error plots
            self.lines['err_roll'].set_data(time_data, self.data_client.data_buffers['err_roll'])
            self.lines['err_pitch'].set_data(time_data, self.data_client.data_buffers['err_pitch'])
            self.lines['err_yaw'].set_data(time_data, self.data_client.data_buffers['err_yaw'])
            
            # Update accelerometer plots
            self.lines['ctrl_acc_x'].set_data(time_data, self.data_client.data_buffers['ctrl_acc_x'])
            self.lines['ctrl_acc_y'].set_data(time_data, self.data_client.data_buffers['ctrl_acc_y'])
            self.lines['ctrl_acc_z'].set_data(time_data, self.data_client.data_buffers['ctrl_acc_z'])
            self.lines['mon_acc_x'].set_data(time_data, self.data_client.data_buffers['mon_acc_x'])
            self.lines['mon_acc_y'].set_data(time_data, self.data_client.data_buffers['mon_acc_y'])
            self.lines['mon_acc_z'].set_data(time_data, self.data_client.data_buffers['mon_acc_z'])
            
            # Update gyroscope plots
            self.lines['ctrl_gyro_x'].set_data(time_data, self.data_client.data_buffers['ctrl_gyro_x'])
            self.lines['ctrl_gyro_y'].set_data(time_data, self.data_client.data_buffers['ctrl_gyro_y'])
            self.lines['ctrl_gyro_z'].set_data(time_data, self.data_client.data_buffers['ctrl_gyro_z'])
            self.lines['mon_gyro_x'].set_data(time_data, self.data_client.data_buffers['mon_gyro_x'])
            self.lines['mon_gyro_y'].set_data(time_data, self.data_client.data_buffers['mon_gyro_y'])
            self.lines['mon_gyro_z'].set_data(time_data, self.data_client.data_buffers['mon_gyro_z'])
            
            # Auto-scale axes
            for ax in self.axes.flat:
                ax.relim()
                ax.autoscale_view()
            
            # Set x-axis limits to show recent data
            if len(time_data) > 0:
                x_min = max(0, time_data[-1] - 30)  # Show last 30 seconds
                x_max = time_data[-1] + 1
                for ax in self.axes.flat:
                    ax.set_xlim(x_min, x_max)
        
        return self.lines.values()
    
    def start_animation(self):
        """Start matplotlib animation"""
        self.setup_plots()
        self.ani = animation.FuncAnimation(
            self.fig, 
            self.update_plots, 
            interval=PLOT_UPDATE_INTERVAL,
            blit=False,
            cache_frame_data=False
        )
        plt.show()


def main():
    """Main function"""
    print("=" * 60)
    print("FCU IMU Telemetry UDP Client")
    print("=" * 60)
    
    # Create data client
    data_client = UDPDataClient()
    
    # Create plotter
    plotter = LivePlotter(data_client)
    
    # Start plotting in separate thread
    plot_thread = threading.Thread(target=plotter.start_animation, daemon=True)
    plot_thread.start()
    
    # Give plotter time to initialize
    time.sleep(1)
    
    # Start UDP listener (blocking)
    try:
        data_client.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        data_client.stop()
        plt.close('all')


if __name__ == "__main__":
    main()
