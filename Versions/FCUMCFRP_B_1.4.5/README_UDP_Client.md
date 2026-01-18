# UDP Data Client for FCU IMU Telemetry

Python client for receiving, visualizing, and logging IMU telemetry data from the FCU Madgwick Control Filter Research Platform.

## Features

- **UDP Packet Reception**: Receives 112-byte IMU data packets at 100Hz (10ms intervals)
- **Live Plotting**: Real-time visualization of:
  - Attitude comparison (Control vs Monitor IMU)
  - Attitude errors
  - Accelerometer data
  - Gyroscope data
- **CSV Logging**: Automatically saves all received data to timestamped CSV files
- **Timestamped Sessions**: Each run creates a new folder with timestamp for data organization

## Requirements

Install dependencies:
```bash
pip install -r requirements.txt
```

Required packages:
- `numpy` >= 1.21.0
- `matplotlib` >= 3.5.0

## Configuration

The script uses the following UDP settings (must match `quad.h`):
- **Local Port**: 8888 (receiving)
- **Remote Port**: 8889 (Arduino sends to this)
- **Remote IP**: 192.168.1.100 (configured in `quad.h`)

To change these settings, edit the constants at the top of `UDPDataClient.py`:
```python
UDP_LOCAL_PORT = 8888
```

## Usage

1. **Ensure Arduino/Teensy is running** with `USE_MPU9250_MONITOR_I2C` enabled
2. **Verify network connectivity** - Arduino and PC must be on same network
3. **Run the client**:
   ```bash
   python UDPDataClient.py
   ```

4. **View live plots** - A matplotlib window will open showing:
   - Top-left: Attitude comparison (Roll, Pitch, Yaw)
   - Top-right: Attitude errors
   - Bottom-left: Accelerometer data
   - Bottom-right: Gyroscope data

5. **Stop recording** - Press `Ctrl+C` to gracefully shutdown

## Output Structure

Data is saved in the `data_logs/` directory:
```
data_logs/
└── YYYYMMDD_HHMMSS/          # Timestamped session folder
    └── imu_data.csv          # Complete dataset
```

## CSV Data Format

The CSV file contains the following columns (in order):

**Timestamp:**
- `timestamp_us` - Arduino timestamp in microseconds

**Control IMU Raw Sensors (MPU6050):**
- `ctrl_acc_x`, `ctrl_acc_y`, `ctrl_acc_z` - Accelerometer (g)
- `ctrl_gyro_x`, `ctrl_gyro_y`, `ctrl_gyro_z` - Gyroscope (deg/s)
- `ctrl_mag_x`, `ctrl_mag_y`, `ctrl_mag_z` - Magnetometer (µT)

**Monitor IMU Raw Sensors (MPU9250):**
- `mon_acc_x`, `mon_acc_y`, `mon_acc_z` - Accelerometer (g)
- `mon_gyro_x`, `mon_gyro_y`, `mon_gyro_z` - Gyroscope (deg/s)
- `mon_mag_x`, `mon_mag_y`, `mon_mag_z` - Magnetometer (µT)

**Control IMU Attitude (Madgwick Filter):**
- `ctrl_roll`, `ctrl_pitch`, `ctrl_yaw` - Euler angles (deg)

**Monitor IMU Attitude (Madgwick Filter):**
- `mon_roll`, `mon_pitch`, `mon_yaw` - Euler angles (deg)

**Attitude Comparison Errors:**
- `err_roll`, `err_pitch`, `err_yaw` - Control - Monitor (deg)

## Troubleshooting

**No data received:**
- Check that Arduino is powered and running
- Verify network connection (ping Arduino IP)
- Check firewall settings (UDP port 8888)
- Ensure `USE_MPU9250_MONITOR_I2C` is enabled in `quad.h`

**Incorrect packet size:**
- Verify Arduino firmware matches expected packet structure
- Check for network packet fragmentation (shouldn't occur for 112 bytes)

**Plotting issues:**
- Ensure matplotlib backend supports GUI (TkAgg, Qt5Agg)
- On headless systems, use `matplotlib.use('Agg')` and save plots instead

## Notes

- Data is buffered in memory (last 1000 points) for plotting
- CSV files are flushed after each packet for data safety
- Plot updates at ~20Hz (50ms intervals) for smooth visualization
- X-axis shows last 30 seconds of data (rolling window)
