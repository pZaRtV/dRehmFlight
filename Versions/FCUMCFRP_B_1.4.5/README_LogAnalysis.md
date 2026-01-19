# Log Analysis Tool for FCU IMU Telemetry

Post-processing analysis tool for saved IMU telemetry CSV log files.

## Features

- **Interactive Log Selection**: Browse and select from timestamped log directories
- **Comprehensive Statistics**: Detailed statistical analysis of all sensor data
- **Stability Validation Metrics**:
  - Mean Absolute Error (MAE)
  - Root Mean Square Error (RMSE)
  - Settling Time (2% and 5% tolerance)
- **Visual Analysis**: Same 4-panel plot layout as live client:
  - Attitude comparison (Control vs Monitor)
  - Attitude error analysis with statistics and settling time markers
  - Accelerometer data comparison
  - Gyroscope data comparison
- **Auto-Save**: Automatically saves plots and stability reports

## Requirements

Install dependencies:
```bash
pip install -r requirements.txt
```

Required packages:
- `numpy` >= 1.21.0
- `matplotlib` >= 3.5.0
- `pandas` >= 1.3.0

## Usage

1. **Run the analysis tool**:
   ```bash
   python logAnalysis.py
   ```

2. **Select a log file**:
   - The tool will list all available log files in `data_logs/`
   - Each entry shows: timestamp, number of rows, file size
   - Enter the number corresponding to the log file you want to analyze
   - Or press 'q' to quit

3. **View results**:
   - Statistical summary is printed to console
   - Stability validation metrics are displayed
   - Interactive plots are displayed
   - Plot and stability report are automatically saved in the log directory

## Output

### Console Statistics

The tool prints comprehensive statistics including:
- **Attitude Statistics**: Mean, std dev, min, max for Control and Monitor IMU
- **Attitude Errors**: Mean, std dev, and maximum absolute error
- **Sensor Statistics**: Mean and std dev for accelerometer and gyroscope data
- **Stability Validation Metrics**:
  - Mean Absolute Error (MAE) for each axis
  - Root Mean Square Error (RMSE) for each axis
  - Settling Time at 2% and 5% tolerance levels

### Plot Output

Four-panel plot showing:
1. **Attitude Comparison**: Control vs Monitor IMU attitude (Roll, Pitch, Yaw)
2. **Attitude Error**: Difference between Control and Monitor with error statistics overlay
3. **Accelerometer Data**: Control vs Monitor accelerometer readings
4. **Gyroscope Data**: Control vs Monitor gyroscope readings

### Saved Files

Two files are automatically saved in the log directory:
1. **`analysis_plot.png`**: Comprehensive 4-panel visualization
2. **`stability_report.txt`**: Detailed stability metrics report

The plot includes:
- Tolerance bands (2% and 5%) on the error plot
- Settling time markers (vertical dashed lines)
- Enhanced statistics overlay with MAE and RMSE values

## Example Output

```
======================================================================
FCU IMU Telemetry - Log Analysis Tool
======================================================================

======================================================================
Available Log Files:
======================================================================
  [ 1] 2026-01-18 19:52:29 |   1234 rows | 0.15 MB
  [ 2] 2026-01-18 15:30:45 |   5678 rows | 0.68 MB
  [ 3] 2026-01-18 12:15:10 |   9012 rows | 1.10 MB
======================================================================

Select log file [1-3] (or 'q' to quit): 1

Selected: 2026-01-18 19:52:29 (1234 rows)

Loading data from: data_logs/20260118_195229/imu_data.csv
Loaded 1234 data points
Time span: 12.34 seconds

======================================================================
Data Statistics:
======================================================================
...
```

## Stability Metrics Explained

- **Mean Absolute Error (MAE)**: Average magnitude of errors, provides a measure of typical error size
- **Root Mean Square Error (RMSE)**: Square root of mean squared errors, penalizes larger errors more heavily
- **Settling Time**: Time required for the error to reach and remain within a specified tolerance band (2% or 5% of maximum error)

These metrics help validate:
- Control system stability
- IMU calibration accuracy
- Filter performance
- System response characteristics

## Notes

- Log files are sorted by timestamp (newest first)
- Only directories containing `imu_data.csv` are listed
- Time axis is automatically calculated from Arduino timestamps
- All plots use the same color scheme as the live client for consistency
- Error statistics and stability metrics are displayed directly on the attitude error plot
- Settling time is calculated based on a rolling window to ensure sustained stability
