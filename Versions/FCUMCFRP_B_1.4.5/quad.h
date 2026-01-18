
// DRehmFlight Teensy Flight Controller Quad Specific Config
// Author         : Patrick Andrasena T.
// Project Start  : 8/21/2025
// Last Updated   : 10/15/2025
// Version        : Beta 1.1 

#ifndef FC_QUAD_H
#define FC_QUAD_H

//USER CONFIGURATION//
// Select ONE receiver type:
// #define USE_PWM_RX
// #define USE_PPM_RX
// #define USE_HYBRID_RX //(PPM + RC1 PWM passthrough for throttle response, nifty clusterfck)
// #define USE_SBUS_RX
// #define USE_DSM_RX
// #define USE_IBUS_RX
#define USE_ELRS_RX

// If using DSM RX, set the number of channels:
// #define NUM_DSM_CHANNELS 6

// Select ONE primary IMU (used by control loop):
#define USE_MPU6050_I2C // Default
// #define USE_MPU9250_SPI

// Optional: enable secondary MPU9250 IMU over I2C (Wire1) as ground-truth / orientation monitor.
// Purpose: Validate control system response by comparing MPU6050 (control) vs MPU9250 (monitor) attitude estimates.
// The monitor IMU runs its own Madgwick filter independently to provide a second opinion on vehicle orientation.
// This allows onboard validation of the control IMU's attitude estimates during flight.
// This sensor is read in parallel and does NOT affect the control loop.
// Uses Wire1 (SDA1=RX4=pin 16, SCL1=TX4=pin 17) with I2C address 0x69, clock speed 400kHz.
// Note: Wire1 is separate from Wire (used by MPU6050 at 1000kHz), allowing independent operation.
// ELRS moved to Serial3 to free RX4/TX4 for Wire1.
#define USE_MPU9250_MONITOR_I2C

// Ethernet/W5500 Configuration for UDP telemetry (used by monitor IMU)
// W5500 uses SPI - specify which SPI bus and CS pin
#define W5500_CS_PIN 10              // Chip select pin for W5500 (default SPI CS)
#define W5500_SPI_BUS SPI            // Use default SPI bus (SCK=13, MISO=12, MOSI=11)
#define UDP_LOCAL_PORT 8888          // Local UDP port for receiving (if needed)
#define UDP_REMOTE_PORT 8889         // Remote UDP port for sending telemetry
// Remote IP address for telemetry (change as needed) - defined as byte array for initialization
#define UDP_REMOTE_IP_0 192
#define UDP_REMOTE_IP_1 168
#define UDP_REMOTE_IP_2 1
#define UDP_REMOTE_IP_3 100
#define MONITOR_UDP_RATE_MS 10       // Send UDP packets every 10ms (100Hz)

// Monitor comparison mode:
// - If defined: Compare attitude angles (requires Madgwick filter on monitor IMU) - RECOMMENDED for control validation
// - If undefined: Compare raw sensor data only (no Madgwick filter, simpler/lighter) - for sensor calibration only
#define USE_MONITOR_ATTITUDE_COMPARISON  // Keep enabled to validate control IMU attitude estimates

// Select ONE full scale gyro range (deg/sec):
#define GYRO_250DPS // Default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Select ONE full scale accelerometer range (G's):
#define ACCEL_2G // Default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

// #define USE_ONESHOT125_ESC // Uncomment this line if you want to use OneShot125 ESCs, otherwise generic PWM ESCs will be used.

// Optional: PPM channel remapping (logical -> physical index in PPM frame)
// If your PPM receiver channel order differs from the sketch expectation:
// (CH1=Throttle, CH2=Aileron, CH3=Elevator, CH4=Rudder, CH5=Gear, CH6=Aux1),
// set these to the PPM slot numbers (1-based) that correspond to each logical channel.
#define PPM_MAP_CH1 3 // throttle
#define PPM_MAP_CH2 1 // aileron
#define PPM_MAP_CH3 2 // elevator
#define PPM_MAP_CH4 4 // rudder
#define PPM_MAP_CH5 5 // gear / throttle cut
#define PPM_MAP_CH6 6 // aux1 / transition

// #define USE_PWM_PASSTHROUGH_CH1

// PWM (1:1 still if wired accordingly)
#define PWM_MAP_CH1 1 // throttle
#define PWM_MAP_CH2 2 // aileron
#define PWM_MAP_CH3 3 // elevator
#define PWM_MAP_CH4 4 // rudder
#define PWM_MAP_CH5 5 // gear / throttle cut
#define PWM_MAP_CH6 6 // aux1 / transition

// ELRS Channel Mapping (maps logical channels to physical CRSF channel indices, 1-based)
// Default is 1:1 mapping, but can be changed if your transmitter sends channels in different order
#define ELRS_MAP_CH1 1 // throttle
#define ELRS_MAP_CH2 2 // aileron
#define ELRS_MAP_CH3 3 // elevator
#define ELRS_MAP_CH4 4 // rudder
#define ELRS_MAP_CH5 5 // gear / throttle cut
#define ELRS_MAP_CH6 6 // aux1 / transition

// SBUS Channel Mapping (maps logical channels to physical SBUS channel indices, 0-based array)
// Default is 1:1 mapping (logical CH1 -> sbusChannels[0], etc.)
#define SBUS_MAP_CH1 1 // throttle (maps to sbusChannels[0])
#define SBUS_MAP_CH2 2 // aileron (maps to sbusChannels[1])
#define SBUS_MAP_CH3 3 // elevator (maps to sbusChannels[2])
#define SBUS_MAP_CH4 4 // rudder (maps to sbusChannels[3])
#define SBUS_MAP_CH5 5 // gear / throttle cut (maps to sbusChannels[4])
#define SBUS_MAP_CH6 6 // aux1 / transition (maps to sbusChannels[5])

// DSM Channel Mapping (maps logical channels to physical DSM channel indices, 0-based array)
// Default is 1:1 mapping (logical CH1 -> values[0], etc.)
#define DSM_MAP_CH1 1 // throttle (maps to values[0])
#define DSM_MAP_CH2 2 // aileron (maps to values[1])
#define DSM_MAP_CH3 3 // elevator (maps to values[2])
#define DSM_MAP_CH4 4 // rudder (maps to values[3])
#define DSM_MAP_CH5 5 // gear / throttle cut (maps to values[4])
#define DSM_MAP_CH6 6 // aux1 / transition (maps to values[5])

// ELRS Specific Configuration
#define ELRS_UART_PORT 3           // Serial3 (RX3=pin 15, TX3=pin 14) - Same as DSM, but only one RX type active at a time
#define ELRS_BAUD_RATE 420000      // CRSF baud
#define ELRS_TIMEOUT_MS 100        // Failsafe timeout
#define ELRS_CHANNELS 6            // Number of active channels (1-16)

// Hybrid (ch3 direct pwm for throttle)
// #define HYBRID_MAP_CH1 3
// ✅ NEW: Enable PWM passthrough for specific channel
// Uncomment to use direct PWM input for throttle instead of PPM
#define USE_PWM_PASSTHROUGH_CH1  // Use PWM on ch1Pin for throttle (faster response)

// If using PWM passthrough, specify which physical pin
// (This overrides the PPM mapping for that channel)
#ifdef USE_PWM_PASSTHROUGH_CH1
  // ch1Pin (pin 15) will be used for direct PWM throttle input
  // PPM_MAP_CH1 will be ignored
#endif

// Pin definitions for PWM and PPM receivers (shared across all .ino files)
//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
//Radio:
//Note: If using SBUS, connect to pin 21 (RX5), if using DSM, connect to pin 15 (RX3)
#if defined USE_PWM_RX || defined USE_PPM_RX || defined USE_PWM_PASSTHROUGH_CH1
  const int ch1Pin = 15; //throttle
  const int ch2Pin = 16; //ail
  const int ch3Pin = 17; //ele
  const int ch4Pin = 20; //rudd
  const int ch5Pin = 21; //gear (throttle cut)
  const int ch6Pin = 22; //aux1 (free aux channel)
#endif

#if defined USE_PPM_RX || defined USE_HYBRID_RX
  const int PPM_Pin = 23;
#endif

//END USER CONFIGURATION//

// ========== Sanity checks (only one option per group) ========= //
#if (defined(USE_PWM_RX) + defined(USE_PPM_RX) + defined(USE_SBUS_RX) + defined(USE_DSM_RX) + defined(USE_IBUS_RX) + defined(USE_HYBRID_RX) + defined (USE_ELRS_RX)) != 1
#error "You must define exactly one receiver type (USE_PWM_RX, USE_PPM_RX, USE_SBUS_RX, USE_DSM_RX, USE_IBUS_RX) in plane.h."
#endif

#if (defined(USE_MPU6050_I2C) + defined(USE_MPU9250_SPI)) != 1
#error "You must define exactly one IMU (USE_MPU6050_I2C or USE_MPU9250_SPI) in plane.h."
#endif

#if (defined(GYRO_250DPS) + defined(GYRO_500DPS) + defined(GYRO_1000DPS) + defined(GYRO_2000DPS)) != 1
#error "You must define exactly one gyro range in plane.h."
#endif

#if (defined(ACCEL_2G) + defined(ACCEL_4G) + defined(ACCEL_8G) + defined(ACCEL_16G)) != 1
#error "You must define exactly one accelerometer range in plane.h."
#endif

#endif // FCQUAD_CONFIG_H 