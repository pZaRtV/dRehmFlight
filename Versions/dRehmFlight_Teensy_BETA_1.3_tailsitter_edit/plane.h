// DRehmFlight Teensy Flight Controller Tailsitter Specific Config
// Author  : Pat

#ifndef TAILSHIT_XIANYING_H
#define TAILSHIT_XIANYING_H

//USER CONFIGURATION//
// Select ONE receiver type:
// #define USE_PWM_RX
#define USE_PPM_RX
// #define USE_SBUS_RX
// #define USE_DSM_RX
// #define USE_IBUS_RX

// If using DSM RX, set the number of channels:
#define NUM_DSM_CHANNELS 6

// Select ONE IMU:
#define USE_MPU6050_I2C // Default
// #define USE_MPU9250_SPI

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
// If your PPM receiver channel order differs from the sketch expectation
// (CH1=Throttle, CH2=Aileron, CH3=Elevator, CH4=Rudder, CH5=Gear, CH6=Aux1),
// set these to the PPM slot numbers (1-based) that correspond to each logical channel.
#define PPM_MAP_CH1 3 // throttle
#define PPM_MAP_CH2 1 // aileron
#define PPM_MAP_CH3 2 // elevator
#define PPM_MAP_CH4 4 // rudder
#define PPM_MAP_CH5 5 // gear / throttle cut
#define PPM_MAP_CH6 6 // aux1 / transition

//END USER CONFIGURATION//

// ========== Sanity checks (only one option per group) ========= //
#if (defined(USE_PWM_RX) + defined(USE_PPM_RX) + defined(USE_SBUS_RX) + defined(USE_DSM_RX) + defined(USE_IBUS_RX)) != 1
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

#endif // DREHMFLIGHT_CONFIG_H 