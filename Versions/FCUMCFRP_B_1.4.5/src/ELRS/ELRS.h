#ifndef ELRS_H
#define ELRS_H

#include <CRSFforArduino.hpp>
#include <HardwareSerial.h>
#include "Arduino.h"

class ELRSReceiver {
private:
  HardwareSerial* _serial;
  CRSFforArduino _crsf;
  uint8_t _rssi;
  uint8_t _lq;
  unsigned long _lastUpdate;
  
public:
  // Constructor
  ELRSReceiver(HardwareSerial* ser);
  
  // Initialization
  void begin();
  
  // Update (call every loop)
  void update();
  
  // Channel access (1-6 or 1-16)
  uint16_t getChannel(uint8_t ch);
  
  // Telemetry (sending data FROM FC TO transmitter)
  void sendAttitude(float roll, float pitch, float yaw);  // Send IMU attitude in degrees
  void sendBattery(float voltage, float current);         // Send battery telemetry (optional)
  
  // Telemetry (receiving data FROM transmitter TO FC)
  // Note: RSSI/LQ values are estimated from channel data validity
  // Actual telemetry requires implementing CRSF telemetry callbacks
  uint8_t getRSSI();        // Returns 0 (not available via direct API)
  uint8_t getLinkQuality(); // Returns 100 if connected, 0 if disconnected
  bool isConnected();
  
  // Debug
  void printDebug();
};

// Global instance declaration (will be defined in ELRS.cpp)
extern ELRSReceiver elrs_rx;

#endif
