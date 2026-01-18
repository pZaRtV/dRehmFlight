#include "ELRS.h"

// Global instance - instantiated here
// Using Serial3: RX3=pin 15, TX3=pin 14 (moved from Serial4 to free RX4/TX4 for Wire1 I2C monitor)
ELRSReceiver elrs_rx(&Serial3);

// Constructor
ELRSReceiver::ELRSReceiver(HardwareSerial* ser) 
  : _serial(ser), _crsf(ser), _rssi(0), _lq(0), _lastUpdate(0) {}

// Initialize UART and CRSF
void ELRSReceiver::begin() {
  _serial->begin(420000);  // CRSF baud rate
  _crsf.begin();
  _lastUpdate = millis();
}

// Parse incoming CRSF packets
void ELRSReceiver::update() {
  // CRSFforArduino library by Cassandra "ZZ Cat" Robinson
  // Library uses event-driven API for telemetry (RSSI/LQ via callbacks)
  // For now, we use channel data validity as connection indicator
  _crsf.update();  // Library parses CRSF frames and handles telemetry callbacks internally
  
  // Check if we have valid channel data (indicates active connection)
  // CRSF channel range is 988-2012 (with some tolerance for noise)
  // If channels are updating, receiver is connected and linked
  uint16_t test_ch = _crsf.getChannel(0);
  if (test_ch >= 988 && test_ch <= 2012) {  // Valid CRSF channel range
    // We're receiving valid channel data - assume good link
    // Note: Actual RSSI/LQ would require telemetry callback implementation
    // For now, use binary connection state: connected (100%) or disconnected (0%)
    _lq = 100;
    _lastUpdate = millis();
  } else {
    // No valid channel data - likely disconnected or initializing
    _lq = 0;
  }
  
  // RSSI remains at default 0
  // To access actual RSSI/LQ telemetry, would need to implement:
  // - Telemetry callback handler using library's event API
  // - Parse LINK_STATISTICS telemetry frame from receiver
  // See CRSFforArduino documentation for telemetry callback examples
}

// Get channel value (1-16 based indexing)
uint16_t ELRSReceiver::getChannel(uint8_t ch) {
  if (ch < 1 || ch > 16) return 1500;  // Safety check
  
  // CRSF uses 0-based indexing, convert from 1-based
  uint16_t raw = _crsf.getChannel(ch - 1);
  
  // Map CRSF (988-2012) to PWM (1000-2000)
  return constrain(map(raw, 988, 2012, 1000, 2000), 1000, 2000);
}

// Get signal strength (dBm)
// Note: CRSFforArduino uses event-driven telemetry API
// Actual RSSI requires telemetry callback implementation
// Currently returns 0 as placeholder
uint8_t ELRSReceiver::getRSSI() {
  return _rssi;  // Always 0 until telemetry callbacks implemented
}

// Get link quality (0-100%)
// Binary connection state based on channel data validity
// Returns 100% if receiving valid channel data, 0% otherwise
// Note: For actual LQ percentage from receiver, implement telemetry callbacks
uint8_t ELRSReceiver::getLinkQuality() {
  return _lq;  // 100 = connected, 0 = disconnected
}

// Check if connected (LQ > 0 and recent update)
bool ELRSReceiver::isConnected() {
  return (_lq > 0) && ((millis() - _lastUpdate) < 100);  // 100ms timeout
}

// Send attitude telemetry to transmitter
// Parameters in degrees, internally converted to decidegrees (degrees * 10) for CRSF
// This allows monitoring IMU attitude on transmitter/ground station
void ELRSReceiver::sendAttitude(float roll, float pitch, float yaw) {
  // Convert degrees to decidegrees (CRSF protocol requirement)
  // decidegrees = degrees * 10 (provides 0.1 degree resolution)
  int16_t rollDeci = (int16_t)(roll * 10.0f);
  int16_t pitchDeci = (int16_t)(pitch * 10.0f);
  int16_t yawDeci = (int16_t)(yaw * 10.0f);
  
  // Send attitude telemetry via CRSF
  // This will be received by your transmitter and can be displayed
  _crsf.telemetryWriteAttitude(rollDeci, pitchDeci, yawDeci);
}

// Send battery telemetry to transmitter (optional, for future use)
// voltage in Volts, current in Amps
void ELRSReceiver::sendBattery(float voltage, float current) {
  // CRSF battery telemetry expects voltage in centivolts (0.01V) and current in cA (0.01A)
  uint16_t voltageCV = (uint16_t)(voltage * 100.0f);
  uint16_t currentCA = (uint16_t)(current * 100.0f);
  
  // Send battery telemetry (uncomment if library supports this method)
  // _crsf.telemetryWriteBattery(voltageCV, currentCA);
  // Note: Check CRSFforArduino library docs for actual method name
}

// Debug output
void ELRSReceiver::printDebug() {
  // Note: RSSI shows 0 (not available) until telemetry callbacks implemented
  Serial.printf("[ELRS] RSSI: %ddBm | LQ: %d%% | Connected: %s\n",
    _rssi, _lq, isConnected() ? "YES" : "NO");
  // LQ = 100 means receiving valid channels, 0 means disconnected/no data
}
