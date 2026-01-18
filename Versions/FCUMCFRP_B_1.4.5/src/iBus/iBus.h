#ifndef IBUS_h
#define IBUS_h

#include "Arduino.h"
#include "elapsedMillis.h" // Include for elapsedMillis

class IBUS {
  public:
    IBUS(HardwareSerial& bus);
    void begin();
    bool read(uint16_t* channels, bool* failsafe, bool* lostFrame);  // Modified to include failsafe and lostFrame
    void setEndPoints(uint8_t channel,uint16_t min,uint16_t max); // Added for calibration
    void getEndPoints(uint8_t channel,uint16_t *min,uint16_t *max); // Added for calibration
    void setReadCal(uint8_t ch, float *coeff, uint8_t len); // Added for calibration
    void getReadCal(uint8_t ch, float *coeff, uint8_t len); // Added for calibration
    ~IBUS();

  private:
    const uint32_t _ibusBaud = 115200;
    static const uint8_t _numChannels = 14;
    static const uint8_t _frameSize   = 32;
    HardwareSerial* _bus;
    uint8_t _buffer[_frameSize];

    // Added for failsafe and lost frame detection, similar to SBUS
    const uint32_t IBUS_TIMEOUT_US = 20000; // Timeout for iBus frames
    uint8_t _parserState = 0; 
    uint8_t _prevByte = 0x00; // Initialize with a non-header/footer value
    uint8_t _curByte;
    const uint8_t _ibusLostFrame = 0x01; // Example bit for lost frame in iBus flags (placeholder, adjust if protocol defines)
    const uint8_t _ibusFailSafe = 0x02;  // Example bit for failsafe in iBus flags (placeholder, adjust if protocol defines)
    elapsedMicros _ibusTime; // For timeout tracking

    // Added for calibration, mirroring SBUS
    const uint16_t _defaultMin = 1000; // Default min for iBus channels
    const uint16_t _defaultMax = 2000; // Default max for iBus channels
    uint16_t _ibusMin[_numChannels];
    uint16_t _ibusMax[_numChannels];
    float _ibusScale[_numChannels];
    float _ibusBias[_numChannels];
    float **_readCoeff;
    uint8_t _readLen[_numChannels];
    bool _useReadCoeff[_numChannels];

    void scaleBias(uint8_t channel); // Helper for calibration
    float PolyVal(size_t PolySize, float *Coefficients, float X); // Helper for polynomial calibration
};

#endif
