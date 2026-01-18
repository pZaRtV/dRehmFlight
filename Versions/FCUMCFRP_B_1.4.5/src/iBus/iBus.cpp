#include "IBus.h"

IBUS::IBUS(HardwareSerial& bus) : _ibusTime(0) {
  _bus = &bus;
  // Initialize calibration-related members
  for (uint8_t i = 0; i < _numChannels; i++) {
    _readLen[i] = 0;
    _useReadCoeff[i] = false;
  }
  _readCoeff = nullptr; // Initialize to nullptr
}

void IBUS::begin() {
  _bus->begin(_ibusBaud, SERIAL_8N1);  // Non-inverted, 115200 baud
  // Initialize default scale factors and biases for all channels
  for (uint8_t i = 0; i < _numChannels; i++) {
    setEndPoints(i, _defaultMin, _defaultMax);
  }
}

bool IBUS::read(uint16_t* channels, bool* failsafe, bool* lostFrame) {
  // reset the parser state if too much time has passed
  if (_ibusTime > IBUS_TIMEOUT_US) {
    _parserState = 0;
    if (lostFrame) { *lostFrame = true; }
    if (failsafe) { *failsafe = true; }  // Added failsafe on timeout
  }

  while (_bus->available() > 0) {
    _ibusTime = 0; // Reset time on byte reception
    _curByte = _bus->read();

    if (_parserState == 0) {
      // Looking for the header byte (0x20)
      if (_curByte == 0x20) {
        _buffer[0] = _curByte; // Store the header
        _parserState = 1;
      } else {
        // Discard non-header byte to resync
        _parserState = 0; 
      }
    } else { // _parserState > 0, we are inside a frame
      // Mid-frame resync: if a header is found, reset parser and start a new frame
      if (_curByte == 0x20) {
          _buffer[0] = _curByte;
          _parserState = 1;
      } else if ((_parserState - 1) < _frameSize) {
        _buffer[_parserState - 1] = _curByte;
        _parserState++;

        if ((_parserState - 1) == _frameSize) {
          // End of frame reached, perform checksum validation
          uint16_t checksum = 0xFFFF;
          for (uint8_t i = 0; i < _frameSize - 2; i++) {
            checksum -= _buffer[i];
          }
          uint16_t rxChecksum = _buffer[_frameSize - 2] | (_buffer[_frameSize - 1] << 8);

          if (checksum == rxChecksum) {
            // Valid frame received
            if (channels) {
              for (uint8_t ch = 0; ch < _numChannels; ch++) {
                uint16_t rawValue = _buffer[2 + ch * 2] | (_buffer[3 + ch * 2] << 8);
                float calibratedValue = rawValue * _ibusScale[ch] + _ibusBias[ch];
                if (_useReadCoeff[ch]) {
                    calibratedValue = PolyVal(_readLen[ch], _readCoeff[ch], calibratedValue);
                }
                channels[ch] = (uint16_t)calibratedValue;
              }
            }
            if (failsafe) { *failsafe = false; }
            if (lostFrame) { *lostFrame = false; }
            _parserState = 0; // Reset for next frame
            return true;
          } else { // Checksum mismatch
            _parserState = 0; // Reset for next frame
            if (lostFrame) { *lostFrame = true; } // Indicate a corrupted frame
            return false;
          }
        }
      }
    }
  }
  return false; // No full valid frame yet
}

IBUS::~IBUS() {
  if (_readCoeff) {
    for (uint8_t i = 0; i < _numChannels; i++) {
      if (_readCoeff[i]) {
        free(_readCoeff[i]);
      }
    }
    free(_readCoeff);
  }
}

void IBUS::setEndPoints(uint8_t channel,uint16_t min,uint16_t max) {
  _ibusMin[channel] = min;
  _ibusMax[channel] = max;
  scaleBias(channel);
}

void IBUS::getEndPoints(uint8_t channel,uint16_t *min,uint16_t *max) {
  if (min && max) {
    *min = _ibusMin[channel];
    *max = _ibusMax[channel];
  }
}

void IBUS::setReadCal(uint8_t channel, float *coeff, uint8_t len) {
  if (coeff) {
    if (!_readCoeff) {
      _readCoeff = (float**) malloc(sizeof(float*)*_numChannels);
    }
    if (!_readCoeff[channel]) {
      _readCoeff[channel] = (float*) malloc(sizeof(float)*len);
    } else {
      free(_readCoeff[channel]);
      _readCoeff[channel] = (float*) malloc(sizeof(float)*len);
    }
    for (uint8_t i = 0; i < len; i++) {
      _readCoeff[channel][i] = coeff[i];
    }
    _readLen[channel] = len;
    _useReadCoeff[channel] = true;
  }
}

void IBUS::getReadCal(uint8_t channel, float *coeff, uint8_t len) {
  if (coeff) {
    for (uint8_t i = 0; (i < _readLen[channel]) && (i < len); i++) {
      coeff[i] = _readCoeff[channel][i];
    }
  }
}

void IBUS::scaleBias(uint8_t channel) {
  _ibusScale[channel] = 2.0f / ((float)_ibusMax[channel] - (float)_ibusMin[channel]);
  _ibusBias[channel] = -1.0f*((float)_ibusMin[channel] + ((float)_ibusMax[channel] - (float)_ibusMin[channel]) / 2.0f) * _ibusScale[channel];
}

float IBUS::PolyVal(size_t PolySize, float *Coefficients, float X) {
  if (Coefficients) {
    float Y = Coefficients[0];
    for (uint8_t i = 1; i < PolySize; i++) {
      Y = Y*X + Coefficients[i];
    }
    return(Y);
  } else {
    return 0;
  }
}
