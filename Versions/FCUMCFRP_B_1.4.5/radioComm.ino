//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

//========================================================================================================================//

#include "quad.h"

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

void radioSetup() {
  //PPM Receiver 
  #if defined USE_PPM_RX
    //Declare interrupt pin
    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    //Attach interrupt and point to corresponding ISR function
    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);

  //PWM Receiver
  #elif defined USE_PWM_RX
    //Declare interrupt pins 
    pinMode(ch1Pin, INPUT_PULLUP);
    pinMode(ch2Pin, INPUT_PULLUP);
    pinMode(ch3Pin, INPUT_PULLUP);
    pinMode(ch4Pin, INPUT_PULLUP);
    pinMode(ch5Pin, INPUT_PULLUP);
    pinMode(ch6Pin, INPUT_PULLUP);
    delay(20);
    //Attach interrupt and point to corresponding ISR functions
    attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch5Pin), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch6Pin), getCh6, CHANGE);
    delay(20);

  //SBUS Recevier 
  #elif defined USE_SBUS_RX
    sbus.begin();
  
  #elif defined USE_ELRS_RX
    elrs_rx.begin();
    // Serial.println(F("[INIT] ELRS ready")); //debug pipeline

  //DSM receiver
  #elif defined USE_DSM_RX
    Serial3.begin(115000);
  #else
    #error No RX type defined...
  #endif
}

unsigned long getRadioPWM(int ch_num) {
  #if defined USE_ELRS_RX
    // ===== ELRS CRSF HANDLING =====
    elrs_rx.update();                    // Parse latest packet
    uint16_t pwm = elrs_rx.getChannel(ch_num);  // 1000-2000 μs
    return constrain(pwm, 1000, 2000);
    
  #elif defined USE_PPM_RX || defined USE_PWM_RX || defined USE_SBUS_RX || defined USE_DSM_RX
    // ===== LEGACY RECEIVERS =====
    // Your existing channel reading logic
    if (ch_num == 1) 
    return channel_1_raw;
    else if (ch_num == 2) return channel_2_raw;
    else if (ch_num == 3) return channel_3_raw;
    else if (ch_num == 4) return channel_4_raw;
    else if (ch_num == 5) return channel_5_raw;
    else if (ch_num == 6) return channel_6_raw;
    
  #else
    return 1500;  // Failsafe center
  #endif
}
//For DSM type receivers
void serialEvent3(void)
{
  #if defined USE_DSM_RX
    while (Serial3.available()) {
        DSM.handleSerialEvent(Serial3.read(), micros());
    }
  #endif
}



//========================================================================================================================//

//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

#if defined USE_PPM_RX || defined USE_HYBRID_RX
void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig==1) { //Only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    
    if (dt_ppm > 5000) { //Waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }
  
    if (ppm_counter == 1) { //First pulse
      channel_1_raw = dt_ppm;
    }
  
    if (ppm_counter == 2) { //Second pulse
      channel_2_raw = dt_ppm;
    }
  
    if (ppm_counter == 3) { //Third pulse
      channel_3_raw = dt_ppm;
    }
  
    if (ppm_counter == 4) { //Fourth pulse
      channel_4_raw = dt_ppm;
    }
  
    if (ppm_counter == 5) { //Fifth pulse
      channel_5_raw = dt_ppm;
    }
  
    if (ppm_counter == 6) { //Sixth pulse
      channel_6_raw = dt_ppm;
    }
    
    ppm_counter = ppm_counter + 1;
  }
}
#endif // USE_PPM_RX || USE_HYBRID_RX

#if defined USE_PWM_RX || defined USE_PWM_PASSTHROUGH_CH1
void getCh1() {
  int trigger = digitalRead(ch1Pin);
  if(trigger == 1) {
    rising_edge_start_1 = micros();
  }
  else if(trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;
  }
}

void getCh2() {
  int trigger = digitalRead(ch2Pin);
  if(trigger == 1) {
    rising_edge_start_2 = micros();
  }
  else if(trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(ch3Pin);
  if(trigger == 1) {
    rising_edge_start_3 = micros();
  }
  else if(trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh4() {
  int trigger = digitalRead(ch4Pin);
  if(trigger == 1) {
    rising_edge_start_4 = micros();
  }
  else if(trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}

void getCh5() {
  int trigger = digitalRead(ch5Pin);
  if(trigger == 1) {
    rising_edge_start_5 = micros();
  }
  else if(trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}

void getCh6() {
  int trigger = digitalRead(ch6Pin);
  if(trigger == 1) {
    rising_edge_start_6 = micros();
  }
  else if(trigger == 0) {
    channel_6_raw = micros() - rising_edge_start_6;
  }
}
#endif // USE_PWM_RX || USE_PWM_PASSTHROUGH_CH1