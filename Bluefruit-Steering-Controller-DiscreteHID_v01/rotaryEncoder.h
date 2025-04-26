// -----------------------------------------------------------------------------------
// Detail specific HCI device/sensor functions for input of steering state and actions
// -----------------------------------------------------------------------------------

#include "EC11.hpp"

/* ------------------------------------------------------------------------------------------------
 * Warning I/O Pins have identical position but different naming depending on the processor board
 * I/O Pin declarations for connection to Rotary Encoder */
#if defined(ARDUINO_NRF52840_FEATHER) 
#define ENC_PINA_CLK A2 // CLK ENCODER -> Connect PinA with a 10K Ohm Pullup resistor to Vcc 3 Volt
#define ENC_PINB_DT  A3 // DT  ENCODER -> Connect PinB with a 10K Ohm Pullup resistor to Vcc 3 Volt
// --> Connect ENCODER PinC to GND
// --> Connect ENCODER PinD with a 10K Ohm Pullup resistor to Vcc 3 Volt
#define ENC_PINE_SW  A4 // SW ENCODER Switch PinE
#endif
#if defined(ARDUINO_NRF52832_FEATHER)
#define ENC_PINA_CLK 4 // CLK ENCODER -> Connect PinA with a 10K Ohm Pullup resistor to Vcc 3 Volt
#define ENC_PINB_DT  5 // DT  ENCODER -> Connect PinB with a 10K Ohm Pullup resistor to Vcc 3 Volt
// --> Connect ENCODER PinC to GND
// --> Connect ENCODER PinD with a 10K Ohm Pullup resistor to Vcc 3 Volt
#define ENC_PINE_SW  6 // SW ENCODER Switch PinE
#endif

EC11 encoder;

// COMPILE Toggle --> Comment out to run in polling mode otherwise use default interrupts
#define INTERRUPTS  // Gives best results! 

#ifdef INTERRUPTS
// Use an Interrupt routine for detecting encoder rotation
void pinDidChange() {
  encoder.checkPins(digitalRead(ENC_PINA_CLK), digitalRead(ENC_PINB_DT));
}
void setupSteeringState(void){
  DEBUG_PRINTLN("Configure HCI device: EC11 Rotary Encoder (Interrupts)!");
  // Enable the ESP32 internal pull up resistors
  pinMode(ENC_PINA_CLK, INPUT_PULLUP);
  pinMode(ENC_PINB_DT, INPUT_PULLUP); 
  pinMode(ENC_PINE_SW, INPUT_PULLUP);
  attachInterrupt(ENC_PINA_CLK, pinDidChange, CHANGE);
  attachInterrupt(ENC_PINB_DT, pinDidChange, CHANGE);
}
#else
// Polling allows to use the encoder with any digital input pin.
void setupSteeringState(void){
  DEBUG_PRINTLN("Configure HCI device: EC11 Rotary Encoder (Polling mode)!");
#ifdef DEBUG
  Serial.setDebugOutput(true); 
#endif
  // Enable the ESP32 internal pull up resistors
  pinMode(ENC_PINA_CLK, INPUT_PULLUP);
  pinMode(ENC_PINB_DT, INPUT_PULLUP);
  pinMode(ENC_PINE_SW, INPUT_PULLUP); 
}
#endif // INTERRUPTS

float getSteeringState(float angle) {
  EC11Event e;
  if (encoder.read(&e)) {
    // Event is a waiting to be handled.
    if (e.type == EC11Event::StepCW) {
      angle += (float)e.count;
    } else {
      angle -= (float)e.count;
    }
  }
  if(digitalRead(ENC_PINE_SW) == LOW) {angle = 0.0;} // Reset angle to zero (neutral)
#ifndef INTERRUPTS
  // With polling-style pin checking we can still read infrequently, but we need to poll the pins often enough.
  for (int i = 0; i < 200; i++) {
    encoder.checkPins(digitalRead(ENC_PINA_CLK), digitalRead(ENC_PINB_DT));
    delay(1);
  }
#endif
  return angle;
}
