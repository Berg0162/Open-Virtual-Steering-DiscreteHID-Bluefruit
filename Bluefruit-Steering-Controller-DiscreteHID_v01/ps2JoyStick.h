// -----------------------------------------------------------------------------------
// Detail specific HCI device/sensor functions for input of steering state and actions
// -----------------------------------------------------------------------------------

/* ------------------------------------------------------------------------------------------------
 * Warning I/O Pins have identical position but different naming depending on the processor board
 * I/O Pin declarations for connection to PS2 Joy Stick */
#if defined(ARDUINO_NRF52840_FEATHER) 
#define PS2_PIN_VRX  A0 // --> A0/P0.04 analog pin connected to VRX output
#define PS2_PIN_VRY  A1 // --> A1/P0.05 analog pin connected to VRY output
#define PS2_PIN_SW   A2 // --> A2/P0.06 digital pin connected to SW-switch output
// PS2_PIN_GND to Feather pin GND and PS2_PIN_5Volt to Feather pin Vcc 3 Volt !!
#endif
#if defined(ARDUINO_NRF52832_FEATHER)
#define PS2_PIN_VRX  2 // --> A0/P0.02 analog pin connected to VRX output
#define PS2_PIN_VRY  3 // --> A1/P0.03 analog pin connected to VRY output
#define PS2_PIN_SW   4 // --> A2/P0.04 digital pin connected to SW-switch output
// PS2_PIN_GND to Feather pin GND and PS2_PIN_5Volt to Feather pin Vcc 3 Volt !!
#endif
//-------------------------------------------------------------------------------------------------

// Definition of some PS2 Joy Stick parameters
#define MAX_ADC_RESOLUTION 4095 // Feather ADC is set to 12 bit resolution
#define MAX_STEER_ANGLE 35 // Steering Angle shoud not exceed this value
// Joystick Calibration -> ZEROPOSDEVIATION is equal to the calculated steer angle 
// when the Joystick is in the neutral position (--> no steering)
#define ZEROPOSDEVIATION 4  // Dependent of the mechanics of your device

void setupSteeringState(void) {
  DEBUG_PRINTLN("Configure HCI device: PS2 Joy Stick (X-axis only)!");
// Setup Joystick SW pin for reading button state
  pinMode(PS2_PIN_SW, INPUT_PULLUP); // SW button pushed --> LOW
}

// Convert ONLY PS2 Joystick xAxis reading to valid steering value!
float getSteeringState(float angle) {
// Map to specifications: left turn is negative, right turn is positive 
   angle = (float)map((long)analogRead(PS2_PIN_VRX), 0, MAX_ADC_RESOLUTION, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
   return (angle-ZEROPOSDEVIATION);
}
