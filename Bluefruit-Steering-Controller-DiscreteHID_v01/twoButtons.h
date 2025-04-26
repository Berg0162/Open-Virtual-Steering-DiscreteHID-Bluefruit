
// -----------------------------------------------------------------------------------
// Detail specific HCI device/sensor functions for input of steering state and actions
// -----------------------------------------------------------------------------------

/* ------------------------------------------------------------------------------------------------
 * Warning I/O Pins have identical position but different naming depending on the processor board
 * I/O Pin declarations for connection to switch steering buttons
 * 2 momentary push buttons attached to pin A0 and A1 from +5V for manual Left or Right steering
 * 10K resistor attached to pin A0 and A1 from ground
*/ 
// Steering Button settings
#if defined(ARDUINO_NRF52840_FEATHER) 
  #define buttonPin_RIGHT A0   // --> A0/P0.04 connected to Right button
  #define buttonPin_LEFT  A1   // --> A1/P0.05 connected to Left button
#endif
#if defined(ARDUINO_NRF52832_FEATHER) 
  #define buttonPin_RIGHT 2    // --> A0/P0.02 connected to Right button
  #define buttonPin_LEFT  3    // --> A1/P0.03 connected to Left button
#endif
//-------------------------------------------------------------------------------------------------

void setupSteeringState(void){
  DEBUG_PRINTLN("Configure HCI device: Left Right buttons!");
  // initialize the LEFT/RIGHT pushbutton pins as an input:
  pinMode(buttonPin_RIGHT, INPUT_PULLUP); // button pushed --> LOW
  pinMode(buttonPin_LEFT, INPUT_PULLUP);  // button pushed --> LOW
}

float getSteeringState(float angle) {
  static bool ButtonState = false;
  // Notice correct sign: left turn is negative, right turn is positive 
  // check if one of the pushbuttons is pressed. 
  if (digitalRead(buttonPin_RIGHT) == LOW) {    
    ButtonState = true;
    return angle += 1.0;
  } 
  if (digitalRead(buttonPin_LEFT) == LOW) {
    ButtonState = true;
    return angle -= 1.0;     
  }
  // No button pressed or
  // One of the buttons was pressed before and is now released!
  ButtonState = false;
  return angle = 0.0; // Reset angle to zero
}
