
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

// Notice correct sign: left turn is negative, right turn is positive 

#include <OneButton.h> 
// Setup to use default PullUp resistors TRUE --> button pushed --> LOW
OneButton button1(buttonPin_RIGHT);
OneButton button2(buttonPin_LEFT);

#define CONSTANTSTEERANGLE1 10.0  // Constant steerAngle value when button is clicked once
#define CONSTANTSTEERANGLE2 15.0  // Constant steerAngle value when button is double clicked 
#define STEERINGDURATION     500  // Duration in milliseconds, when the steerAngle has a controlled value 
#define LONGPRESSDELAY       500  // The time delay before LongPressStart is called (to eliminate accidental holding down)
unsigned long pressStartTime = 0;
float _angle = 0.0; // "Local" variable that holds steerAngle value

void singleClickButton1(void) {
  _angle = -CONSTANTSTEERANGLE1;
  pressStartTime = millis();
  DEBUG_PRINTLN("singleClick() on pin #1");
} 

void doubleClickButton1(void) {
  _angle = -CONSTANTSTEERANGLE2;
  pressStartTime = millis();
  DEBUG_PRINTLN("doubleClick() on pin #1");
} 

void longPressStartButton1(void) {
  pressStartTime = millis();
  DEBUG_PRINTLN("pressStart() on pin #1");
} 

void longPressStopButton1(void) {  
  pressStartTime = 0;
  _angle = 0;
  DEBUG_PRINTLN("pressStop() on pin #1");
}

void duringLongPressButton1() {
  _angle -= 1;
  delay(100);
  pressStartTime = millis();
  //DEBUG_PRINTLN("Button 1 longPress");
} 

void singleClickButton2(void) {
  pressStartTime = millis();
  _angle = CONSTANTSTEERANGLE1;
  DEBUG_PRINTLN("singleClick() on pin #2");
} // singleClick

void doubleClickButton2(void) {
  pressStartTime = millis();
  _angle = CONSTANTSTEERANGLE2;
  DEBUG_PRINTLN("doubleClick() on pin #2");
}

void longPressStartButton2(void) {
  pressStartTime = millis();
  DEBUG_PRINTLN("pressStart() on pin #2");
}

void longPressStopButton2(void) {
  pressStartTime = 0;
  DEBUG_PRINTLN("pressStop() on pin #2");
}

void duringLongPressButton2() {
  _angle += 1;
  delay(100);
  pressStartTime = millis();
  //DEBUG_PRINTLN("Button 2 longPress");
} 

void checkIntervalDuration(void) {
  if( (millis()-pressStartTime) < STEERINGDURATION ) return;
  _angle = 0.0;
}

void setupSteeringState(void){
  DEBUG_PRINTLN("Configure HCI device: Left Right buttons Plus!");
  // Set the callBack functions
  button1.attachClick(singleClickButton1);
  button1.attachDoubleClick(doubleClickButton1);

  button1.setPressMs(LONGPRESSDELAY); 
  button1.attachLongPressStart(longPressStartButton1);
  button1.attachLongPressStop(longPressStopButton1);
  button1.attachDuringLongPress(duringLongPressButton1);

  button2.attachClick(singleClickButton2);
  button2.attachDoubleClick(doubleClickButton2);

  button2.setPressMs(LONGPRESSDELAY); 
  button2.attachLongPressStart(longPressStartButton2);
  button2.attachLongPressStop(longPressStopButton2);
  button2.attachDuringLongPress(duringLongPressButton2);
}

float getSteeringState(float angle) {  
  _angle = angle; // Set "Local" steerAngle variable
  button1.tick();
  button2.tick();
  checkIntervalDuration();
  return _angle;
}
