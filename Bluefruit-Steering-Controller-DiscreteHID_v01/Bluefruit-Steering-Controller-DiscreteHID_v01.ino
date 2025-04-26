/*********************************************************************
 This is programming code for the nRF52 based Bluefruit BLE boards
 
 The code uses heavily the Adafruit supplied Bluefruit BLE libraries !!
 Adafruit invests time and resources providing open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text must be included in any redistribution
***********************************************************************/

// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
// ------------------------------------------

#include <BluefruitSteeringServer.h>

// LiPo Battery connected ? -> uncomment
#include "batterylevel.h" 
// Battery level percentage value 0-100%
uint8_t batteryPercentage = 90; // Default value

// Sterzo steering global variables
float steerAngle = 0.0; 		// Steering Angle

// -----------------------------------------------------------------------------------
// Detail specific HCI device/sensor functions for input of steering state and actions
// -----------------------------------------------------------------------------------
//#include "twoButtons.h"
#include "twoButtonsPlus.h"
//#include "rotaryEncoder.h"
//#include "ps2JoyStick.h"
// -----------------------------------------------------------------------------------

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb, milliseconds
  delay(500);
  DEBUG_PRINTLN("Bluefruit Steering Controller DiscreteHID");
  DEBUG_PRINTLN("----------  Feather nRF52840  -----------");
  delay(500);
#endif

// When "batterylevel.h" is included -> BATTERY is defined
#ifdef BATTERY
    setupBatteryState();
    batteryPercentage = getBatteryState();
    DEBUG_PRINTF("LiPo Battery Setup - Level: %3d%%\n", batteryPercentage);
#else
    DEBUG_PRINTF("NO LiPo Battery! - Default level: %3d%%\n", batteryPercentage);
#endif

  // Setup HCI device or sensor for steering
  setupSteeringState();
 
  // Start BluefruitSteeringServer  
  BluefruitSteeringServer::getInstance().begin();
} // end setup

bool isTimeToNotifyBatteryLevel(void) {
  static unsigned long notifyDelayTime = 0;
  if(millis() >= notifyDelayTime) {
    notifyDelayTime = millis() + 30000; // Set next moment after 30 seconds
    return true;
  } 
  return false;
}

// Set Number-of-Notifications-per-Second to be sent to Client
#define NOTIFICATIONSPERSECOND 8  
// Calculate time delay between notifications
const unsigned long TIMESPANNOTIFYDELAY = 1000/NOTIFICATIONSPERSECOND; 

bool isTimeToNotifySteerAngle(void) {
  static unsigned long notifyDelayTime = 0;
  if(millis() >= notifyDelayTime) {
    notifyDelayTime = millis() + TIMESPANNOTIFYDELAY; // Set next moment
    return true;
  } 
  return false;
}

void loop() {
    steerAngle = getSteeringState(steerAngle);  // Always fetch HCI input with minimal delay
    if(isTimeToNotifyBatteryLevel()) {          // Update present battery level value 
        // When "batterylevel.h" is included -> BATTERY is defined      
#ifdef BATTERY
        batteryPercentage = getBatteryState();
#endif
        BluefruitSteeringServer::getInstance().updateBatteryPercentage(batteryPercentage);
        }
    if(isTimeToNotifySteerAngle()) {           // Notify steer angle at max allowed pace!
        if(BluefruitSteeringServer::getInstance().updateSteeringValue(steerAngle))
            DEBUG_PRINTF("Steer Angle: %.1f\n", steerAngle);
    }
} // loop
