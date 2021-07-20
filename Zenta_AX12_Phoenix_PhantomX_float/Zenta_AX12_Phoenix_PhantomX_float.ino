#include <ax12Serial.h>
#include <BioloidSerial.h>


// Warning setup to build for standard hexapod or for quad.
//  #define QUADMODE  
//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!     
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Arbotix Robocontroller board
//
//=============================================================================
// Warning:: This configuration does not check voltages, so you should be careful to
// not allow the lipo to discharge too far. 
//
// This configuration should hopefully run on a stock PhantomX, without any
// of my changes.
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

#define DEFINE_HEX_GLOBALS
#include <Arduino.h>
#include <EEPROM.h>
#include <avr\pgmspace.h>
#include "Hex_Cfg.h"

#include "phoenix_float.h"

// Are we use the USB Joystick code
#if defined(USE_USB_JOYSTICK) 
#include "USBPSXController.h"
USBPSXController usbControl;
#define INPUT_CONTROLLER_DEFINED
#endif

#if defined(USE_COMMANDER)
// We are using the commander. 
#include "phoenix_input_Commander.h"
CommanderInputController commander;
#define INPUT_CONTROLLER_DEFINED
#endif

#if defined(USE_DIY_COMMANDER)
// We are using the diy commander. 
#include "phoenix_input_DIY_Commander.h"
CommanderInputController commander;
#define INPUT_CONTROLLER_DEFINED
#endif

#ifndef INPUT_CONTROLLER_DEFINED
#error "No input controller defined in config file" 
#endif

#include "phoenix_driver_bioloid.h"


// Using Bioloid:
DynamixelServoDriver dxlServo;

void SketchSetup() {
#if defined(USE_USB_JOYSTICK) 
  InputController::controller(usbControl);
#endif  
#if defined(USE_COMMANDER) || defined(USE_DIY_COMMANDER)
  InputController::controller(commander);
#endif
  ServoDriver::driver(dxlServo);
}
