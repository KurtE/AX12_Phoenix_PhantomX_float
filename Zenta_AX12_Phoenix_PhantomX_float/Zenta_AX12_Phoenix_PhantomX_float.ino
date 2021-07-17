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
#include "USBPSXController.h"
#include "phoenix_input_DIY_Commander.h"
#include "phoenix_driver_bioloid.h"
// We are using the commander. 
//CommanderInputController commander;
USBPSXController usbControl;

// Using Bioloid:
DynamixelServoDriver dxlServo;

void SketchSetup() {
  //g_InputController = &commander;
  //InputController::controller(commander);
  InputController::controller(usbControl);
  ServoDriver::driver(dxlServo);

}
