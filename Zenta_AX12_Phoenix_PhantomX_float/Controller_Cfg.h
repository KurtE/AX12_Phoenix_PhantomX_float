//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the 
//    Version 2 PhantomX robot.
//    Will first define to use their commander unit.
//
//Date: June 19, 2013
//Programmer: Kurt (aka KurtE)
//
//NEW IN V1.0
//   - First Release - Changes from Trossen member KingPin
//
//====================================================================
#ifndef BOARD_CFG_H
#define BOARD_CFG_H

//==================================================================================================================================
// Define which Robot we are building for
//==================================================================================================================================
//#define MXPhoenix //setup for my MX64/106 based hexapod
#if defined(ARDUINO_OpenCM904)
#define MKIV_XL430 // Setup for the PhantomX MK4  XL430-W250 base hexapod
#else
#define MKIII_AX12 //Setup for the PhantomX MKIII AX-12 based hexapod
#endif
//#define MKI_AX18 //Setup for the PhantomX MKI symmetric with orientation sensor

//==================================================================================================================================
// Define which input classes we will use. If we wish to use more than one we need to define USEMULTI - This will define a forwarder
//    type implementation, that the Inputcontroller will need to call.  There will be some negotion for which one is in contol.
//
//  If this is not defined, The included Controller should simply implement the InputController Class...
//==================================================================================================================================
#if defined(ARDUINO_OpenCM904)
#define USE_COMMANDER  // Use the XBee Commander code. 
#define COMMANDER_USE_TIMER 16000 // time in US... 
#else 

#define USE_USB_JOYSTICK
#define USE_BT_KEYPAD
#define BLUETOOTH   // Enable the Bluetooth code in the USB joystick.

//#define USE_COMMANDER  // Use the XBee Commander code.
//#define COMMANDER_USE_TIMER 16000 // time in US... 

//#define_USE_DIY_COMMANDER 

#define ESP_NOW
#if defined(ESP_NOW)
#define ESPserial Serial5
#endif

#endif

//==================================================================================================================================
// Define Which Servo Controller code to use
//==================================================================================================================================

//#define USE_USB_SERIAL_DXL // You are using a USB Host based Servo controller like USB2AX or U2D2

#if defined(USE_USB_SERIAL_DXL)
  #if defined(__IMXRT1062__) || defined(ARDUINO_TEENSY36)
  #else
  #undef USE_USB_SERIAL_DXL
  #endif
#endif

#if defined(KINETISK) || defined(KINETISL) || defined(__IMXRT1062__)
#if defined(ARDUINO_TEENSY31)
#define DXL_SERIAL Serial3 // Old through hole board test...
#else
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN 2
#define DXL_ENABLE_PIN 3
#endif

#ifndef DXL_DIR_PIN
#define DXL_DIR_PIN -1 // 2 - 
#endif

#define DXL_BAUD 1000000 
#define DXL_SERVO_COUNT 18
#endif    

#if defined(ARDUINO_OpenCM904)
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN  28  // 22 if expansion Serial3//OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#define DXL_BAUD 1000000 
#define DXL_SERVO_COUNT 18
#endif
#if defined(ARDUINO_SAMD_MKRZERO)
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN  A6
#define DXL_BAUD 1000000 
#define DXL_SERVO_COUNT 18
#endif


#ifdef MKIV_XL430
#define DYNAMIXEL_PROTOCOL  2
#define GOAL_POSITION_REG 116
#else
#define DYNAMIXEL_PROTOCOL  1
#endif

// Global defines to control which configuration we are using.  Note: Only define one of these...
// 
// Which type of control(s) do you want to compile in
//#if defined(__MK20DX256__)
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
#define DBGSerial         Serial

#elif defined(UBRR2H)
#define DBGSerial         Serial
#else
#define DBGSerial         Serial
#endif

//===================================================================
// Debug Options
#ifdef DBGSerial
#define OPT_TERMINAL_MONITOR 
#define OPT_TERMINAL_MONITOR_IC    // Allow the input controller to define stuff as well
//#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
//#define OPT_PYPOSE
#endif

//#define DEBUG_IOPINS
#ifdef DEBUG_IOPINS
#define DebugToggle(pin)  {digitalWrite(pin, !digitalRead(pin));}
#define DebugWrite(pin, state) {digitalWrite(pin, state);}
#else
#define DebugToggle(pin)  {;}
#define DebugWrite(pin, state) {;}
#endif


// Also define that we are using the AX12 driver
#define USE_AX12_DRIVER
#define OPT_BACKGROUND_PROCESS    // The AX12 has a background process
#define OPT_CHECK_SERVO_RESET     // Try to find single servo that reset it's ID...
#define OPT_SINGLELEG

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
//  PhantomX
//==================================================================================================================================
//[SERIAL CONNECTIONS]

//====================================================================
// XBEE on non mega???
//#if defined(__MK20DX256__)
#if defined(ARDUINO_TEENSY31)
#define XBeeSerial Serial1  // Old through hole board...
#elif defined(KINETISK)  || defined(__IMXRT1062__)
#define XBeeSerial Serial3
#elif defined(ARDUINO_OpenCM904)
#define XBeeSerial Serial2
#elif defined(ARDUINO_SAMD_MKRZERO)
#define XBeeSerial Serial2
#else
#if defined(UBRR2H)
#define XBeeSerial Serial2
#endif
#define XBeeSerial Serial
#endif
#define XBEE_BAUD        38400
//--------------------------------------------------------------------
//[Processor Pin Numbers]
//#if defined(__MK20DX256__)
#if defined(ARDUINO_TEENSY31)
#define SOUND_PIN 6
#define USER 13
#elif defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
#define SOUND_PIN    36
#define USER 13
#elif defined(ARDUINO_OpenCM904)
#define USER 14
#elif defined(ARDUINO_SAMD_MKRZERO)
#define USER LED_BUILTIN

#else
#define SOUND_PIN    1 //0xff        // Tell system we have no IO pin...
#define USER 0                        // defaults to 13 but Arbotix on 0...
#endif

// Define Analog pin and minimum voltage that we will allow the servos to run
//#if defined(__MK20DX256__)
#if defined(ARDUINO_TEENSY31)
#define cVoltagePin  A10
#define CVADR1      402  // VD Resistor 1 - reduced as only need ratio... 40.2K and 10K
#define CVADR2      100    // VD Resistor 2
#define CVREF       330    // 3.3v

#elif defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
// Our Teensy board
#define cVoltagePin  23

#define CVADR1      402  // VD Resistor 1 - reduced as only need ratio... 40.2K and 10K
#define CVADR2      100    // VD Resistor 2
#define CVREF       330    // 3.3v
#elif defined(ARDUINO_OpenCM904)

#endif
//#define cVoltagePin  7      // Use our Analog pin jumper here...
//#define CVADR1      1000  // VD Resistor 1 - reduced as only need ratio... 20K and 4.66K
//#define CVADR2      233   // VD Resistor 2

//#define cTurnOffVol  1095     // 10,95v (3,65v per cell)
//#define cWarningVolt 1104     // 11,04v (3,68v per cell)
//#define cTurnOnVol   1150     // 11V - optional part to say if voltage goes back up, turn it back on...

//====================================================================
// Defines for Optional XBee Init and configuration code.
//====================================================================
#define CHECK_AND_CONFIG_XBEE
#define DEFAULT_MY 0x101  // Swap My/DL on 2nd unit
#define DEFAULT_DL 0x102
#define DEFAULT_ID 0x3332

#endif // BOARD_CFG_H
