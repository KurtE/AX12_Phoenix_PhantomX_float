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
#ifndef HEX_CFG_H
#define HEX_CFG_H

//==================================================================================================================================
// Define which Robot we are building for
//==================================================================================================================================
//#define MXPhoenix //setup for my MX64/106 based hexapod
#define MKIII_AX12 //Setup for the PhantomX MKIII AX-12 based hexapod
//#define MKI_AX18 //Setup for the PhantomX MKI symmetric with orientation sensor

//==================================================================================================================================
// Define which input classes we will use. If we wish to use more than one we need to define USEMULTI - This will define a forwarder
//    type implementation, that the Inputcontroller will need to call.  There will be some negotion for which one is in contol.
//
//  If this is not defined, The included Controller should simply implement the InputController Class...
//==================================================================================================================================
//#define USE_USB_JOYSTICK
//#define USE_BT_KEYPAD
//#define BLUETOOTH   // Enable the Bluetooth code in the USB joystick.

#define USE_COMMANDER  // Use the XBee Commander code.
//#define_USE_DIY_COMMANDER 

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
#define DXL_SERIAL (HardwareSerial*)&Serial1
#define DXL_DIR_PIN -1 // 2 - 
#endif    

//==================================================================================================================================
// Details for the different options above. 
//==================================================================================================================================
#ifdef MXPhoenix
#define ServoRes 4096
#define VoltRef 1014
//MIN MAX control definitions:
#define MaxLegLiftHeight 200
#define MedHighLegLiftHeight 150
#define MedLegLiftHeight 100
#define MinLegLiftHeight 70
#define cTibiaHornOffset1 -295//Need to measure for MX-Phoenix = 295 or 29,5deg//463
#define UseFootSensors
#endif

#ifdef MKIII_AX12
#define ServoRes 1024
#define VoltRef 1021
//MIN MAX control definitions:
#define MaxLegLiftHeight 120
#define MedHighLegLiftHeight 90
#define MedLegLiftHeight 60
#define MinLegLiftHeight 40
#define cFemurHornOffset1 -140 //- 14 deg
#define cTibiaHornOffset1 580 //+ 58 deg due to how the MKIII is mounted ref. Trossen assembly instruction
//#define UseFootSensors //bug bug not installed, must disable
#endif

#ifdef MKI_AX18 //I'm using PhantomX MKI AX based robot
#define ServoRes 1024
#define VoltRef 1020
//MIN MAX control definitions:
#define MaxLegLiftHeight 120
#define MedHighLegLiftHeight 90
#define MedLegLiftHeight 60
#define MinLegLiftHeight 40
//No Tibia or Femur Servo horn offset
#define OPT_WALK_UPSIDE_DOWN
#define UseFootSensors //bug bug not installed, must disable
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
#ifdef UseFootSensors
//Define Input pin for foot switches
#define RF_FS 23//Right Front foot switch pin
#define RM_FS	22
#define RR_FS	21
#define LF_FS 2
#define LM_FS 3
#define LR_FS 4
#endif
// Define other optional compnents to be included or not...

#ifdef MXPhoenix //Compared to the PhantomX MKI the Femur and Tibia must be reversed
#define cRRFemurInv 0 
#define cRMFemurInv 0 
#define cRFFemurInv 0 
#define cLRFemurInv 1 
#define cLMFemurInv 1 
#define cLFFemurInv 1 
#define cRRTibiaInv 0 
#define cRMTibiaInv 0 
#define cRFTibiaInv 0 
#define cLRTibiaInv 1 
#define cLMTibiaInv 1 
#define cLFTibiaInv 1 
#endif

#ifdef MKIII_AX12 //Compared to the PhantomX MKI the Tibia must be reversed
#define cRRTibiaInv 0 
#define cRMTibiaInv 0 
#define cRFTibiaInv 0 
#define cLRTibiaInv 1 
#define cLMTibiaInv 1 
#define cLFTibiaInv 1 
#endif

//===================================================================
// Debug Options
#ifdef DBGSerial
#define OPT_TERMINAL_MONITOR 
#define OPT_TERMINAL_MONITOR_IC    // Allow the input controller to define stuff as well
#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
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
#if defined(KINETISK)  || defined(__IMXRT1062__)
#define XBeeSerial Serial3
#else
#if defined(UBRR2H)
#define XBeeSerial Serial2
#endif
#define XBeeSerial Serial
#endif
#define XBEE_BAUD        38400
//--------------------------------------------------------------------
//[Arbotix Pin Numbers]
//#if defined(__MK20DX256__)
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
#define SOUND_PIN    36
#define USER 13
#else
#define SOUND_PIN    1 //0xff        // Tell system we have no IO pin...
#define USER 0                        // defaults to 13 but Arbotix on 0...
#endif

// Define Analog pin and minimum voltage that we will allow the servos to run
//#if defined(__MK20DX256__)
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
// Our Teensy board
#define cVoltagePin  23

#define CVADR1      402  // VD Resistor 1 - reduced as only need ratio... 40.2K and 10K
#define CVADR2      100    // VD Resistor 2
#define CVREF       330    // 3.3v
#endif
//#define cVoltagePin  7      // Use our Analog pin jumper here...
//#define CVADR1      1000  // VD Resistor 1 - reduced as only need ratio... 20K and 4.66K
//#define CVADR2      233   // VD Resistor 2

//#define cTurnOffVol  1095     // 10,95v (3,65v per cell)
//#define cWarningVolt 1104			// 11,04v (3,68v per cell)
//#define cTurnOnVol   1150     // 11V - optional part to say if voltage goes back up, turn it back on...

//===================================================================
// Code to allow Phantom to walk if it is turned upside down.
// This code assumes that the tibias have been changed such that the 0 point has the legs
// sticking straight out instead of at a 90 degree angle.

#ifdef OPT_WALK_UPSIDE_DOWN
#define  IsRobotUpsideDown  (analogRead(A6) < 500)
#else
#endif

//===================================================================


//====================================================================
#define  DEFAULT_GAIT_SPEED 30  // Default gait speed  - Will depend on what Servos you are using...
#define  DEFAULT_SLOW_GAIT  50  // Had a couple different speeds...

//====================================================================
// Defines for Optional XBee Init and configuration code.
//====================================================================
#define CHECK_AND_CONFIG_XBEE
#define DEFAULT_MY 0x101  // Swap My/DL on 2nd unit
#define DEFAULT_DL 0x102
#define DEFAULT_ID 0x3332

//--------------------------------------------------------------------
// Define which pins(sevo IDS go with which joint

#define cRRCoxaPin      8   //Rear Right leg Hip Horizontal
#define cRRFemurPin     10   //Rear Right leg Hip Vertical
#define cRRTibiaPin     12   //Rear Right leg Knee

#define cRMCoxaPin      14  //Middle Right leg Hip Horizontal
#define cRMFemurPin     16  //Middle Right leg Hip Vertical
#define cRMTibiaPin     18  //Middle Right leg Knee

#define cRFCoxaPin      2  //Front Right leg Hip Horizontal
#define cRFFemurPin     4  //Front Right leg Hip Vertical
#define cRFTibiaPin     6   //Front Right leg Knee

#define cLRCoxaPin      7   //Rear Left leg Hip Horizontal
#define cLRFemurPin     9   //Rear Left leg Hip Vertical
#define cLRTibiaPin     11   //Rear Left leg Knee

#define cLMCoxaPin      13   //Middle Left leg Hip Horizontal
#define cLMFemurPin     15   //Middle Left leg Hip Vertical
#define cLMTibiaPin     17  //Middle Left leg Knee

#ifdef OPT_CHECK_SERVO_RESET     // Try to find single servo that reset it's ID...
#define cLFCoxaPin      19   //Front Left leg Hip Horizontal - Avoid servo 1
#else
#define cLFCoxaPin      1   //Front Left leg Hip Horizontal
#endif
#define cLFFemurPin     3   //Front Left leg Hip Vertical
#define cLFTibiaPin     5   //Front Left leg Knee

//#define cTurretRotPin  20   // Turret Rotate Pin
//#define cTurretTiltPin 21  // Turret Tilt pin

//--------------------------------------------------------------------
//[MIN/MAX ANGLES] - Start off assume same as Phoenix...
//Zenta Changes Tibia min/max
//Must calibrate these for MX-Phoenix!
#ifdef MXPhoenix
#define cXXTibiaMin1    -65
#define cXXTibiaMax1    75
#define cXXFemurMin			-120
#define cXXFemurMax			90
#define cXXCoxaMin		-75
#define cXXCoxaMax		75
#endif

#ifdef MKI_AX18 
#define cXXTibiaMin1    -150
#define cXXTibiaMax1    150
#define cXXFemurMin			-100
#define cXXFemurMax			100	
#define cXXCoxaMin		-75
#define cXXCoxaMax		75
#endif

#ifdef MKIII_AX12 
#define cXXTibiaMin1    -150
#define cXXTibiaMax1    150
#define cXXFemurMin			-100
#define cXXFemurMax			100	
#define cXXCoxaMin		-75
#define cXXCoxaMax		75
#endif



#define cRRCoxaMin			cXXCoxaMin
#define cRRCoxaMax			cXXCoxaMax
#define cRRFemurMin			cXXFemurMin
#define cRRFemurMax			cXXFemurMax
#define cRRTibiaMin			cXXTibiaMin1
#define cRRTibiaMax			cXXTibiaMax1

#define cRMCoxaMin			cXXCoxaMin    //Mechanical limits of the Right Middle Leg
#define cRMCoxaMax			cXXCoxaMax
#define cRMFemurMin     cXXFemurMin
#define cRMFemurMax     cXXFemurMax
#define cRMTibiaMin			cXXTibiaMin1
#define cRMTibiaMax     cXXTibiaMax1

#define cRFCoxaMin			cXXCoxaMin    //Mechanical limits of the Right Front Leg
#define cRFCoxaMax			cXXCoxaMax
#define cRFFemurMin			cXXFemurMin
#define cRFFemurMax			cXXFemurMax
#define cRFTibiaMin			cXXTibiaMin1
#define cRFTibiaMax			cXXTibiaMax1

#define cLRCoxaMin			cXXCoxaMin    //Mechanical limits of the Left Rear Leg
#define cLRCoxaMax			cXXCoxaMax
#define cLRFemurMin     cXXFemurMin
#define cLRFemurMax     cXXFemurMax
#define cLRTibiaMin			cXXTibiaMin1
#define cLRTibiaMax     cXXTibiaMax1

#define cLMCoxaMin			cXXCoxaMin    //Mechanical limits of the Left Middle Leg
#define cLMCoxaMax			cXXCoxaMax
#define cLMFemurMin     cXXFemurMin
#define cLMFemurMax     cXXFemurMax
#define cLMTibiaMin			cXXTibiaMin1
#define cLMTibiaMax			cXXTibiaMax1

#define cLFCoxaMin			cXXCoxaMin    //Mechanical limits of the Left Front Leg
#define cLFCoxaMax			cXXCoxaMax
#define cLFFemurMin			cXXFemurMin
#define cLFFemurMax     cXXFemurMax
#define cLFTibiaMin			cXXTibiaMin1
#define cLFTibiaMax			cXXTibiaMax1

/*#define cXXTibiaMin1    -1500
#define cXXTibiaMax1    1500

#define cRRCoxaMin1    -750
#define cRRCoxaMax1    750
#define cRRFemurMin1    -1000
#define cRRFemurMax1    1000
#define cRRTibiaMin1    cXXTibiaMin1
#define cRRTibiaMax1    cXXTibiaMax1

#define cRMCoxaMin1    -750    //Mechanical limits of the Right Middle Leg, decimals = 1
#define cRMCoxaMax1     750
#define cRMFemurMin1     -1000
#define cRMFemurMax1     1000
#define cRMTibiaMin1    cXXTibiaMin1
#define cRMTibiaMax1     cXXTibiaMax1

#define cRFCoxaMin1    -750    //Mechanical limits of the Right Front Leg, decimals = 1
#define cRFCoxaMax1     750
#define cRFFemurMin1    -1000
#define cRFFemurMax1    1000
#define cRFTibiaMin1    cXXTibiaMin1
#define cRFTibiaMax1    cXXTibiaMax1

#define cLRCoxaMin1    -750    //Mechanical limits of the Left Rear Leg, decimals = 1
#define cLRCoxaMax1     750
#define cLRFemurMin1     -1000
#define cLRFemurMax1     1000
#define cLRTibiaMin1    cXXTibiaMin1
#define cLRTibiaMax1     cXXTibiaMax1

#define cLMCoxaMin1    -1000    //Mechanical limits of the Left Middle Leg, decimals = 1
#define cLMCoxaMax1     1000
#define cLMFemurMin1     -1000
#define cLMFemurMax1     1000
#define cLMTibiaMin1    cXXTibiaMin1
#define cLMTibiaMax1     cXXTibiaMax1

#define cLFCoxaMin1     -750    //Mechanical limits of the Left Front Leg, decimals = 1
#define cLFCoxaMax1     750
#define cLFFemurMin1     -1000
#define cLFFemurMax1     1000
#define cLFTibiaMin1    cXXTibiaMin1
#define cLFTibiaMax1     cXXTibiaMax1*/

#define cTurretRotMin  -150
#define cTurretRotMax  150
#define cTurretTiltMin  -12
#define cTurretTiltMax  110

//--------------------------------------------------------------------
//[Joint offsets]
// This allows us to calibrate servos to some fixed position, and then adjust them by moving theim
// one or more servo horn clicks.  This requires us to adjust the value for those servos by 15 degrees
// per click.  This is used with the T-Hex 4DOF legs
//First calibrate the servos in the 0 deg position using the SSC-32 reg offsets, then:
//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#ifdef MXPhoenix
#define cXXCoxaLength     60    // MXPhoenix leg dimensions.
#define cXXFemurLength    127    // 126,501mm to be exact
#define cXXTibiaLength    227  // 
#endif

#ifdef MKI_AX18
#define cXXCoxaLength     52    // PhantomX leg dimensions.
#define cXXFemurLength    82    // Zenta
#define cXXTibiaLength    142  // Zenta
#endif

#ifdef MKIII_AX12
#define cXXCoxaLength     53
#define cXXFemurLength    66
#define cXXTibiaLength    131
#endif

#ifdef PER_LEG_LENGTHS
#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength


#define cRMCoxaLength     cXXCoxaLength	    //Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength


#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength


#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength


#define cLMCoxaLength     cXXCoxaLength	    //Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength


#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength
#endif


//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#ifdef MXPhoenix
#define cRRCoxaAngle   -55   //Default Coxa setup angle
#define cRMCoxaAngle    0      //Default Coxa setup angle
#define cRFCoxaAngle    55      //Default Coxa setup angle
#define cLRCoxaAngle    -55   //Default Coxa setup angle
#define cLMCoxaAngle    0      //Default Coxa setup angle
#define cLFCoxaAngle    55      //Default Coxa setup angle

#define cRROffsetX      -73    //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      105     //Distance Z from center of the body to the Right Rear coxa

#define cRMOffsetX      -113    //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ      0       //Distance Z from center of the body to the Right Middle coxa

#define cRFOffsetX      -73     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -105    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      73      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      105     //Distance Z from center of the body to the Left Rear coxa

#define cLMOffsetX      113     //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ      0       //Distance Z from center of the body to the Left Middle coxa

#define cLFOffsetX      73      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -105    //Distance Z from center of the body to the Left Front coxa
#endif

#ifdef MKI_AX18
#define cRRCoxaAngle   -45   //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle    0      //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle    45      //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle    -45   //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle    45      //Default Coxa setup angle, decimals = 1

#define cRROffsetX      -60     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      120     //Distance Z from center of the body to the Right Rear coxa

#define cRMOffsetX      -100    //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ      0       //Distance Z from center of the body to the Right Middle coxa

#define cRFOffsetX      -60     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -120    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      60      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      120     //Distance Z from center of the body to the Left Rear coxa

#define cLMOffsetX      100     //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ      0       //Distance Z from center of the body to the Left Middle coxa

#define cLFOffsetX      60      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -120    //Distance Z from center of the body to the Left Front coxa
#endif

#ifdef MKIII_AX12
#define cRRCoxaAngle   -45   //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle    0      //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle    45      //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle    -45   //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle    45      //Default Coxa setup angle, decimals = 1

#define cRROffsetX      -60     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      120     //Distance Z from center of the body to the Right Rear coxa

#define cRMOffsetX      -100    //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ      0       //Distance Z from center of the body to the Right Middle coxa

#define cRFOffsetX      -60     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -120    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      60      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      120     //Distance Z from center of the body to the Left Rear coxa

#define cLMOffsetX      100     //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ      0       //Distance Z from center of the body to the Left Middle coxa

#define cLFOffsetX      60      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -120    //Distance Z from center of the body to the Left Front coxa
#endif


//--------------------------------------------------------------------
//[START POSITIONS FEET]
#ifdef MXPhoenix
#define cHexInitXZ	 240
#define CHexInitXZCos60  137        // COS(55) = .574 *240 = 137
#define CHexInitXZSin60  196    // sin(55) = .819 *240 =196
#define CHexInitY	 10 //30
#define cHexGroundPos 25
#endif

#ifdef MKIII_AX12
#define cHexInitXZ	 160
#define CHexInitXZCos60  113       // COS(45) = .707
#define CHexInitXZSin60  113    // sin(45) = .707
#define CHexInitY	 25 //30
#define cHexGroundPos 23 //bug bug no sensors used here yet
#endif

#ifdef MKI_AX18
#define cHexInitXZ	 147
#define CHexInitXZCos60  104        // COS(45) = .707
#define CHexInitXZSin60  104    // sin(45) = .707
#define CHexInitY	 15 //30
#define cHexGroundPos 23 //bug bug no sensors used here yet
#endif
//I'll probably make the init position more dynamic one time..

// Lets try some multi leg positions depending on height settings.
#define CNT_HEX_INITS 2 //Number of init setting
#ifdef MXPhoenix
#define MAX_BODY_Y  250
#define MAX_BODY_Z	120
#define MIN_BODY_Z -120
#else
#define MAX_BODY_Y  150
#define STAND_BODY_Y	50
#define MAX_BODY_Z	80
#define MIN_BODY_Z -80
#endif
extern const byte g_abHexIntXZ[] PROGMEM;
extern const byte g_abHexMaxBodyY[] PROGMEM;

#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ      //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60

// Turret initial position
#define cTurretRotInit  0
#define cTurretTiltInit 0

//[PARKED POSITIONS FEET] //Zenta BETA, not sure if I'll do it this way afterall
#ifdef MXPhoenix
#define cHexParkXZ	 180
#define CHexParkXZCos60  103        // COS(55) = .574 *180 = 103
#define CHexParkXZSin60  147    // sin(55) = .819 *180 =147
#define CHexParkY	 25 //30
#else
//Not defined yet
#define cHexParkXZ	 130
#define CHexParkXZCos60  74        // COS(55) = .574 *180 = 103
#define CHexParkXZSin60  106    // sin(55) = .819 *180 =147
#define CHexParkY	 25 //30
#endif
#define cRRParkPosX     CHexParkXZCos60      //Park positions of the Right Rear leg
#define cRRParkPosY     CHexParkY
#define cRRParkPosZ     CHexParkXZSin60

#define cRMParkPosX     cHexParkXZ      //Park positions of the Right Middle leg
#define cRMParkPosY     CHexParkY
#define cRMParkPosZ     0

#define cRFParkPosX     CHexParkXZCos60      //Park positions of the Right Front leg
#define cRFParkPosY     CHexParkY
#define cRFParkPosZ     -CHexParkXZSin60

#define cLRParkPosX     CHexParkXZCos60      //Park positions of the Left Rear leg
#define cLRParkPosY     CHexParkY
#define cLRParkPosZ     CHexParkXZSin60

#define cLMParkPosX     cHexParkXZ      //Park positions of the Left Middle leg
#define cLMParkPosY     CHexParkY
#define cLMParkPosZ     0

#define cLFParkPosX     CHexParkXZCos60      //Park positions of the Left Front leg
#define cLFParkPosY     CHexParkY
#define cLFParkPosZ     -CHexParkXZSin60
//--------------------------------------------------------------------
//[Tars factors used in formula to calc Tarsus angle relative to the ground]
#define cTarsst	720	//4DOF ONLY
#define cTarsMulti	2	//4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

#if defined __has_include
#  if __has_include ("Local_Hex_Cfg.h")
#    include "Local_Hex_Cfg.h"
#  endif
#endif

#endif // HEX_CFG_H
