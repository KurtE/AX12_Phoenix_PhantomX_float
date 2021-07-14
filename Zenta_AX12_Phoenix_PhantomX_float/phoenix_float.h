//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kare Halvorsen aka Zenta - Makes everything work correctly!     
//
// This version of the Phoenix code was ported over to the Arduino Environement
//
//
// Phoenix.h - This is the first header file that is needed to build
//          a Phoenix program for a specific Hex Robot.
//
//
// This file assumes that the main source file either directly or through include
// file has defined all of the configuration information for the specific robot.
// Each robot will also need to include:
//  
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
//==============================================================================
#ifndef _PHOENIX_CORE_H_
#define _PHOENIX_CORE_H_
#include <stdarg.h>
//#include <EEPROM.h>
#if defined(__SAM3X8E__)
#define PROGMEM
#define pgm_read_byte(x)        (*((char *)x))
//  #define pgm_read_word(x)        (*((short *)(x & 0xfffffffe)))
#define pgm_read_word(x)        ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x)))
#define pgm_read_byte_near(x)   (*((char *)x))
#define pgm_read_byte_far(x)    (*((char *)x))
//  #define pgm_read_word_near(x)   (*((short *)(x & 0xfffffffe))
//  #define pgm_read_word_far(x)    (*((short *)(x & 0xfffffffe)))
#define pgm_read_word_near(x)   ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x)))
#define pgm_read_word_far(x)    ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x))))
#define PSTR(x)  x
#endif

#ifdef USEXBEE
#include "diyxbee.h"
#endif

//=============================================================================
//[CONSTANTS]
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP   1

#define c1DEC       10
#define c2DEC       100
#define c4DEC       10000
#define c6DEC       1000000


#ifdef QUADMODE
enum {
  cRR=0, cRF, cLR, cLF, CNT_LEGS};
#else
enum {
  cRR=0, cRM, cRF, cLR, cLM, cLF, CNT_LEGS};
#endif

#define WTIMERTICSPERMSMUL      64  // BAP28 is 16mhz need a multiplyer and divider to make the conversion with /8192
#define WTIMERTICSPERMSDIV      125 // 
#define USEINT_TIMERAV

//DynaZgaitEngine
//[Number of gaits]
#define NumOfGaits 4

//[Travel Dead Zone]
#define cTDZ 2      //The Travel Dead Zone for the analog input from the remote

extern float SmoothControl (short CtrlMoveInp, float CtrlMoveOut, byte CtrlDivider);



//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
extern boolean          g_fDebugOutput;
extern boolean          g_fEnableServos;      // Hack to allow me to turn servo processing off...
extern boolean          g_fRobotUpsideDown;    // Is the robot upside down?
extern boolean					g_WakeUpState;					//True if the robot has just been turned on, starting with reduced torque etc.
extern boolean					g_InhibitMovement;		//Set true for temporarily inhibit the controller input e.g when the robot is placed on the ground we don't want to move or walk	
extern void MSound(byte cNotes, ...);
extern boolean CheckVoltage(void);
extern word FilterAnalog(uint8_t pin, word OldValue, byte FilterF);

extern word GetLegsXZLength(void);
extern void AdjustLegPositions(float XZLength);
extern void AdjustLegPositionsToBodyHeight();
extern void ResetLegInitAngles(void);
extern void RotateLegInitAngles (int iDeltaAngle);
extern long GetCmdLineNum(byte **ppszCmdLine);

// debug handler...
extern boolean g_fDBGHandleError;

#ifdef c4DOF
extern const byte cTarsLength[] PROGMEM;
#endif

#ifdef OPT_BACKGROUND_PROCESS
#define DoBackgroundProcess()   ServoDriver::driver()->BackgroundProcess()
#else
#define DoBackgroundProcess()   
#endif

#ifdef DEBUG_IOPINS
#define DebugToggle(pin)  {digitalWrite(pin, !digitalRead(pin));}
#define DebugWrite(pin, state) {digitalWrite(pin, state);}
#else
#define DebugToggle(pin)  {;}
#define DebugWrite(pin, state) {;}
#endif



#ifdef __AVR__
#if not defined(UBRR1H)
#if cSSC_IN != 0
extern SoftwareSerial SSCSerial;
#endif
#endif
#endif
#if defined(__PIC32MX__)
#if defined F
#undef F
#endif
#define F(X) (X)
#endif



//=============================================================================
//=============================================================================
// Define the class(s) for our Input controllers.  
//=============================================================================
//=============================================================================
class InputController {
public:
  static void controller(InputController &ctrlr) {s_controller = &ctrlr;}
  static inline InputController *controller() {return s_controller;}

  virtual void     Init(void);
  virtual void     ControlInput(void);
  virtual void     AllowControllerInterrupts(boolean fAllow);
	virtual void		 SendMsgs(byte Voltage, byte CMD, char Data[21]);

#ifdef OPT_TERMINAL_MONITOR_IC  // Allow Input controller to define stuff as well
  virtual void            ShowTerminalCommandList(void);
  virtual boolean         ProcessTerminalCommand(byte *psz, byte bLen);
#endif

private:
  static InputController *s_controller;
} ;


// Define a function that allows us to define which controllers are to be used.
extern void  RegisterInputController(InputController *pic);



struct coord3D {
    float      x;
    float      y;
    float      z;
} ;

//==============================================================================
// class ControlState: This is the main structure of data that the Control 
//      manipulates and is used by the main Phoenix Code to make it do what is
//      requested.
//==============================================================================
typedef struct _InControlState {
  boolean       fRobotOn;           //Switch to turn on Phoenix
  boolean       fPrev_RobotOn;      //Previous loop state 
		
  //Body position
    coord3D       BodyPos;
    coord3D        BodyRotOffset;                 // Body rotation offset;

  //Body Inverse Kinematics
    coord3D       BodyRot1;                       // X -Pitch, Y-Rotation, Z-Roll

  //[gait]
  byte          GaitType;            //Gait type
	float					DWrotY;							// Delayed Walk BodyRotY 
	float					DWtransX;						//Delayed Walk Body translate X
	float					DWtransZ;						//Delayed Walk Body translate Z
  short       LegLiftHeight;       //Current Travel height
  coord3D       TravelLength;                   // X-Z or Length, Y is rotation.

#ifdef cTurretRotPin
  // Turret information
  int           TurretRotAngle1;      // Rotation of turrent in 10ths of degree
  int           TurretTiltAngle1;    // the tile for the turret
#endif

  //[Single Leg Control]
#ifdef OPT_SINGLELEG
  byte          SelectedLeg;
  coord3D       SLLeg;               // 
  boolean       fSLHold;             //Single leg control mode
	float					SLyawRot;						//Single leg, local leg rotation along Y axis (Yaw rotation) testing!!
#endif

	//[Terrain Adaptation]
	boolean				TAmode;
  
	//[Balance]
  byte       BalanceMode; //Balancemode is always on. Now using this to toogle between different balance methods

  //[TIMING]
  byte          InputTimeDelay; //Delay that depends on the input to get the "sneaking" effect
  word          SpeedControl;   //Adjustible Delay
	byte					ForceSlowCycleWait;  //Some moves need speed control, init with value 2
  byte          ForceGaitStepCnt;          // new to allow us to force a step even when not moving
	boolean				DampDwnSpeed; //Set this true to active more smooth walking on flat floor
	unsigned long		lWhenWeLastSetDatamode; //Keep track of when we started sending the data back to remote, 
	//bug bug doing this since the remote not always get a complete pack. 
	//So we just want to send several times for a second or so

	//[Messages]
	byte					DataMode; //What kind of data are we sending back to the remote
	char					DataPack[21];	//Max 20 characters!, A data holder for the information we want to send back to the remote, mostly text or other commands

#ifdef OPT_DYNAMIC_ADJUST_LEGS
  short         aCoxaInitAngle1[CNT_LEGS]; 
#endif

  // 

} 
INCONTROLSTATE;

//==============================================================================
//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================
//==============================================================================
class ServoDriver {
public:
  static void driver(ServoDriver &drvr) {s_driver = &drvr;}
  static inline ServoDriver *driver() {return s_driver;}

  virtual void Init(void);

  virtual word GetBatteryVoltage(void);
  virtual void WakeUpRoutine(void);

  virtual void            BeginServoUpdate(void);    // Start the update 
#ifdef c4DOF
  virtual void            OutputServoInfoForLeg(int LegIndex, float CoxaAngle, float FemurAngle, float TibiaAngle, float TarsAngle);
#else
  virtual void            OutputServoInfoForLeg(int LegIndex, float CoxaAngle, float FemurAngle, float TibiaAngle);
#endif    
#ifdef cTurretRotPin
  virtual void            OutputServoInfoForTurret(float RotateAngle, float TiltAngle);
#endif
  virtual void            CommitServoDriver(word wMoveTime);
  virtual void            FreeServos(void);
  
  virtual void            IdleTime(void);        // called when the main loop when the robot is not on

  // Allow for background process to happen...
#ifdef OPT_BACKGROUND_PROCESS
  virtual void            BackgroundProcess(void);
#endif    

#ifdef OPT_TERMINAL_MONITOR
  virtual void            ShowTerminalCommandList(void);
  virtual boolean         ProcessTerminalCommand(byte *psz, byte bLen);
#endif

private:
  static ServoDriver *s_driver;

} 
;   

//==============================================================================
//==============================================================================
// Define global class objects
//==============================================================================
//==============================================================================
extern ServoDriver      g_ServoDriver;           // our global servo driver class
extern InputController  *g_InputController;       // Our Input controller 
extern INCONTROLSTATE   g_InControlState;        // State information that controller changes

// BUGBUG:: 
extern float           CoxaAngle[CNT_LEGS];             //Actual Angle of the horizontal hip, decimals = 1
extern float           FemurAngle[CNT_LEGS];            //Actual Angle of the vertical hip, decimals = 1
extern float           TibiaAngle[CNT_LEGS];            //Actual Angle of the knee, decimals = 1
#ifdef c4DOF
extern float           TarsAngle[CNT_LEGS];             //Actual Angle of the knee, decimals = 1
#endif
extern unsigned long   lWakeUpStartTime; //Start time when we entered the WakeUpState
extern const short cInitPosX[CNT_LEGS] ;
extern short cInitPosY[CNT_LEGS];
extern const short cInitPosZ[CNT_LEGS];
//Park positions for the leg
extern const short cParkPosX[CNT_LEGS];
extern const short cParkPosY[CNT_LEGS];
extern const short cParkPosZ[CNT_LEGS];

extern void SketchSetup();

#endif
