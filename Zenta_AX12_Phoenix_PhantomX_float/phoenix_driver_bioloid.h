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
// This library now uses the Robotis library: Dynamixel2Arduino, which you
// can install through the Arduino library install or from  github
//  
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
//==============================================================================
#ifndef _PHOENIX_DRIVER_BIOLOID_H_
#define _PHOENIX_DRIVER_BIOLOID_H_
#include "phoenix_float.h"
#include "Hex_Cfg.h"

//#define USE_AX12_SPEED_CONTROL   // Experiment to see if the speed control works well enough...

//==============================================================================
//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================
//==============================================================================
class DynamixelServoDriver : public ServoDriver {
public:
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

  enum {
    #ifdef c4DOF
      NUMSERVOSPERLEG = 4,
    #else
      NUMSERVOSPERLEG = 3,
    #endif
    #ifdef cTurretRotPin
      NUMSERVOS = NUMSERVOSPERLEG*CNT_LEGS +2
    #else
      NUMSERVOS = (NUMSERVOSPERLEG*CNT_LEGS)
    #endif
  };

private:
  void MakeSureServosAreOn(void);
  void TCSetServoID(byte *psz);
  void TCTrackServos();
  void TCTestServos();
  void TCWiggleServo(byte *psz);
  void TCSetAllServoToCenter();
  void TCSetFreeAllServos();
  
  void SetRegOnAllServos(uint8_t bReg, int32_t bVal, bool checkStatusErrors=false);
  void SetRegOnAllServos2(uint8_t bReg, int32_t wVal, bool checkStatusErrors=false) {SetRegOnAllServos(bReg, wVal, checkStatusErrors);}

 boolean _fServosFree;    // Are the servos in a free state?
  boolean   _fAXSpeedControl;      // flag to know which way we are doing output...
 
#ifdef USE_AX12_SPEED_CONTROL
// Current positions in AX coordinates
  word      _awCurAXPos[NUMSERVOS];
  word      _awGoalAXPos[NUMSERVOS];
#endif

} 
;   
#endif //_PHOENIX_DRIVER_BIOLOID_H_
