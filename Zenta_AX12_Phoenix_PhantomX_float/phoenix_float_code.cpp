//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 27.08.2020 Zenta stuff
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!     
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Lynxmotion BotBoarduino 
//
// Phoenix_Code.h
//
//     This contains the main code for the Phoenix project.  It is included in
//     all of the different configurations of the phoenix code.
//
//NEW IN V2.X
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

//#define SafetyMode //Run at reduced torque during testing
#include <Arduino.h>
#include "Hex_Cfg.h"
#include "phoenix_float.h"

#include <pins_arduino.h>
//#include <SoftwareSerial.h>        
#define BalanceDivFactor CNT_LEGS    //;Other values than 6 can be used, testing...CAUTION!! At your own risk ;)
//#include <Wire.h>
//#include <I2CEEProm.h>

// Only compile in Debug code if we have something to output to
#ifdef DBGSerial
//#define DEBUG
//#define DEBUG_CHKANGLE
//#define DEBUG_BUG
//#define DEBUG_BALANCE
//#define DEBUG_BAL_ONELEG
//#define DEBUG_X
//#define DEBUG_TIMINGS
//#define DEBUG_UpsideDown
//#define DEBUG_IK
//#define DEBUG_Gait
//#define Debug_ABSPos
//#define DEBUG_Gait_Pos
//#define DEBUG_Equalize
//#define DEBUG_Gait_Speed
#define DEBUG_WakeUp_Pos
//#define DEBUG_GaitPosLimits
//#define DebugVoltage

#endif

#ifdef DEBUG_TIMINGS
boolean g_fDisplayTimings = true;
uint32_t  g_ulDeltaLoopTimes = 0;
uint8_t  g_cDeltaLoopTimes = 0;
uint32_t  g_ulWaitTimes = 0;
uint32_t  g_cWaitTImes = 0;
#endif

//--------------------------------------------------------------------
//[TABLES]
//Build tables for Leg configuration like I/O and MIN/ Max values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

// BUGBUG: Need a cleaner way to define...
// Lets allow for which legs servos to be inverted to be defined by the robot
// This is used by the Lynxmotion Symetrical Quad.
#ifndef cRRCoxaInv
#define cRRCoxaInv 1 
#endif
#ifndef cRMCoxaInv 
#define cRMCoxaInv 1 
#endif
#ifndef cRFCoxaInv 
#define cRFCoxaInv 1 
#endif

#ifndef cLRCoxaInv 
#define cLRCoxaInv 0 
#endif
#ifndef cLMCoxaInv 
#define cLMCoxaInv 0 
#endif
#ifndef cLFCoxaInv 
#define cLFCoxaInv 0 
#endif

#ifndef cRRFemurInv 
#define cRRFemurInv 1 
#endif
#ifndef cRMFemurInv 
#define cRMFemurInv 1 
#endif
#ifndef cRFFemurInv 
#define cRFFemurInv 1 
#endif

#ifndef cLRFemurInv 
#define cLRFemurInv 0 
#endif
#ifndef cLMFemurInv 
#define cLMFemurInv 0 
#endif
#ifndef cLFFemurInv 
#define cLFFemurInv 0 
#endif

#ifndef cRRTibiaInv 
#define cRRTibiaInv 1 
#endif
#ifndef cRMTibiaInv 
#define cRMTibiaInv 1 
#endif
#ifndef cRFTibiaInv 
#define cRFTibiaInv 1 
#endif

#ifndef cLRTibiaInv 
#define cLRTibiaInv 0 
#endif
#ifndef cLMTibiaInv 
#define cLMTibiaInv 0 
#endif
#ifndef cLFTibiaInv 
#define cLFTibiaInv 0 
#endif

#ifndef cRRTarsInv
#define cRRTarsInv 1 
#endif
#ifndef cRMTarsInv 
#define cRMTarsInv 1 
#endif
#ifndef cRFTarsInv 
#define cRFTarsInv 1 
#endif

#ifndef cLRTarsInv 
#define cLRTarsInv 0 
#endif
#ifndef cLMTarsInv 
#define cLMTarsInv 0 
#endif
#ifndef cLFTarsInv 
#define cLFTarsInv 0 
#endif

// Also define default BalanceDelay
#ifndef BALANCE_DELAY
#define BALANCE_DELAY 20
#endif



// Standard Hexapod...
// Servo Horn offsets
#ifdef cRRFemurHornOffset1   // per leg configuration
static const short cFemurHornOffset1[] PROGMEM = {
  cRRFemurHornOffset1, cRMFemurHornOffset1, cRFFemurHornOffset1, cLRFemurHornOffset1, cLMFemurHornOffset1, cLFFemurHornOffset1};
#define CFEMURHORNOFFSET1(LEGI) ((short)pgm_read_word(&cFemurHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cFemurHornOffset1
#define cFemurHornOffset1  0
#endif
#define CFEMURHORNOFFSET1(LEGI)  (cFemurHornOffset1)
#endif

#ifdef cRRTibiaHornOffset1   // per leg configuration
static const short cTibiaHornOffset1[] PROGMEM = {
  cRRTibiaHornOffset1, cRMTibiaHornOffset1, cRFTibiaHornOffset1, cLRTibiaHornOffset1, cLMTibiaHornOffset1, cLFTibiaHornOffset1};
#define CTIBIAHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTibiaHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cTibiaHornOffset1
#define cTibiaHornOffset1  0
#endif
#define CTIBIAHORNOFFSET1(LEGI)  (cTibiaHornOffset1)
#endif

#ifdef c4DOF
#ifdef cRRTarsHornOffset1   // per leg configuration
static const short cTarsHornOffset1[] PROGMEM = {
  cRRTarsHornOffset1,  cRMTarsHornOffset1,  cRFTarsHornOffset1,  cLRTarsHornOffset1,  cLMTarsHornOffset1,  cLFTarsHornOffset1};
#define CTARSHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTarsHornOffset1[LEGI]))
#else   // Fixed per leg, if not defined 0
#ifndef cTarsHornOffset1
#define cTarsHornOffset1  0
#endif
#define CTARSHORNOFFSET1(LEGI)  cTarsHornOffset1
#endif
#endif

//Min / Max values
#ifndef SERVOS_DO_MINMAX
const short cCoxaMin[] PROGMEM = {
  cRRCoxaMin,  cRMCoxaMin,  cRFCoxaMin,  cLRCoxaMin,  cLMCoxaMin,  cLFCoxaMin};
const short cCoxaMax[] PROGMEM = {
  cRRCoxaMax,  cRMCoxaMax,  cRFCoxaMax,  cLRCoxaMax,  cLMCoxaMax,  cLFCoxaMax};
const short cFemurMin[] PROGMEM ={
  cRRFemurMin, cRMFemurMin, cRFFemurMin, cLRFemurMin, cLMFemurMin, cLFFemurMin};
const short cFemurMax[] PROGMEM ={
  cRRFemurMax, cRMFemurMax, cRFFemurMax, cLRFemurMax, cLMFemurMax, cLFFemurMax};
const short cTibiaMin[] PROGMEM ={
  cRRTibiaMin, cRMTibiaMin, cRFTibiaMin, cLRTibiaMin, cLMTibiaMin, cLFTibiaMin};
const short cTibiaMax[] PROGMEM = {
  cRRTibiaMax, cRMTibiaMax, cRFTibiaMax, cLRTibiaMax, cLMTibiaMax, cLFTibiaMax};

#ifdef c4DOF
const short cTarsMin[] PROGMEM = {
  cRRTarsMin, cRMTarsMin, cRFTarsMin, cLRTarsMin, cLMTarsMin, cLFTarsMin};
const short cTarsMax[] PROGMEM = {
  cRRTarsMax, cRMTarsMax, cRFTarsMax, cLRTarsMax, cLMTarsMax, cLFTarsMax};
#endif
#endif

// Servo inverse direction
const bool cCoxaInv[] = {cRRCoxaInv, cRMCoxaInv, cRFCoxaInv, cLRCoxaInv, cLMCoxaInv, cLFCoxaInv};
const bool cFemurInv[] = {cRRFemurInv, cRMFemurInv, cRFFemurInv, cLRFemurInv, cLMFemurInv, cLFFemurInv};
const bool cTibiaInv[] = {cRRTibiaInv, cRMTibiaInv, cRFTibiaInv, cLRTibiaInv, cLMTibiaInv, cLFTibiaInv};

#ifdef c4DOF
const boolean cTarsInv[] = {cRRTarsInv, cRMTarsInv, cRFTarsInv, cLRTarsInv, cLMTarsInv, cLFTarsInv};
#endif  

//Leg Lengths
#ifdef cRRCoxaLength
const byte cCoxaLength[]= {
  cRRCoxaLength,  cRMCoxaLength,  cRFCoxaLength,  cLRCoxaLength,  cLMCoxaLength,  cLFCoxaLength};
const byte cFemurLength[]= {
  cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength};
const byte cTibiaLength[] = {
  cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength};
#ifdef c4DOF
const byte cTarsLength[] = {
  cRRTarsLength, cRMTarsLength, cRFTarsLength, cLRTarsLength, cLMTarsLength, cLFTarsLength};
#endif

#define COXALENGTH(leg)  (cCoxaLength[leg])
#define FEMURLENGTH(leg) (cFemurLength[leg])
#define TIBIALENGTH(leg) (cTibiaLength[leg])
#define TARSLENGTH(leg)  (cTarsLength[leg])

#else
// 
#define COXALENGTH(leg)  (cXXCoxaLength)
#define FEMURLENGTH(leg) (cXXFemurLength)
#define TIBIALENGTH(leg) (cXXTibiaLength)
#define TARSLENGTH(leg)  (cXXTarsLength) 

#endif


//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] PROGMEM = {
  cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX};
const short cOffsetZ[] PROGMEM = {
  cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ};

//Default leg angle
const short cCoxaAngle[] PROGMEM = {
  cRRCoxaAngle, cRMCoxaAngle, cRFCoxaAngle, cLRCoxaAngle, cLMCoxaAngle, cLFCoxaAngle};

//Femur pin table, DONT need this, look how the PinTable is made...
const byte cFemurPinTable[] PROGMEM = {
	cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin,
};

#ifdef UseFootSensors
//FootSensor pin table
const byte cFootSensorPinTable[] PROGMEM = {
	RR_FS, RM_FS, RF_FS, LR_FS, LM_FS, LF_FS,
};

#endif

#ifdef cRRInitCoxaAngle    // We can set different angles for the legs than just where they servo horns are set...
const short cCoxaInitAngle[] PROGMEM = {
  cRRInitCoxaAngle, cRMInitCoxaAngle, cRFInitCoxaAngle, cLRInitCoxaAngle, cLMInitCoxaAngle, cLFInitCoxaAngle};
#endif

//Start positions for the leg
const short cInitPosX[] PROGMEM = {
  cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX};
short cInitPosY[]  = {
  cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY};
const short cInitPosZ[] PROGMEM = {
  cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ};

//Park positions for the leg
const short cParkPosX[] PROGMEM = {
	cRRParkPosX, cRMParkPosX, cRFParkPosX, cLRParkPosX, cLMParkPosX, cLFParkPosX };
const short cParkPosY[] PROGMEM = {
	cRRParkPosY, cRMParkPosY, cRFParkPosY, cLRParkPosY, cLMParkPosY, cLFParkPosY };
const short cParkPosZ[] PROGMEM = {
	cRRParkPosZ, cRMParkPosZ, cRFParkPosZ, cLRParkPosZ, cLMParkPosZ, cLFParkPosZ };

const byte g_abHexIntXZ[] PROGMEM = {cHexInitXZ, 200};//Different XZ init positions (try 200 as optional for MXPhoenix) this is Kurt's work not entirely sure how it work yet..
const byte g_abHexMaxBodyY[] PROGMEM = { 20, MAX_BODY_Y};


//=============================================================================


// Define some globals for debug information
boolean g_fShowDebugPrompt;
boolean g_fDebugOutput;
boolean g_fEnableServos = true;


//====================================================================
//[ANGLES]
float           CoxaAngle[CNT_LEGS];             //Actual Angle of the horizontal hip, decimals = 1
float           FemurAngle[CNT_LEGS];            //Actual Angle of the vertical hip, decimals = 1
float           TibiaAngle[CNT_LEGS];            //Actual Angle of the knee, decimals = 1
#ifdef c4DOF
float           TarsAngle[CNT_LEGS];             //Actual Angle of the knee, decimals = 1
#endif

//--------------------------------------------------------------------
//[POSITIONS SINGLE LEG CONTROL]

float           LegPosX[CNT_LEGS];                //Actual X Position of the Leg
float           LegPosY[CNT_LEGS];                //Actual Y Position of the Leg 
float           LegPosZ[CNT_LEGS];                //Actual Z Position of the Leg
float						LegRotY[CNT_LEGS];								//Accumulated value use for local leg rotation
float						LegRotYxPos[CNT_LEGS];						//Only used during leg is lifted then added to LegPos
float						LegRotYzPos[CNT_LEGS];						//Only used during leg is lifted then added to LegPos
//--------------------------------------------------------------------
//[VARIABLES]
//byte            Index;                            //Index universal used
byte            LegIndex;                //Index used for leg Index Number

//GetSinCos / ArcCos
float           AngleDeg1;                        //Input Angle in degrees, decimals = 1
float           sinA;                             //Output Sinus of the given Angle, decimals = 4
float           cosA;                             //Output Cosinus of the given Angle, decimals = 4
float           AngleRad4;                        //Output Angle in radials, decimals = 4


//Body Inverse Kinematics
float           PosX;                             //Input position of the feet X
float           PosZ;                             //Input position of the feet Z
float           PosY;                             //Input position of the feet Y
float            BodyFKPosX;                       //Output Position X of feet with Rotation
float            BodyFKPosY;                       //Output Position Y of feet with Rotation
float            BodyFKPosZ;                       //Output Position Z of feet with Rotation
float						BalFKPosX;
float						BalFKPosY;
float						BalFKPosZ;

//Leg Inverse Kinematics
float            IKFeetPosX;                       //Input position of the Feet X //Not used???
float            IKFeetPosY;                       //Input position of the Feet Y
float            IKFeetPosZ;                       //Input Position of the Feet Z
float						ABSFootPosX[CNT_LEGS];						//Absolute Position of the foot with center of body as origo used during anti leg colision feature (New in DynaZGaitEngine)
float						ABSFootPosZ[CNT_LEGS];						//Absolute Position of the foot with center of body as origo used during anti leg colision feature (New in DynaZGaitEngine)

byte						IKSolution[CNT_LEGS];        //For each leg, 0 = Solution OK, 1 = Warning, 2 = Error No solution, Finally to be used for something useful in the new DynaZGaitEngine

//--------------------------------------------------------------------
//[TIMING]
unsigned long   lTimerStart;    //Start time of the calculation cycles
unsigned long   lTimerEnd;        //End time of the calculation cycles
unsigned long		lWakeUpStartTime; //Start time when we entered the WakeUpState
byte            CycleTime;        //Total Cycle time
unsigned long		lSendDataTimer;	//Keep track of time, since we only want to send data back to remote every 100mS
unsigned long		lWarningTimer;  //Trigger a notification when the robot-battery should be recharged


word            ServoMoveTime;        //Time for servo updates
word            PrevServoMoveTime;    //Previous time for the servo updates

//--------------------------------------------------------------------
//[GLOBAL]
//--------------------------------------------------------------------

// Define our global Input Control State object
INCONTROLSTATE   g_InControlState;      // This is our global Input control state object...

// Define our ServoWriter class

boolean         g_fLowVoltageShutdown;    // If set the bot shuts down because the input voltage is to low
word						RawVoltage;
word            Voltage;

byte            BattPst;//Batt state in percent


//--boolean         g_InControlState.fRobotOn;            //Switch to turn on Phoenix
//--boolean         g_InControlState.fPrev_RobotOn;        //Previous loop state 
//--------------------------------------------------------------------
//[Balance]
float            TotalTransX;
float            TotalTransZ;
float            TotalTransY;
float            TotalYBal1;
float            TotalXBal1;
float            TotalZBal1;
//[Single Leg Control]
byte            PrevSelectedLeg;
boolean         AllDown;
boolean					PrevFootState;//test

//DynaZgaitEngine

boolean         LegOnGnd[CNT_LEGS]; // To keep track of what leg is on ground and not, true = On ground and the swing phase ends
boolean					MoveLegUp[CNT_LEGS];//True = means the leg must be moved uppward and vice versa
boolean					TAgndTouch[CNT_LEGS];//Foot Sensor Status True= GroundContact
boolean					IncXok;	//Allow Incrementation of LegPosX, part of Anti leg colision
boolean					DecXok;	//Allow Decrementation of LegPosX, part of Anti leg colision 
boolean					IncZok; //Allow Incrementation of LegPosZ, part of Anti leg colision
boolean					DecZok;	//Allow Decrementation of LegPosZ, part of Anti leg colision
boolean					IncRYok;//Allow Incrementation of GaitRotY, part of Anti leg colision
boolean					DecRYok;//Allow Decrementation of GaitRotY, part of Anti leg colision
byte						CGP;//Current gait Phase, constantly synced and calculated for the leg in swing phase
byte						PrevCGP;
byte						CycleCGPBugCnt;//A counter to check how many cycles the halt bug occur
word						StepCounter[CNT_LEGS]; //A stepcounter for each leg, reset when ground contact
word						HLPsc[CNT_LEGS];//Holds the current stepcounter value when the leg in swing phase reached the highest position
word						ESSP;// [CNT_LEGS];//Estimated Steps in Swing Phase	
byte						DampDist; //Damping distance, for how many mm to move the leg at decreased speed before hitting the floor.
byte						DampDiv; //Damping divider. Divider for the DownSpeed for decreasing the downward movement of the leg before hitting the floor.


boolean EqualizeBody;
boolean AllowEquBody;
byte EquCounter;
#define EquCycles 50				// Equalice body in small steps for # of cycles
#define EqualizeDelay 2000	// interpolate for 2000 microseconds for every equcycles

byte DynaZGait[NumOfGaits][7] = {
	{ 20, 10, 0, 50, 40, 30, 10 },//Wavegait Gait Phase values , the last value = swingphase. RR,RM,RF,LR,LM,LF,SP(Swing Phase)
	{ 40, 20, 0, 10, 50, 30, 20 },// ripple
	{ 10, 35, 0, 40, 5, 30, 20 },// tripple
	{ 0, 30, 0, 30, 0, 30, 30 },// tripod
};
boolean AllowGaitShift; //DynaZGait stuff, but not currently used
byte		GT;//DynaZGait Type

const byte LT[8] = { 5, 2, 1, 0, 3, 4, 5, 2 };//remap Leg Table{LF, RF,RM,RR,LR,LM,LF, RF} return correct legadr and make it easier to compare neighbour legs

//[gait - State]
// Note: Information about the current gait is now part of the g_InControlState...

float            GaitPosX[CNT_LEGS];               //Array containing Relative X position corresponding to the Gait
float            GaitPosY[CNT_LEGS];               //Array containing Relative Y position corresponding to the Gait
float            TouchPosY[CNT_LEGS];               //Array containing Relative Y position when touching ground
float            GaitPosZ[CNT_LEGS];               //Array containing Relative Z position corresponding to the Gait
float            GaitRotY[CNT_LEGS];               //Array containing Relative Y rotation corresponding to the Gait

//boolean           GaitLegInAir[CNT_LEGS];     // True if leg is in the air
//byte          GaitNextLeg;                // The next leg which will be lifted

boolean         fWalking;            //  True if the robot are walking
byte            bExtraCycle;          // Forcing some extra timed cycles for avoiding "end of gait bug"
#define         cGPlimit 2           // GP=GaitPos testing different limits

boolean        g_fRobotUpsideDown;    // Is the robot upside down?
int							iAccSamples;
int							iStatus;
unsigned long		lFirstSampleTime;

boolean				g_WakeUpState;					//True if the robot has just been turned on, starting with reduced torque etc.
boolean					g_InhibitMovement;


//=============================================================================
// Define our default standard Gaits
//=============================================================================
#ifndef DEFAULT_GAIT_SPEED
#define DEFAULT_GAIT_SPEED 50
#define DEFAULT_SLOW_GAIT 70
#endif


//=============================================================================
// Function prototypes
//=============================================================================
extern void  WriteOutputs(void);    
extern void SingleLegControl(void);
extern void DynaZGaitEngine(void); //WIP!
extern void InhibitMoving(void);
//extern void WakeUpRoutine(void); //WIP
extern void AntiLegColision(byte Leg_index);
extern void CheckGaitPosLimits(byte Leg_index);//WIP
extern void BalanceBody(void);
extern void CheckAngles();

extern void    PrintSystemStuff(void);            // Try to see why we fault...

extern void BalCalcOneLeg (float PosX, float PosZ, float PosY, byte BalLegNr);
extern void BodyFK (float PosX, float PosZ, float PosY, float RotationY, byte BodyIKLeg, boolean Mode) ;
extern void LegYawRotate(float PosX, float PosZ, float YawAngle, byte LegIndex);
extern void LegIK (float feetPosX, float feetPosY, float feetPosZ, int legIndex);
extern void Gait (byte GaitCurrentLegNr);
extern void GetSinCos(float AngleDeg);

extern void StartUpdateServos(void);
extern boolean TerminalMonitor(void);


InputController  *InputController::s_controller = nullptr;       // Our Input controller 
ServoDriver      *ServoDriver::s_driver = nullptr;


//--------------------------------------------------------------------------
// SETUP: the main arduino setup function.
//--------------------------------------------------------------------------
void setup(){
  SketchSetup();
  delay(500);  // giveus some time to init
  g_fShowDebugPrompt = true;
  g_fDebugOutput = false;
#ifdef DBGSerial    
  DBGSerial.begin(115200);
#endif
  // Init our ServoDriver
  ServoDriver::driver()->Init();
	RawVoltage = analogRead(cVoltagePin);//Preset value, first readings

  //Checks to see if our Servo Driver support a GP Player
  //    DBGSerial.write("Program Start\n\r");
  // debug stuff
  delay(10);
	
	g_WakeUpState = false;					//default false only initiated by g_InControlState.fRobotOn
	
	
  // Setup Init Positions (wakeup position)
  for (LegIndex= 0; LegIndex < CNT_LEGS; LegIndex++ )
  {
    LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);    //Set start positions for each leg
    LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
    LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]); 
		LegRotY[LegIndex] = 0;//just to be sure
		LegRotYxPos[LegIndex] = 0;
		LegRotYzPos[LegIndex] = 0;
		LegOnGnd[LegIndex] = true;//Assume all legs are on ground on ground, maybe change this later
		MoveLegUp[LegIndex] = true;// All legs must be moved up at first
		TAgndTouch[LegIndex] = false;
		TouchPosY[LegIndex] = 0;//Just to be sure
  }
	CGP = 0;//reset to be sure
  ResetLegInitAngles();//Not in use?

  //Single leg control. Make sure no leg is selected
  #ifdef OPT_SINGLELEG
  g_InControlState.SelectedLeg = 255; // No Leg selected
  PrevSelectedLeg = 255;
#endif
  //Body Positions
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;

  //Body Rotations
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.BodyRotOffset.x = 0;
  g_InControlState.BodyRotOffset.y = 0;        //Input Y offset value to adjust centerpoint of rotation
  g_InControlState.BodyRotOffset.z = 0;


  //Gait
	g_InControlState.DWrotY = 0;
	g_InControlState.DWtransX = 0;
	g_InControlState.DWtransZ = 0;
  g_InControlState.GaitType = 2; 
  g_InControlState.BalanceMode = 1;					//Start with full balance mode
	g_InControlState.LegLiftHeight = MedLegLiftHeight;
  g_InControlState.ForceGaitStepCnt = 0;    // added to try to adjust starting positions depending on height...
	g_InControlState.TAmode = false;
	g_InControlState.DampDwnSpeed = true;
	GT = 0; 
	IncXok = true;//just to be sure, probably not necessary since we set them in the Anti Leg Colision function anyway
	DecXok = true;
	IncZok = true;
	DecZok = true;
	IncRYok = true;
	DecRYok = true;
	DampDist = 10;//testing these values
	DampDiv = 3;
//	SmDiv = 10; //Default SmootControl divider
	
#ifdef cTurretRotPin
  g_InControlState.TurretRotAngle = cTurretRotInit;      // Rotation of turrent in 10ths of degree
  g_InControlState.TurretTiltAngle = cTurretTiltInit;    // the tile for the turret
#endif

  InputController::controller()->Init();

  // Servo Driver
  ServoMoveTime = 150;
  g_InControlState.fRobotOn = 0;
  g_fLowVoltageShutdown = false;
#ifdef DEBUG_IOPINS    
  //  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
#endif  
#ifdef UseFootSensors
	pinMode(RF_FS, INPUT_PULLUP); //Right front foot switch test
	pinMode(RM_FS, INPUT_PULLUP);
	pinMode(RR_FS, INPUT_PULLUP);
	pinMode(LF_FS, INPUT_PULLUP);
	pinMode(LM_FS, INPUT_PULLUP);
	pinMode(LR_FS, INPUT_PULLUP);
#endif
#ifdef OPT_WALK_UPSIDE_DOWN
  g_fRobotUpsideDown = false; //Assume off... 
#ifdef DBGSerial  
  DBGSerial.println(IsRobotUpsideDown, DEC);
#endif  
#endif
	MSound(1, 150, 1500);//Just make some sound to indicate setup is done

}


//=============================================================================
// Loop: the main arduino main Loop function
//=============================================================================


void loop(void)
{
  //Start time
  unsigned long lTimeWaitEnd;
  lTimerStart = millis(); 

  DoBackgroundProcess();

  
	CheckVoltage();        // check our voltages...
	
	//Read input from controller
  if (!g_fLowVoltageShutdown) {
        DebugWrite(A1, HIGH);
				
				InputController::controller()->ControlInput();
		
        DebugWrite(A1, LOW);
  }
	if (g_InhibitMovement) {
		InhibitMoving();//Reset all inputs
	}
  WriteOutputs();        // Write Outputs Zenta - remove
#ifdef IsRobotUpsideDown
  
	if (iAccSamples == 0)lFirstSampleTime = millis();
	if (IsRobotUpsideDown)iStatus++;
	else { 
		if(iStatus>0)iStatus--; 
		else iStatus = 0;
	}
	iAccSamples++;
	if (millis() > (lFirstSampleTime + 500)){//The Acc readings must be stable during the last 500mS
		if ((iStatus==iAccSamples)&&!fWalking) g_fRobotUpsideDown = true;
		else if ((iStatus == 0) && !fWalking) g_fRobotUpsideDown = false;
		iAccSamples = 0;
		iStatus = 0;
	}
#ifdef DEBUG_UpsideDown
	DBGSerial.print(iAccSamples,DEC);
	DBGSerial.print(" ");
	DBGSerial.print(iStatus, DEC);
	DBGSerial.print(" ");
	DBGSerial.println(g_fRobotUpsideDown, DEC);
#endif	

#endif
#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown){
	  g_InControlState.TravelLength.x = -g_InControlState.TravelLength.x;
	  g_InControlState.BodyPos.x = -g_InControlState.BodyPos.x;
	  g_InControlState.SLLeg.x = -g_InControlState.SLLeg.x;
	  g_InControlState.BodyRot1.z = -g_InControlState.BodyRot1.z;
		g_InControlState.BodyRot1.y = -g_InControlState.BodyRot1.y;
  }
#endif



	//Dynamic Gait Engine, WIP
	DynaZGaitEngine();
	//DoBackgroundProcess();
  //Single leg control
  SingleLegControl ();

  DoBackgroundProcess();
  DebugWrite(A5, HIGH);
  //Balance calculations
  TotalTransX = 0;     //reset values used for calculation of balance
  TotalTransZ = 0;
  TotalTransY = 0;
  TotalXBal1 = 0;
  TotalYBal1 = 0;
  TotalZBal1 = 0;
  
  
	//Always use balance

    for (LegIndex = 0; LegIndex < (CNT_LEGS/2); LegIndex++) {    // balance calculations for all Right legs

      DoBackgroundProcess();
			BalCalcOneLeg(-LegPosX[LegIndex] - LegRotYxPos[LegIndex] + GaitPosX[LegIndex], LegPosZ[LegIndex] - LegRotYzPos[LegIndex] + GaitPosZ[LegIndex],
          (LegPosY[LegIndex]-(short)pgm_read_word(&cInitPosY[LegIndex]))+GaitPosY[LegIndex], LegIndex);
    }

    for (LegIndex = (CNT_LEGS/2); LegIndex < CNT_LEGS; LegIndex++) {    // balance calculations for all Left legs
      DoBackgroundProcess();
			BalCalcOneLeg(LegPosX[LegIndex] - LegRotYxPos[LegIndex] + GaitPosX[LegIndex], LegPosZ[LegIndex] - LegRotYzPos[LegIndex] + GaitPosZ[LegIndex],
          (LegPosY[LegIndex]-(short)pgm_read_word(&cInitPosY[LegIndex]))+GaitPosY[LegIndex], LegIndex);
    }
    BalanceBody();
		

  for (LegIndex = 0; LegIndex < (CNT_LEGS/2); LegIndex++) {  //Do IK for all Right legs 
	  DoBackgroundProcess();
#ifdef DEBUG_TIMINGS
    uint32_t ulST = micros();
#endif
		
		//There was an old error in this code when calling the BodyFK() should be "+ g_InControlState.BodyPos.x" for both left and right legs

		BodyFK(-LegPosX[LegIndex] - LegRotYxPos[LegIndex] + g_InControlState.BodyPos.x + g_InControlState.DWtransX + GaitPosX[LegIndex],//- TotalTransX
			LegPosZ[LegIndex] - LegRotYzPos[LegIndex] + g_InControlState.BodyPos.z + g_InControlState.DWtransZ + GaitPosZ[LegIndex],//- TotalTransZ
			LegPosY[LegIndex] + g_InControlState.BodyPos.y + GaitPosY[LegIndex],//- TotalTransY
        GaitRotY[LegIndex], LegIndex, 0);//Balance rotation only
  
		if (EqualizeBody){//
			GaitPosX[LegIndex] += BalFKPosX / EquCycles;//test
			GaitPosY[LegIndex] -= BalFKPosY / EquCycles;
			GaitPosZ[LegIndex] -= BalFKPosZ / EquCycles;
			BalFKPosX -= BalFKPosX / EquCycles;
			BalFKPosY -= BalFKPosY / EquCycles;
			BalFKPosZ -= BalFKPosZ / EquCycles;
      //TotalTransX = 0;
      //TotalTransZ = 0;
      //TotalTransY = 0;
		}
   //Move all TotalTransXX calc here, since we must do balance rotation from the same numbers?
   GaitPosX[LegIndex] -= TotalTransX;//-TotalTransX? //LegPosX[LegIndex] += TotalTransX;// testing new method by accumualting the balance translation into LegPosX.
   GaitPosZ[LegIndex] -= TotalTransZ;//LegPosZ[LegIndex] -= TotalTransZ;
   
#ifdef DEBUG_Equalize
		DBGSerial.print("_|_i:");
		DBGSerial.print(LegIndex, DEC);
		DBGSerial.print("_GY:");
		DBGSerial.print((int)GaitPosY[LegIndex], DEC);
		DBGSerial.print("_BY:");
		DBGSerial.print((int)BalFKPosY, DEC);
#endif
		BodyFK(-LegPosX[LegIndex] - LegRotYxPos[LegIndex] + g_InControlState.BodyPos.x + g_InControlState.DWtransX + GaitPosX[LegIndex] + BalFKPosX,//- TotalTransX
			LegPosZ[LegIndex] - LegRotYzPos[LegIndex] + g_InControlState.BodyPos.z + g_InControlState.DWtransZ + GaitPosZ[LegIndex] - BalFKPosZ,//- TotalTransZ
			LegPosY[LegIndex] + g_InControlState.BodyPos.y + GaitPosY[LegIndex] - TotalTransY - BalFKPosY,//- TotalTransY
			GaitRotY[LegIndex], LegIndex, 1);//Rest of the rotations

		LegIK(LegPosX[LegIndex] + LegRotYxPos[LegIndex] - g_InControlState.BodyPos.x - g_InControlState.DWtransX + BodyFKPosX - (GaitPosX[LegIndex]) + BalFKPosX, //g_InControlState.DWtransX + BodyFKPosX____-(GaitPosX[LegIndex] - TotalTransX) 
			LegPosY[LegIndex] + g_InControlState.BodyPos.y - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY - BalFKPosY,//g_InControlState.BodyPos.y - BodyFKPosY_____ - TotalTransY
			LegPosZ[LegIndex] - LegRotYzPos[LegIndex] + g_InControlState.BodyPos.z + g_InControlState.DWtransZ - BodyFKPosZ + GaitPosZ[LegIndex] - BalFKPosZ, LegIndex);//g_InControlState.DWtransZ - BodyFKPosZ____- TotalTransZ, LegIndex
#ifdef DEBUG_TIMINGS
    g_ulDeltaLoopTimes += (micros() - ulST);
#endif
  }
  
  //Do IK for all Left legs  
  for (LegIndex = (CNT_LEGS/2); LegIndex < CNT_LEGS; LegIndex++) {
	  DoBackgroundProcess();
#ifdef DEBUG_TIMINGS
    uint32_t ulST = micros();
#endif
				
		BodyFK(LegPosX[LegIndex] - LegRotYxPos[LegIndex] + g_InControlState.BodyPos.x - g_InControlState.DWtransX + GaitPosX[LegIndex],//- TotalTransX
			LegPosZ[LegIndex] - LegRotYzPos[LegIndex] + g_InControlState.BodyPos.z + g_InControlState.DWtransZ + GaitPosZ[LegIndex],//- TotalTransZ
			LegPosY[LegIndex] + g_InControlState.BodyPos.y + GaitPosY[LegIndex],//- TotalTransY
        GaitRotY[LegIndex], LegIndex, 0);

		if (EqualizeBody){
			GaitPosX[LegIndex] -= BalFKPosX / EquCycles;//test
			GaitPosY[LegIndex] -= BalFKPosY / EquCycles;
			GaitPosZ[LegIndex] -= BalFKPosZ / EquCycles;
			BalFKPosX -= BalFKPosX / EquCycles;
			BalFKPosY -= BalFKPosY / EquCycles;
			BalFKPosZ -= BalFKPosZ / EquCycles;
      //TotalTransX = 0;
      //TotalTransZ = 0;
      //TotalTransY = 0;
			
		}
   GaitPosX[LegIndex] -= TotalTransX;//LegPosX[LegIndex] -= TotalTransX;// testing new method
   GaitPosZ[LegIndex] -= TotalTransZ;//LegPosZ[LegIndex] -= TotalTransZ;
   
#ifdef DEBUG_Equalize
		DBGSerial.print("_|_i:");
		DBGSerial.print(LegIndex, DEC);
		DBGSerial.print("_GY:");
		DBGSerial.print((int)GaitPosY[LegIndex], DEC);
		DBGSerial.print("_BY:");
		DBGSerial.print((int)BalFKPosY, DEC);
#endif

		BodyFK(LegPosX[LegIndex] - LegRotYxPos[LegIndex] + g_InControlState.BodyPos.x - g_InControlState.DWtransX + GaitPosX[LegIndex] - BalFKPosX,//- TotalTransX
			LegPosZ[LegIndex] - LegRotYzPos[LegIndex] + g_InControlState.BodyPos.z + g_InControlState.DWtransZ + GaitPosZ[LegIndex] - BalFKPosZ,//- TotalTransZ
			LegPosY[LegIndex] + g_InControlState.BodyPos.y + GaitPosY[LegIndex] - TotalTransY - BalFKPosY,//- TotalTransY
			GaitRotY[LegIndex], LegIndex, 1);

		LegIK(LegPosX[LegIndex] - LegRotYxPos[LegIndex] + g_InControlState.BodyPos.x + g_InControlState.DWtransX - BodyFKPosX + GaitPosX[LegIndex] - BalFKPosX,//g_InControlState.DWtransX - BodyFKPosX____- TotalTransX
			LegPosY[LegIndex] + g_InControlState.BodyPos.y - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY - BalFKPosY,//g_InControlState.BodyPos.y - BodyFKPosY______- TotalTransY
			LegPosZ[LegIndex] - LegRotYzPos[LegIndex] + g_InControlState.BodyPos.z + g_InControlState.DWtransZ - BodyFKPosZ + GaitPosZ[LegIndex] - BalFKPosZ, LegIndex);//g_InControlState.DWtransZ - BodyFKPosZ______- TotalTransZ, LegIndex
#ifdef DEBUG_TIMINGS
    g_ulDeltaLoopTimes += (micros() - ulST);
#endif
  }
#ifdef DEBUG_Equalize
	DBGSerial.print(" Equ:");
	DBGSerial.println(EqualizeBody, DEC);
#endif
	if (EqualizeBody){
		
		if (EquCounter > (EquCycles-1)){
			DBGSerial.println("************ Equalized!! ************");
			EqualizeBody = false;//run this only ones
			
		}
		EquCounter++;
	}
	
#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown){ //Need to set them back for not messing with the SmoothControl
    g_InControlState.BodyPos.x = -g_InControlState.BodyPos.x;
    g_InControlState.SLLeg.x = -g_InControlState.SLLeg.x;
    g_InControlState.BodyRot1.z = -g_InControlState.BodyRot1.z;
		g_InControlState.BodyRot1.y = -g_InControlState.BodyRot1.y;
  }
#endif

#ifdef DEBUG_TIMINGS
  // Quick and dirty timings to see how long calculations are taking
  g_cDeltaLoopTimes++;
  if (g_cDeltaLoopTimes == 100) {
    DBGSerial.println(g_ulDeltaLoopTimes, DEC);
    g_cDeltaLoopTimes = 0;
    g_ulDeltaLoopTimes = 0;
  }
#endif
	
  //Check mechanical limits
  CheckAngles();

  DebugWrite(A5, LOW);
  //Drive Servos
  if (g_InControlState.fRobotOn) {
    if (g_InControlState.fRobotOn && !g_InControlState.fPrev_RobotOn) {
      MSound(3, 60, 2000, 80, 2250, 100, 2500);
			g_InControlState.BodyPos.y = -10;//During wake up the feet should be a little over ground
			lWakeUpStartTime = millis();
#ifdef USEXBEE
      XBeePlaySounds(3, 60, 2000, 80, 2250, 100, 2500);
#endif  
			
			strcpy(g_InControlState.DataPack, "Init wakeup pos");
			g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
			g_InControlState.lWhenWeLastSetDatamode = millis();

    }

    //Calculate Servo Move time
    /*if ((abs(g_InControlState.TravelLength.x)>cTDZ) || (abs(g_InControlState.TravelLength.z)>cTDZ) ||
      (abs(g_InControlState.TravelLength.y*2)>cTDZ)) {         
      ServoMoveTime = g_InControlState.gaitCur.NomGaitSpeed + (g_InControlState.InputTimeDelay*2) + g_InControlState.SpeedControl;
    } 
    else //Movement speed excl. Walking*/
		ServoMoveTime = DEFAULT_GAIT_SPEED;// +EqualizeDelay;// testing direct control from Left Sliderpot       +10
		
		
    //Add aditional delay when Balance mode is on
      //if (g_InControlState.BalanceMode)
        //ServoMoveTime = ServoMoveTime + BALANCE_DELAY;

        
    // note we broke up the servo driver into start/commit that way we can output all of the servo information
    // before we wait and only have the termination information to output after the wait.  That way we hopefully
    // be more accurate with our timings...
		
    DoBackgroundProcess();
		//WakeUpRoutine();//Not sure where to best place this function..
		//===================================
		//*** SEND DATA TO REMOTE ***
		
		if ((millis() - lSendDataTimer)>30){
			lSendDataTimer = millis();
			InputController::controller()->SendMsgs(BattPst, g_InControlState.DataMode, g_InControlState.DataPack);//Test sending data back to remote, not sure when or where it is best to place it
			if ((g_InControlState.DataMode > 0) && (millis() - g_InControlState.lWhenWeLastSetDatamode) >300){//Reset Datamode after 300mS to be sure the remote got the package. bug bug
				//MSound(1, 30, 1500);
				
				g_InControlState.DataMode = 0;//Reset since we only want to update it for a shorter period
				
			}

    }
		//====================================
    StartUpdateServos();
    
		if ((!g_InControlState.TAmode && (g_InControlState.InputTimeDelay > 10)) || EqualizeBody){//Run at full speed when InputTimeDelay is 10 or less
			if (EqualizeBody){
				lTimeWaitEnd = micros() + EqualizeDelay;
			}
			else{
				lTimeWaitEnd = micros() + ((g_InControlState.InputTimeDelay - 10) * 64);
			}
			do {
				// Test speedcontrol
				DoBackgroundProcess();
			} while (micros() < lTimeWaitEnd);
		}
		
		//Zenta, also check if we want an extra slow motion. E.g when stand up/down
		if (g_InControlState.ForceSlowCycleWait == 1){
			//Wait in the 2. cycle
			lTimeWaitEnd = lTimerStart + PrevServoMoveTime;
			do {
				// Wait the appropriate time, call any background process while waiting...
				DoBackgroundProcess();
			} while (millis() < lTimeWaitEnd);
			g_InControlState.ForceSlowCycleWait = 0;
		}
		if (g_InControlState.ForceSlowCycleWait==2) {
			//Set slow ServoMoveTime in the 1. cycle
			ServoMoveTime = 1000;
			g_InControlState.ForceSlowCycleWait--;
		}
		
		ServoDriver::driver()->WakeUpRoutine();//Not sure where to best place this function..

    // See if we need to sync our processor with the servo driver while walking to ensure the prev is completed 
    //before sending the next one

    // Finding any incident of GaitPos/Rot <>0:
    /*for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
      if ( (GaitPosX[LegIndex] > cGPlimit) || (GaitPosX[LegIndex] < -cGPlimit)
        || (GaitPosZ[LegIndex] > cGPlimit) || (GaitPosZ[LegIndex] < -cGPlimit) 
        || (GaitRotY[LegIndex] > cGPlimit) || (GaitRotY[LegIndex] < -cGPlimit))    {

        bExtraCycle = g_InControlState.gaitCur.NrLiftedPos + 1;//For making sure that we are using timed move until all legs are down
        break;
      }
    }*/
    /*if (bExtraCycle>0){
			
      bExtraCycle--;
      fWalking = !(bExtraCycle==0);

      //Get endtime and calculate wait time
      lTimeWaitEnd = lTimerStart + PrevServoMoveTime;
#ifdef DEBUG_TIMINGS
      uint32_t ulT = millis();
      if (lTimeWaitEnd > ulT)
        g_ulWaitTimes += lTimeWaitEnd - ulT;
      g_cWaitTImes++;
      if (g_cWaitTImes == 100) {
        DBGSerial.print("W ");
        DBGSerial.println(g_ulWaitTimes, DEC);
		
        g_ulWaitTimes = 0;
        g_cWaitTImes = 0;
      }
#endif
      DebugWrite(A1, HIGH);
      do {
        // Wait the appropriate time, call any background process while waiting...
        DoBackgroundProcess();
      } 
      while (millis() < lTimeWaitEnd);
      DebugWrite(A1, LOW);
#ifdef DEBUG_X
      if (g_fDebugOutput) {

        DBGSerial.print("BRX:");
        DBGSerial.print(g_InControlState.BodyRot1.x,DEC); 
        DBGSerial.print("W?:");
         DBGSerial.print(fWalking,DEC);  
         DBGSerial.print(" GS:");
         DBGSerial.print(g_InControlState.GaitStep,DEC);  
         //Debug LF leg
         DBGSerial.print(" GPZ:");
         DBGSerial.print(GaitPosZ[cLF],DEC);
         DBGSerial.print(" GPY:");
         DBGSerial.println(GaitPosY[cLF],DEC);
      }
#endif
    }*/
		
#ifdef DEBUG_X
    if (g_fDebugOutput) {


      DBGSerial.print("TY:");
      DBGSerial.print(TotalYBal1,DEC); 
      DBGSerial.print(" LFZ:");
      DBGSerial.println(LegPosZ[cLF],DEC);
      DBGSerial.flush();  // see if forcing it to output helps...
    }
#endif
    // Only do commit if we are actually doing something...
    DebugToggle(A2);
    ServoDriver::driver()->CommitServoDriver(ServoMoveTime);


  } 
  else {
    //Turn the bot off - May need to add ajust here...
    if (g_InControlState.fPrev_RobotOn || (AllDown= 0)) {
      ServoMoveTime = 600;
      StartUpdateServos();
      ServoDriver::driver()->CommitServoDriver(ServoMoveTime);
      MSound(3, 100, 2500, 80, 2250, 60, 2000);
#ifdef USEXBEE            
      XBeePlaySounds(3, 100, 2500, 80, 2250, 60, 2000);
#endif    
      lTimeWaitEnd = millis() + 600;    // setup to process background stuff while we wait...
      do {
        // Wait the appropriate time, call any background process while waiting...
        DoBackgroundProcess();
      } 
      while (millis() < lTimeWaitEnd);
      //delay(600);
    } 
    else {
      ServoDriver::driver()->FreeServos();
    }

    // Allow the Servo driver to do stuff durint our idle time
    ServoDriver::driver()->IdleTime();

    // We also have a simple debug monitor that allows us to 
    // check things. call it here..
#ifdef OPT_TERMINAL_MONITOR  
    if (TerminalMonitor())
      return;           
#endif
    delay(20);  // give a pause between times we call if nothing is happening
		
  }

  PrevServoMoveTime = ServoMoveTime;
	
  //Store previous g_InControlState.fRobotOn State
  if (g_InControlState.fRobotOn)
    g_InControlState.fPrev_RobotOn = 1;
  else
    g_InControlState.fPrev_RobotOn = 0;
}


void StartUpdateServos()
{        
  byte    LegIndex;

  // First call off to the init...
  ServoDriver::driver()->BeginServoUpdate();    // Start the update 

    for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
			//Why doesn't we just permanently inverse the angle? TEST:
			
#ifdef c4DOF
    ServoDriver::driver()->OutputServoInfoForLeg(LegIndex, 
        cCoxaInv[LegIndex]? -CoxaAngle[LegIndex] : CoxaAngle[LegIndex], 
        cFemurInv[LegIndex]? -FemurAngle[LegIndex] : FemurAngle[LegIndex], 
        cTibiaInv[LegIndex]? -TibiaAngle[LegIndex] : TibiaAngle[LegIndex], 
        cTarsInv[LegIndex]? -TarsAngle[LegIndex] : TarsAngle[LegIndex]);
#else
			CoxaAngle[LegIndex] = (cCoxaInv[LegIndex] ? -CoxaAngle[LegIndex] : CoxaAngle[LegIndex]);
			FemurAngle[LegIndex] = (cFemurInv[LegIndex] ? -FemurAngle[LegIndex] : FemurAngle[LegIndex]);
			TibiaAngle[LegIndex] = (cTibiaInv[LegIndex] ? -TibiaAngle[LegIndex] : TibiaAngle[LegIndex]);
			ServoDriver::driver()->OutputServoInfoForLeg(LegIndex, CoxaAngle[LegIndex], FemurAngle[LegIndex], TibiaAngle[LegIndex]);
    /*ServoDriver::driver()->OutputServoInfoForLeg(LegIndex, 
        cCoxaInv[LegIndex]? -CoxaAngle[LegIndex] : CoxaAngle[LegIndex], 
        cFemurInv[LegIndex]? -FemurAngle[LegIndex] : FemurAngle[LegIndex], 
        cTibiaInv[LegIndex]? -TibiaAngle[LegIndex] : TibiaAngle[LegIndex]);*/
#endif      
  }
#ifdef cTurretRotPin
  ServoDriver::driver()->OutputServoInfoForTurret(g_InControlState.TurretRotAngle, g_InControlState.TurretTiltAngle);  // fist just see if it will talk
#endif  
}




//--------------------------------------------------------------------
//[WriteOutputs] Updates the state of the leds
//--------------------------------------------------------------------
void WriteOutputs(void)//Zenta, should we remove this?
{
#ifdef cEyesPin
  digitalWrite(cEyesPin, Eyes);
#endif        
}
//--------------------------------------------------------------------
//[CHECK VOLTAGE]
//Reads the input voltage and shuts down the bot when the power drops
byte s_bLVBeepCnt;
boolean CheckVoltage() {
#ifdef cTurnOffVol
  // Moved to Servo Driver - BUGBUG: Need to do when I merge back...
  //    Voltage = analogRead(cVoltagePin); // Battery voltage 

	RawVoltage = FilterAnalog(cVoltagePin, RawVoltage, 10);
  Voltage = ((long)RawVoltage*VoltRef)/618;//Small variations on each robot:1018 for MKI 1021 for MKIII Changed to *1014 instead of 1000. I think the reference voltage is about 3,347v instead of 3,300v It seem to work more correct though.. Measured an analog value of 3,3v return analogread = 977!
  //Voltage = ServoDriver::driver()->GetBatteryVoltage();
#ifdef DebugVoltage   
	DBGSerial.print((long)RawVoltage, DEC);
	DBGSerial.print(" V:");
	DBGSerial.println(Voltage, DEC);
#endif  

	BattPst = map(Voltage, 1070, 1240, 0, 100); // 1070 = 3,56v per cell which is very close to critical low LiPo voltage
	if (BattPst > 99) BattPst = 99;//I don't want a percentage of more than 2 digits, for most of the time it will be less than 100% anyway..

  // BUGBUG:: if voltage is 0 it failed to retrieve don't hang program...
  //    if (!Voltage)
  //      return;
	//Send a warning to the remote when Voltage gets very low
	if ((Voltage <= cWarningVolt) && (millis() - lWarningTimer) > 7000){//Send a warning every 7th second
		lWarningTimer = millis();  //Keep track of time
		strcpy(g_InControlState.DataPack, "LOW BATT ON ROBOT");
		g_InControlState.DataMode = 2;//We want to send a text message and make some sound notification on the remote too
		g_InControlState.lWhenWeLastSetDatamode = millis();
	}
  if (!g_fLowVoltageShutdown) {
    if ((Voltage < cTurnOffVol) || (Voltage >= 1999)) {
#ifdef DBGSerial          
      DBGSerial.print("Voltage went low, turn off robot ");
      DBGSerial.println(Voltage, DEC);
#endif            
      //Turn off
			InhibitMoving();//Reset all inputs

#ifdef OPT_SINGLELEG
      g_InControlState.TravelLength.y = 0;
      g_InControlState.SelectedLeg = 255;
#endif
      g_fLowVoltageShutdown = 1;
      s_bLVBeepCnt = 0;    // how many times we beeped...
      g_InControlState.fRobotOn = false;
    }
#ifdef cTurnOnVol
  } 
  else if ((Voltage > cTurnOnVol) && (Voltage < 1999)) {
#ifdef DBGSerial
    DBGSerial.print(F("Voltage restored: "));
    DBGSerial.println(Voltage, DEC);
#endif          
    g_fLowVoltageShutdown = 0;

#endif      
  } 
  else {
    if (s_bLVBeepCnt < 5) {
      s_bLVBeepCnt++;
#ifdef DBGSerial
      DBGSerial.println(Voltage, DEC);
#endif          
      MSound( 1, 45, 2000);
    }
    delay(2000);
  }
#endif  
  return g_fLowVoltageShutdown;
}
//Simple filter function FilterF is the filter factor
word FilterAnalog(uint8_t pin, word OldValue, byte FilterF){
	word NewReading;
	word FilteredValue;
	NewReading = analogRead(pin);
	FilteredValue = (NewReading + (OldValue * FilterF)) / (FilterF + 1);
	return FilteredValue;
}
//A startup routine making sure the legs gets safely into init/start position
//[WakeUp Routine]
//--------------------------------------------------------------------
//--Inhibit moving and walking
void InhibitMoving(void) {
	g_InControlState.BodyPos.x = 0;
	g_InControlState.BodyPos.y = 0;
	g_InControlState.BodyPos.z = 0;
	g_InControlState.BodyRot1.x = 0;
	g_InControlState.BodyRot1.y = 0;
	g_InControlState.BodyRot1.z = 0;
	g_InControlState.TravelLength.x = 0;
	g_InControlState.TravelLength.z = 0;
}
//--------------------------------------------------------------------
// Main rule for a hexapod gait engine:
// No neighbour legs can be lifted at the same time, ex: (LF and RF, or LF and LM)
//[Dynamic Gait Engine] WIP!
void DynaZGaitEngine(void){
	/***********************************************************
	To do and notes:
	A pure GaitPos manipulated gaitengine is more accurate since the GaitPos isn't continously affected by balancemode (TotalTrans)
	But GaitPos continously manipulated by the balance mode gives a little smoother gait transistion and start of some gait methods more fluid, especially ripple. Overall better performance!
	Each leg is simply lifted in a defined order, only the lifted leg(s) get affected by RC directly in direction along the X and Z axis. The balance mode controls the legs on ground (Stance Phase).
	Should a time variable define the length of a lifted state? or just the time it takes to lift the leg? Lowering will vary depending when it hits ground
	Ground detection: On flat ground only stop when LegPosY <= 0. On terrain the state of foot switch and femur load determine ground-hit.
	Different modes of balance. Ex: only compensate for X and Z translation and Y-rotation, faster on flat floor?. 
	Determine if the leg can move to the goal position, safety feature anti leg colision
	2 modes: 
		A. Follow the leg. start with this..
		B. Follow the body. More natural, the hex start by translating/rotate body the leg follows triggered by a threshold?
	When the control values are zero only the legs far from init-position moves back to the init position. 
	-might add a timed feature moving the legs randomly to the init after a while
	A random single leg movement/tap on the front legs to add more life when the hex stands idle over 5 sec? (Toogle feature)
	Different patterns for leglifting. Symmetric (equal speed up and down) or non symmetric. Linear or "curved", Accelerate and Decelerate
	Don't start translating the leg until it's lifted a little, of course.. (make it adjustible)
	
	boolean         LegOnGnd[CNT_LEGS]; // To keep track of what leg is on ground and not, true = On ground and the swing phase ends
	float						CGP;//Current gait Phase, constantly synced and calculated for the leg in swing phase
	word						StepCounter[CNT_LEGS]; //A stepcounter for each leg, reset when ground contact
	word						ESSP[CNT_LEGS];//Estimated Steps in Swing Phase
	const byte DynaZGait[7] = {20,10,0,50,40,30,10};//Gait Phase values start with a wavegait, the last value = swingphase. RR,RM,RF,LR,LM,LF,SP(Swing Phase)
	Use a remap table, changing order: const byte LT[8] ={5, 2, 1, 0, 3, 4, 5, 2}// LF, RF,RM,RR,LR,LM,LF, RF return correct legadr and make it easier to compare neighbour legs
	use a for loop LegIndex 1 to 6, use RmLeg[LegIndex] as a pointer to all other tables!
	Ex: if ((LegOnGnd[LT[(LegIndex-1)]])&&(LegOnGnd[LT[(LegIndex+1)]]))//both neighbour legs to current LegIndex are down and leg can be lifted
	So, if we want to check the neighbour legs to LR the current LegIndex is 4, LT[4-1]= 0(RR) and LT(4+1)= 4(LM)
	
	Try dampening the downspeed the last 10 or 20mm? when walking on flat floor (TA off), make it an option first?
	*/
	float GPYavrage;//Contain the sum of the GaitPosY from all the legs that are currently on ground, then divided by the count of legs on ground to get the average GaitPosY value
	byte CountLegOnGnd;//
	byte LowestCGP = 255;//A temp containg the lowest case of CGP, make sure it gets updated correctly
	//Some values that should be controlfactors:
	//Use g_InControlState.SpeedControl as base for speed calcs

	float UpSpeed = ((67-(float)g_InControlState.SpeedControl)/16);//  All these values must be fine tuned together and simultaniously. Need to figure out a method

	word HLPSteps = (word)((float)g_InControlState.LegLiftHeight / UpSpeed) / 6;;//turned off for now //For how many steps should the leg in swing phase hold the Highest Leg Position, should be calculated based on a % or a factor of LegLiftHeight/UpSpeed?
	
	float DwnSpeed;
	DwnSpeed = g_InControlState.TAmode? UpSpeed / 2: UpSpeed;//mm Up/Dwn speed 0.2 - 8mm?
	
	/*if (g_InControlState.DampDwnSpeed && !g_InControlState.TAmode){// Temporarily removed this extra adjustment of ESSP, since it doesn't give the result I want. It actually gets better by not extending the travel length. Better dampening effect.
		ESSP = (word)((float)g_InControlState.LegLiftHeight / UpSpeed + (float)(g_InControlState.LegLiftHeight - 10) / DwnSpeed + ((float)(DampDist*DampDiv) / DwnSpeed)) + HLPSteps;// DampDist/(DwnSpeed/DampDiv) = DampDist*DampDiv/DwnSpeed
	}
	else{*/
		//If travellength (TL) abs(TL)>(abs(GaitPos)-Margin) -> Speed = 2*TL/ESSP, else Speed = 2*MaxTL/ESSP, could add an tolerance margin=20?
		//For X and Z speed should it only apply to the leg in swing phase? GaitRotY must count for all in swing phase
		//Maybe try to calc an average XZspeed value based on the actual required travel length abs(GaitPosXX - travellength.xx)??

		ESSP = (word)((float)g_InControlState.LegLiftHeight / UpSpeed + (float)g_InControlState.LegLiftHeight / DwnSpeed) + HLPSteps;
	//}
	float ZSpeed; // = UpSpeed * 1.2;//mm
	float XSpeed; 
	float RotSpeed = (abs(g_InControlState.TravelLength.y)) / (float)ESSP;//UpSpeed / 7;//0.2;//deg 
	if (g_InControlState.DampDwnSpeed && !g_InControlState.TAmode){
		ZSpeed = ((4 * abs(g_InControlState.TravelLength.z)) / (float)ESSP);// *(((float)DynaZGait[GT][6] / (60 - (float)DynaZGait[GT][6])));//need to debug GaitPos vs TL
		XSpeed = ((4 * abs(g_InControlState.TravelLength.x)) / (float)ESSP);// *(((float)DynaZGait[GT][6] / (60 - (float)DynaZGait[GT][6])));//3* works a little better than 2*
	}
	else{
		ZSpeed = ((3 * abs(g_InControlState.TravelLength.z)) / (float)ESSP);// *(((float)DynaZGait[GT][6] / (60 - (float)DynaZGait[GT][6])));//need to debug GaitPos vs TL
		XSpeed = ((3 * abs(g_InControlState.TravelLength.x)) / (float)ESSP);// *(((float)DynaZGait[GT][6] / (60 - (float)DynaZGait[GT][6])));//3* works a little better than 2*
	}
  //Have one XSpeed and ZSpeed, Also calc ESSP ones here, no table
  boolean GaitRotYok  = true;  //True if no legs are rotated more than it is allowed to
		
	IncXok = true;//Reset Anti Leg Colision indicators
	DecXok = true;
	IncZok = true;
	DecZok = true;
	IncRYok = true;
	DecRYok = true;
	
		//if ((CGP == 0) && AllowGaitShift){// This is the most ideal gait phase to shift gait.
			GT = g_InControlState.GaitType; //Allow gaitshift any time, testing
			//AllowGaitShift = false;//don't shift more than ones in a complete cycle
		//}
			CountLegOnGnd = 0;
			GPYavrage = 0;
			for (LegIndex = 1; LegIndex <= CNT_LEGS; LegIndex++) {//1. Check through all legs first
				if (LegOnGnd[LT[LegIndex]] == false){ //Check the legs in swing phase 
					//Call the Anti Leg Crash function here
					AntiLegColision(LT[LegIndex]);
				}
				else{//Check the legs in stance phase
					CheckGaitPosLimits(LT[LegIndex]);//Check if current leg on ground is about to cause an IK out of range warning

					//TA-mode: Also check if the legs on ground actually still have contact, if not move further down a little
					//TA-mode:Should I do a GaitPosY neutralizer here?
					//The GPY neutralizer work like this:
					//For all legs on ground do an average calc of all the GaitPosY (GPY), the balance mode also do this but it doesn't affect the GaitPosY directly. Also, I wan't this to work when balance mode is off
					//The main goal is to adjust all the GPY towards 0. Example readings like 10, 12, 5, 2, 4 and 3 gives an average of 6mm after compensating this gives a result of 4, 6, -1, -4, -2 and -3, this gives an average of 0
					//What if one leg is lifted a lot? Like 10, 12, -55, 2, 4, 3, this gives an average of -4. New values 14, 16, -51, 6, 8, 7. And the GPY's are neutralized.
					//Remember to count how many legs are on ground, will vary from 3 to 6.
          #ifdef UseFootSensors //We will not be here if we don't have any sensors
					if (g_InControlState.TAmode){
						CountLegOnGnd++;
						TAgndTouch[LT[LegIndex]] = !digitalRead(pgm_read_byte(&cFootSensorPinTable[LT[LegIndex]])); //BUG BUG during turned off state Need to test more!
						if (TAgndTouch[LT[LegIndex]] == false){//if lost contact
							GaitPosY[LT[LegIndex]] += (DwnSpeed/2);//Move leg down half speed 
							//Should have some sort of safety limits when leg doesn't touch any ground or if the switch fails, similar to what's done for the legs in swingphase
						}
						GPYavrage += GaitPosY[LT[LegIndex]];
            #ifdef DebugTA
						DBGSerial.print("Y:");
						DBGSerial.print((int)GaitPosY[LT[LegIndex]], DEC);
						DBGSerial.print("_");
            #endif //DebugTA
					}
          #endif
				}
				

#ifdef DEBUG_GaitPosLimits
				if (!IncZok || !DecZok || !IncXok || !DecXok || !IncRYok || !DecRYok) {//sound indicator for debug
					MSound(1, 5, 2000);
				}
#endif
			}
			if (g_InControlState.TAmode){
				GPYavrage = GPYavrage / (float)CountLegOnGnd;
#ifdef DebugTA
				DBGSerial.print(" Count:");
				DBGSerial.print(CountLegOnGnd, DEC);
				DBGSerial.print("_Average:");
				DBGSerial.println(GPYavrage, DEC);
#endif //DebugTA
        for (LegIndex = 1; LegIndex <= CNT_LEGS; LegIndex++) {//Do the GaitPosY neutralizer
        if (LegOnGnd[LT[LegIndex]] == true){
          if(GPYavrage> 0.5){
              GaitPosY[LT[LegIndex]] -= 0.5;//Try a fixed value for correction at first, maybe just use DownSpeed/2 ?
            }
            else if(GPYavrage< -0.5){
                GaitPosY[LT[LegIndex]] += 0.5;
              }
						if (GaitPosY[LT[LegIndex]]> -50){//test!!!
							TouchPosY[LT[LegIndex]] = GaitPosY[LT[LegIndex]];
						}
						else{
							TouchPosY[LT[LegIndex]] = -50;//This determine the maximum extra lifting for the next leg lift
						}
          }
        }
				if ((CountLegOnGnd == 6) && AllowEquBody){//test equalize balance mode only ones
					EqualizeBody = true;
					AllowEquBody = false;//Make sure this happen only ones
					//Initate a counter for equalizing the body in xx steps?
					EquCounter = 0;
				}
				if (CountLegOnGnd < 6){
					AllowEquBody = true; //Make sure we do the equalizebody next time all legs are down again
				}
			}

			for (LegIndex = 1; LegIndex <= CNT_LEGS; LegIndex++) {//2. Then go through all the legs again

				if (LegOnGnd[LT[LegIndex]] == false){//this leg is currently in swing phase, do all the control stuff
					//first check if conditions are met for checking ground contact
					StepCounter[LT[LegIndex]]++;
					CycleCGPBugCnt = 0;//Always reset this when we have a leg in swing phase
					if (MoveLegUp[LT[LegIndex]]) {
						if (GaitPosY[LT[LegIndex]] > (-(g_InControlState.LegLiftHeight - TouchPosY[LT[LegIndex]]))){// Move leg upward ***! , if TouchPosY >0 the leg won't be lifted unnecessary to much, useful when walking to a lower ground
							GaitPosY[LT[LegIndex]] -= UpSpeed;//Move leg upward fast until it reaches the current legliftheight, could make this a function, sine or whatever (but that will affect ESSP)..
							TAgndTouch[LT[LegIndex]] = false;
						}
						else {//The leg should be at the top, stop moving it
							GaitPosY[LT[LegIndex]] = (-(g_InControlState.LegLiftHeight - TouchPosY[LT[LegIndex]]));//
							MoveLegUp[LT[LegIndex]] = false;//start moving it down again
							TAgndTouch[LT[LegIndex]] = false;
							HLPsc[LT[LegIndex]] = StepCounter[LT[LegIndex]];
						}
					}
					else if (g_InControlState.TAmode || ((GaitPosY[LT[LegIndex]] < 0) && !g_InControlState.TAmode)){//Move leg down again
												
							if (StepCounter[LT[LegIndex]]> (HLPsc[LT[LegIndex]] + HLPSteps)){// Hold the leg at top a little. BUG BUG mess up gait translation a little
								if (g_InControlState.TAmode){
#ifdef UseFootSensors //We will not be here if we don't have any sensors
									//**** Footsensor readings start
									TAgndTouch[LT[LegIndex]] = !digitalRead(pgm_read_byte(&cFootSensorPinTable[LT[LegIndex]])); //The actual port return a 0 if switch is activated (ground contact)
									//**** Footsensor reading ends
#endif
									if (TAgndTouch[LT[LegIndex]] == false){//if no contact yet
										if ((IKSolution[LT[LegIndex]] == 0)){//If no IK warning keep on moving leg downwards
											GaitPosY[LT[LegIndex]] += DwnSpeed;//Move leg down
										}
										else {//Need some sort of limit. Must determine when to stop moving down.
											MoveLegUp[LT[LegIndex]] = true;//Do a new attempt if the current leg doesn't sense any ground until the IK warning hits us.
										}
										
									}
									
								}
								else{//TA off:
									if (g_InControlState.DampDwnSpeed && (GaitPosY[LT[LegIndex]]> -DampDist)){
										GaitPosY[LT[LegIndex]] += (DwnSpeed/DampDiv);//Move leg down slowly, test bug bug. Might need to calc the ESSP first
									}
									else{
										GaitPosY[LT[LegIndex]] += DwnSpeed;//Move leg down,
									}
									 
									TouchPosY[LT[LegIndex]] = 0;//just to be sure
								}
							}
						


					}


					//Calculate the ESSP here (more to do in TA mode!?): Moved to top of function
					//ESSP = (word)((float)g_InControlState.LegLiftHeight / UpSpeed + (float)g_InControlState.LegLiftHeight / DwnSpeed); //Any reason for doing this for each leg? Should be a constant during one cycle, move to top?
					
					//Calc CGP here, rule: can't be larger than gait phase + swing phase
					//Should just check if StepCounter>ESSP -> StepCounter=ESSP, or adjust StepCounter with CGP?
					CGP = (((StepCounter[LT[LegIndex]] * DynaZGait[GT][6]) / ESSP) + DynaZGait[GT][LT[LegIndex]]);//need to debug this probably
					if (CGP > (DynaZGait[GT][LT[LegIndex]] + DynaZGait[GT][6])){//DynaZGait[xxx][6] = length of Swing Phase
						CGP = (DynaZGait[GT][LT[LegIndex]] + DynaZGait[GT][6]);
					}

					//*** Ground Contact *** :
					if (TAgndTouch[LT[LegIndex]] || ((GaitPosY[LT[LegIndex]] >= 0) && !g_InControlState.TAmode)){ //might add another condition if we are in TA mode
						//remember to include and reset the LegRotYx+zPos into the LegPosX+Z after ground contact. Also reset Stepcounter and set the LegOnGnd=true!
						LegOnGnd[LT[LegIndex]] = true;//Ground contact (on flat floor)
						MoveLegUp[LT[LegIndex]] = true;//make sure it move upward on next gait cycle again
						StepCounter[LT[LegIndex]] = 0;
						CGP = (DynaZGait[GT][LT[LegIndex]] + DynaZGait[GT][6]);//This swing phase is over anyway..
						TAgndTouch[LT[LegIndex]] = false;

						//In TA mode we might need to save the GaitPosY value, we need it when calcing ESSP
					}
					else{//Do PosX, PosZ and LegRotY control here:
						//Also have a rule that inhibit the leg to move before it has been lifted more than a few mm ?
						//*** Movement control code***
						//If abs(TL)>(abs(GaitPos)-Margin) -> Speed = 2*TL/ESSP, else Speed = 2*MaxTL/ESSP, could add an tolerance margin=20?
						//This make sure the leg moves at correct speed when the g_InControlState.TravelLength is less than the actual GaitPos value
						//If you suddenly release the joystick you need to have a fixed speed for the legs to return. This value isn't that crucial
						//But make this a variable depending on the Max TL. Not yet defined (To Do list in the commander file)
						if (abs(g_InControlState.TravelLength.z) < abs(GaitPosZ[LT[LegIndex]])){//do it for each leg or all at ones?
							ZSpeed = 255 / (float)ESSP;//ZSpeed = 2xMaxTL/ESSP //180
						}
						if (abs(g_InControlState.TravelLength.x) < abs(GaitPosX[LT[LegIndex]])) {
							XSpeed = 255 / (float)ESSP;
						}
						if (abs(g_InControlState.TravelLength.y) < abs(GaitRotY[LT[LegIndex]])) {
							RotSpeed = 64 / (float)ESSP;// 50
						}
						if ((GaitPosZ[LT[LegIndex]] < (g_InControlState.TravelLength.z - ZSpeed)) && IncZok){
							GaitPosZ[LT[LegIndex]] += ZSpeed;
						}
						else if ((GaitPosZ[LT[LegIndex]] > (g_InControlState.TravelLength.z + ZSpeed)) && DecZok){
							GaitPosZ[LT[LegIndex]] -= ZSpeed;
						}

						if ((GaitPosX[LT[LegIndex]] < (-g_InControlState.TravelLength.x - XSpeed)) && IncXok){
							GaitPosX[LT[LegIndex]] += XSpeed;
						}
						else if ((GaitPosX[LT[LegIndex]] > (-g_InControlState.TravelLength.x + XSpeed)) && DecXok){
							GaitPosX[LT[LegIndex]] -= XSpeed;
						}
						
						if ((GaitRotY[LT[LegIndex]]<(g_InControlState.TravelLength.y - RotSpeed)) && IncRYok){//Rotate leg CCW
							GaitRotY[LT[LegIndex]] += RotSpeed;//
						}
						else if ((GaitRotY[LT[LegIndex]]>(g_InControlState.TravelLength.y + RotSpeed)) && DecRYok){//Rotate leg CW
							GaitRotY[LT[LegIndex]] -= RotSpeed;
						}
						//*** Movement control code ends***
					}

					/*if (CGP > 0){
						AllowGaitShift = true;
					}*/
					if (CGP >= 60){
						CGP -= 60;
					}
					if (CGP < LowestCGP){// Make sure transition from 59 - 0 gets correctly.
							LowestCGP = CGP;
					}

				}//Swing Phase ends here. if (LegOnGnd[LT[LegIndex]] == false) 
#ifdef DEBUG_Gait			
				DBGSerial.print(LT[LegIndex], DEC);
				DBGSerial.print("-");
				DBGSerial.print(CGP, DEC);
				DBGSerial.print("_T:");
				DBGSerial.print(LegOnGnd[LT[LegIndex]], DEC);
				DBGSerial.print("-SC:");
				DBGSerial.print(StepCounter[LT[LegIndex]], DEC);
				DBGSerial.print("-Y:");
				DBGSerial.print((int)GaitPosY[LT[LegIndex]], DEC);
				DBGSerial.print("-:");
				DBGSerial.print((int)TouchPosY[LT[LegIndex]], DEC);
				DBGSerial.print("]__[");
#endif			
			}//for loop ends
			//
			if (LowestCGP<255){
				CGP = LowestCGP;//Current Gait Phase should be the lowest value calculated this cycle
			}
			//A bug/problem might occur when shifting from one gait type to another if all legs are down and the CGP get stuck we then need to increase the CGP 
			if ((CGP == PrevCGP) && LegOnGnd[0] && LegOnGnd[1] && LegOnGnd[2] && LegOnGnd[3] && LegOnGnd[4] && LegOnGnd[5] && ((abs(g_InControlState.TravelLength.z)>cTDZ) || (abs(g_InControlState.TravelLength.x)>cTDZ) || (abs(g_InControlState.TravelLength.y)>cTDZ))){
				//Give it a chance at first, maybe count cycles = ESSP/(Swing Phase) when ESSP>(Swing Phase)?
				if (CycleCGPBugCnt > 10){//10 should be plenty bug bug
					CGP++;
					if (CGP >= 60){
						CGP -= 60;
					}
				}
				else{
					CycleCGPBugCnt++;
				}

			}
			PrevCGP = CGP;

		
			//Do another for loop here since we need CGP to be updated on all legs first
			for (LegIndex = 1; LegIndex <= CNT_LEGS; LegIndex++) {//Check through all legs
			
				if (LegOnGnd[LT[LegIndex]] == true){
					if ((LegOnGnd[LT[(LegIndex - 1)]]) && (LegOnGnd[LT[(LegIndex + 1)]]) && (CGP == DynaZGait[GT][LT[LegIndex]])){//the leg is on ground check all conditions to see if it can be lifted again
						//The leg is currently in stance phase, and can only be lifted if both neighbour legs are down and we are at the current gait phase (CGP)
						//Should we allow leglift when CGP == , and + 1 ?
						//As soon as these requirements are met, we can start a new swing phase
						//Could also check other conditions, like if the controls are 0 we must decide if the leg should be lifted again or just stay on ground
						if ((abs(g_InControlState.TravelLength.z)>cTDZ) || (abs(g_InControlState.TravelLength.x) > cTDZ) || (abs(g_InControlState.TravelLength.y) >cTDZ)){
							LegOnGnd[LT[LegIndex]] = false;//Very simple at first, just keep lifting legs..
						}
					}
					else {//STANCE PHASE
						//Drive the GaitRotY in the oposite direction than we do in swing phase
						//The GaitPosX and GaitPosZ are controlled by the balance algorithm.
						//Speedfactor in Stance phase = (Swing Phase)/(Stance Phase) = (Swing Phase) / (60- Swing Phase)
						//Should I try this method for the PosX and PosZ, introduce the GaitPos again?
						//BUG BUG
						/*if ((g_InControlState.TravelLength.y < -1) && DecRYok){//Using DecRYok since the stance leg move in oposite direction. 
							GaitRotY[LT[LegIndex]] += (RotSpeed*((float)(DynaZGait[GT][6] / (float)(60 - DynaZGait[GT][6]))));//speed make it a variable
						}
						else if ((g_InControlState.TravelLength.y > 1) && IncRYok){//
							GaitRotY[LT[LegIndex]] -= (RotSpeed*((float)(DynaZGait[GT][6] / (float)(60 - DynaZGait[GT][6]))));
						}*/
						//Ex: I.TL.y=10 means GaitRotY must dec towards -10
						//if one leg has GaitRotY <-10 and another is >-10 we have a problem. What direction to move? All legs must move in same direction!
            //The sign of I.TL decide what direction to move, of course..
            //Need to check all legs on ground first to see if all conditions are met
						if ((GaitRotY[LT[LegIndex]]<(-g_InControlState.TravelLength.y - RotSpeed)) && DecRYok && (g_InControlState.TravelLength.y < 0)){//Using DecRYok since the stance leg move in oposite direction. 
							// If true Increment GaitRotY
              //CCW, Do nothing yet until all legs on ground are checked
						}
           //Could just use an OR statement between these two conditions
						else if ((GaitRotY[LT[LegIndex]]>(-g_InControlState.TravelLength.y + RotSpeed)) && IncRYok && (g_InControlState.TravelLength.y > 0)){
							// If true Decrement GaitRotY
              //CW, ditto
						}
            else{//No rotation on any leg if one of the legs is out of current range
              GaitRotYok = false;
              }
						//Call a routine to check the IKSolution for the current leg. If warning or error we get an indicator of to large values on LegPosX,Z or GaitRotY (maybe debug this?)
						//we can avoid this by limiting the legs at swing phase, similar to what we do during anti leg colision
					}
				}

				
		}//end for loop
		
		
    if (GaitRotYok) {//Now we can rotate all legs on ground
			RotSpeed = ((abs(g_InControlState.TravelLength.y)) / (float)ESSP)*(((float)DynaZGait[GT][6] / (60 - (float)DynaZGait[GT][6])));//maybe just calc it ones first, since all legs must have same speed
     for (LegIndex = 1; LegIndex <= CNT_LEGS; LegIndex++) {
      if (LegOnGnd[LT[LegIndex]] == true){
        if (g_InControlState.TravelLength.y < -1){
					GaitRotY[LT[LegIndex]] += RotSpeed;//(RotSpeed*((float)(DynaZGait[GT][6] / (float)(60 - DynaZGait[GT][6]))));//speed make it a variable
            }
            else if (g_InControlState.TravelLength.y > 1){//
							GaitRotY[LT[LegIndex]] -= RotSpeed;//(RotSpeed*((float)(DynaZGait[GT][6] / (float)(60 - DynaZGait[GT][6]))));
            }
       } 
     }
    } 

#ifdef DEBUG_Gait_Pos
			DBGSerial.print(CGP, DEC);
			DBGSerial.print(" ");
			DBGSerial.print((int)g_InControlState.TravelLength.y, DEC);
			DBGSerial.print(":[");
			DBGSerial.print((int)GaitRotY[0], DEC);//debug one leg RR
			DBGSerial.println("]");
		
#endif
		
#ifdef DEBUG_Gait	
		DBGSerial.print("TLZ: ");
		DBGSerial.print((int)g_InControlState.TravelLength.z, DEC);
		DBGSerial.print(" ESSP: ");
		DBGSerial.print(ESSP, DEC);
		DBGSerial.print(" Bug:");
		DBGSerial.print(CycleCGPBugCnt, DEC);
		DBGSerial.print(" ");
		DBGSerial.print(CGP, DEC);
		DBGSerial.println(" ");
#endif		

#ifdef DEBUG_Gait_Speed
		DBGSerial.print("ZS:");
		DBGSerial.print(ZSpeed, DEC);
		DBGSerial.print(" TZ:");
		DBGSerial.print(g_InControlState.TravelLength.z, DEC);
		DBGSerial.print(" ESSP:");
		DBGSerial.println(ESSP, DEC);
#endif
		
	
	
}
//--------------------------------------------------------------------
//A routine that make sure no feet exceeds their legal positions
//[Check gait positions limits]
void CheckGaitPosLimits(byte Leg_index){
	//Should I also check for IK warnings first? If no warnings, just return from this function?
	//Start simple by limiting the legs on ground, say +/- 100mm?
	if ((IKSolution[Leg_index] > 0)){ //removed || (abs(GaitPosX[Leg_index])> 100) || (abs(GaitPosZ[Leg_index])> 100) || (abs(GaitRotY[Leg_index])> 30)
#ifdef DEBUG_GaitPosLimits	
		
		//MSound(1, 5, 2000);//make some sound
		DBGSerial.print("Leg:");
		DBGSerial.print(Leg_index, DEC);
		DBGSerial.print(" X:");
		DBGSerial.print((int)GaitPosX[Leg_index], DEC);
		DBGSerial.print(" Z:");
		DBGSerial.print((int)GaitPosZ[Leg_index], DEC);
		DBGSerial.print(" Y:");
		DBGSerial.print((int)GaitRotY[Leg_index], DEC);
		DBGSerial.print(" IKsol:");
		DBGSerial.println(IKSolution[Leg_index], DEC);
#endif
		//Now we must determine what to do, remember that we are only checking the legs on ground (stance phase)
		//The legs in swing phase shouldn't exceed any limits because they are limited by the controlinputs (hopefully!)
		
			//Always limit the largest value
			if (abs(GaitPosX[Leg_index]) > abs(GaitPosZ[Leg_index])){
				if (GaitPosX[Leg_index] > 0){
					DecXok = false; //we must inhibit movement of the legs in swing phase in oposite direction, therefore DecXok
				}
				else{
					IncXok = false;
				}
			}
			else {//GaitPosZ is largest:
				if (GaitPosZ[Leg_index] > 0){
					DecZok = false;
				}
				else{
					IncZok = false;
				}
			}
		
	}

}
//--------------------------------------------------------------------
//A rather simple anti leg colision function
//[Anti Leg Colision]
void AntiLegColision(byte Leg_index){
	//Notes: Use a switch case for the current leg (better overview) and analyze the ABSFootPosX/Z of the current leg (during swing phase) and the two neighbour legs.
	//Check if there are conflicts and then decide what restrictions to do
	//Example of default absolute leg positions: [RR] X:-165 Z:256 [RM] X:-246 Z:-28 [RF] X:-162 Z:-191 [LR] X:162 Z:222 [LM] X:246 Z:34 [LF] X:165 Z:-296
	//Simple check rules:
	//For the rear legs we check if we have Z-coord conflict with the middle legs and X-coord conflict with the other rear leg
	//For the middle legs we check if we have Z-coord conflict with the rear and front legs
	//For the front legs we check if we have Z-coord conflict with the middle leg and X-coord conflict with the other front leg
	//Might also check the other coord to see if they are more than Safetymargin away from each other, we could reduce the Safetymargin on the main control.?
	//There are more sophisticated ways to do this, but I hope this should be sufficient.
	//The shape of the body and init position matter. These rules are most effective for an inline body
	//For example walking sideways should not cause any leg colisions, but forward/backward and rotational walking does cause conflicts
	//Note about GaitRotY: CW = decrementation of GaitRotY, CCW = incrementation
	//Make boolean variables that allow incrementation or decrementation of GaitPosX/Z and GaitRotY, ex IncXok, DecXok, IncZok, DecZok, IncRYok, DecRYok
	
	float Safetymargin = 65;//This is the width of the leg + some more (about 57mm).. make this a global definition
	switch (Leg_index){
	case cRR:
			//For the Right rear leg we check Z conflict with Right middle and X conflict with Left rear
			//If Z conflict we must block further decrementation of LegPosZ and incrementation of GaitRotY. If X conflict we must block incrementation of GaitPosX and decrementation of GaitRotY.
		if (ABSFootPosZ[Leg_index] < (ABSFootPosZ[cRM] + Safetymargin)){
			DecZok = false;
			IncRYok = false;
		}
		if (ABSFootPosX[Leg_index] > (ABSFootPosX[cLR] - Safetymargin)){
			IncXok = false;
			DecRYok = false;
		}
		break;
	case cRM:
		//For the Right middle leg we check Z conflict with Right front and rear leg
		//If Z conflict with front leg we must block further decrementation of LegPosZ and incrementation of GaitRotY. And vice versa when conflict with the rear leg
		if (ABSFootPosZ[Leg_index] < (ABSFootPosZ[cRF] + Safetymargin)){
			DecZok = false;
			IncRYok = false;
		}
		if (ABSFootPosZ[Leg_index] > (ABSFootPosZ[cRR] - Safetymargin)){
			IncZok = false;
			DecRYok = false;
		}
		break;
	case cRF:
		//For the Right front leg we check Z conflict with Right middle and X conflict with left front
		//If Z conflict with right middle we must block further incrementation of LegPosZ and decrementation of GaitRotY. 
		//If X conflict with left front we must block incrementation of GaitPosX and GaitRotY.
		if (ABSFootPosZ[Leg_index] > (ABSFootPosZ[cRM] - Safetymargin)){
			IncZok = false;
			DecRYok = false;
		}
		if (ABSFootPosX[Leg_index] > (ABSFootPosX[cLF] - Safetymargin)){
			IncXok = false;
			IncRYok = false;
		}
		break;
	case cLR:
		//For the left rear leg we check Z conflict with left middle and X conflict with right rear
		//If Z conflict with left middle we must block further decrementation of LegPosZ and GaitRotY. 
		//If X conflict with right rear we must block decrementation of GaitPosX and incrementation of GaitRotY.
		if (ABSFootPosZ[Leg_index] < (ABSFootPosZ[cLM] + Safetymargin)){
			DecZok = false;
			DecRYok = false;
		}
		if (ABSFootPosX[Leg_index] < (ABSFootPosX[cRR] + Safetymargin)){
			DecXok = false;
			IncRYok = false;
		}
		break;
	case cLM:
		//For the left middle leg we check Z conflict with left front and rear leg
		//If Z conflict with front leg we must block further decrementation of LegPosZ and GaitRotY. And vice versa when conflict with the rear leg
		if (ABSFootPosZ[Leg_index] < (ABSFootPosZ[cLF] + Safetymargin)){
			DecZok = false;
			DecRYok = false;
		}
		if (ABSFootPosZ[Leg_index] > (ABSFootPosZ[cLR] - Safetymargin)){
			IncZok = false;
			IncRYok = false;
		}
		break;
	case cLF:
		//For the left front leg we check Z conflict with left middle and X conflict with right front
		//If Z conflict with left middle we must block further incrementation of LegPosZ and GaitRotY. 
		//If X conflict with right front we must block decrementation of GaitPosX and GaitRotY.
		if (ABSFootPosZ[Leg_index] > (ABSFootPosZ[cLM] - Safetymargin)){
			IncZok = false;
			IncRYok = false;
		}
		if (ABSFootPosX[Leg_index] < (ABSFootPosX[cRF] + Safetymargin)){
			DecXok = false;
			DecRYok = false;
		}
		break;
	}
}
//--------------------------------------------------------------------
//[SINGLE LEG CONTROL]
void SingleLegControl(void)
{
#ifdef OPT_SINGLELEG
	
#ifdef DBGSerial
	//DBGSerial.println(" ");//
#endif  
  //Check if all legs are down
  AllDown = (LegPosY[cRF]==(short)pgm_read_word(&cInitPosY[cRF])) && 
    (LegPosY[cRR]==(short)pgm_read_word(&cInitPosY[cRR])) && 
    (LegPosY[cLR]==(short)pgm_read_word(&cInitPosY[cLR])) && 
#ifndef QUADMODE
    (LegPosY[cRM]==(short)pgm_read_word(&cInitPosY[cRM])) && 
    (LegPosY[cLM]==(short)pgm_read_word(&cInitPosY[cLM])) && 
#endif  
    (LegPosY[cLF]==(short)pgm_read_word(&cInitPosY[cLF]));

  if (g_InControlState.SelectedLeg<= (CNT_LEGS-1)) {
    if (g_InControlState.SelectedLeg!=PrevSelectedLeg) {
      if (AllDown) { //Lift leg a bit when it got selected
        LegPosY[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosY[g_InControlState.SelectedLeg])-20;

        //Store current status
        PrevSelectedLeg = g_InControlState.SelectedLeg;
      } 
      else {//Return prev leg back to the init position
        LegPosX[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosX[PrevSelectedLeg]);
        LegPosY[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosY[PrevSelectedLeg]);
        LegPosZ[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosZ[PrevSelectedLeg]);
      }
    } 
		else if (!g_InControlState.fSLHold) {
			LegPosY[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosY[g_InControlState.SelectedLeg]) + g_InControlState.SLLeg.y;

			LegPosZ[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosZ[g_InControlState.SelectedLeg]) + g_InControlState.SLLeg.z;
			if (g_InControlState.SelectedLeg >= (CNT_LEGS / 2)) {//Must be reversed for left legs
				LegPosX[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosX[g_InControlState.SelectedLeg]) - g_InControlState.SLLeg.x;
			}
			else{
				LegPosX[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosX[g_InControlState.SelectedLeg]) + g_InControlState.SLLeg.x;
			}


			/*DBGSerial.print(LegPosX[g_InControlState.SelectedLeg], DEC);
			DBGSerial.print("_");
			DBGSerial.println(LegPosY[g_InControlState.SelectedLeg], DEC);
			DBGSerial.print("_");
			DBGSerial.print(" ");
			DBGSerial.print(LegPosZ[g_InControlState.SelectedLeg], DEC);
			DBGSerial.print(" ");
			DBGSerial.print(g_InControlState.SLyawRot, DEC);
			DBGSerial.print(" ");*/
			boolean UpdatedYawAngle;
			if (LegRotY[g_InControlState.SelectedLeg]<(g_InControlState.SLyawRot - 0.5)){//a margin off 0.1 maybe test other values
				//Do the new leg Y rotation test here, using a Yaw rotation matrix for each leg
				LegRotY[g_InControlState.SelectedLeg] += 0.5;//speed
				UpdatedYawAngle = true;
			}
			else if (LegRotY[g_InControlState.SelectedLeg]>(g_InControlState.SLyawRot + 0.5)){

				LegRotY[g_InControlState.SelectedLeg] -= 0.5;
				UpdatedYawAngle = true;
			}
			else{
				UpdatedYawAngle = false;
			}
			if ((g_InControlState.SelectedLeg < (CNT_LEGS / 2)) && UpdatedYawAngle){//Right legs
				LegYawRotate(-(short)pgm_read_word(&cInitPosX[g_InControlState.SelectedLeg]), (short)pgm_read_word(&cInitPosZ[g_InControlState.SelectedLeg]), LegRotY[g_InControlState.SelectedLeg], g_InControlState.SelectedLeg);// inc by 0,1 deg
			}
			else if (UpdatedYawAngle){//left legs
				LegYawRotate((short)pgm_read_word(&cInitPosX[g_InControlState.SelectedLeg]), (short)pgm_read_word(&cInitPosZ[g_InControlState.SelectedLeg]), LegRotY[g_InControlState.SelectedLeg], g_InControlState.SelectedLeg);// inc by 0,1 deg
			}


			//PosX value must be reversed on right leg
			/*DBGSerial.print(LegPosX[g_InControlState.SelectedLeg], DEC);2
			DBGSerial.print(" ");
			DBGSerial.print(LegPosZ[g_InControlState.SelectedLeg], DEC);
			DBGSerial.print(" ");
			DBGSerial.println(LegRotY[g_InControlState.SelectedLeg], DEC);*/
		}
  } 
  else {//All legs to init position
    if (!AllDown) {
      for(LegIndex = 0; LegIndex <= (CNT_LEGS-1);LegIndex++) {
        LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);
        LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
        LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);
				LegRotY[LegIndex] = 0;//should we do this here?
      }
    } 
    if (PrevSelectedLeg!=255)
      PrevSelectedLeg = 255;
  }
#endif
}
 


//--------------------------------------------------------------------
//[BalCalcOneLeg]
#ifndef PI
#define PI 3.14159265
#endif

void BalCalcOneLeg (float PosX, float PosZ, float PosY, byte BalLegNr)
{
  float            cpr_x;                        //Final X value for centerpoint of rotation
  float            cpr_y;                    //Final Y value for centerpoint of rotation
  float            cpr_z;                    //Final Z value for centerpoint of rotation

	float LegRotX;
	float LegRotY;
	float LegRotZ;

#ifdef QUADMODE
  if (g_InControlState.gaitCur.COGAngleStep1 == 0) {  // In Quad mode only do those for those who don't support COG Balance...
#endif

    //Calculating totals from center of the body to the feet
    cpr_z = (short)pgm_read_word(&cOffsetZ[BalLegNr]) + PosZ;
    cpr_x = (short)pgm_read_word(&cOffsetX[BalLegNr]) + PosX;
    cpr_y = 260 + PosY;        //200 + PosY; Quick fix, more debug using the value 150 to lower the centerpoint of rotation 'g_InControlState.BodyPos.y +

    TotalTransY += PosY;
    TotalTransZ += cpr_z;
    TotalTransX += cpr_x;

		if (g_InControlState.BalanceMode == 1){
			LegRotY = atan2f(cpr_z, cpr_x)*180.0 / PI;
			TotalYBal1 += LegRotY;
#ifdef DEBUG
			if (g_fDebugOutput) {
				DBGSerial.print(" ");
				DBGSerial.print(cpr_x, DEC);
				DBGSerial.print(":");
				DBGSerial.print(cpr_y, DEC);
				DBGSerial.print(":");
				DBGSerial.print(cpr_z, DEC);
				DBGSerial.print(":");
				DBGSerial.print(TotalYBal1, DEC);
			}    
#endif
			
			LegRotZ = (atan2f(cpr_y, cpr_x)*180.0) / PI - 90.0;  //Rotate balance circle 90.0 deg
			TotalZBal1 += LegRotZ;
			//if (!(BalLegNr == 1) && !(BalLegNr == 4)){//Test, don't do Xbal for the middle legs bug bug
				LegRotX = (atan2f(cpr_y, cpr_z)*180.0) / PI - 90.0;//Rotate balance circle 90.0 deg
				TotalXBal1 += LegRotX;
			//}
		}
#ifdef DEBUG_BAL_ONELEG
		//if (BalLegNr == 1){//only debug Right middle leg
			DBGSerial.print("|L:");
			DBGSerial.print(BalLegNr, DEC);
			DBGSerial.print("_P");
			DBGSerial.print((int)cpr_x, DEC);
			DBGSerial.print(" ");
			DBGSerial.print((int)cpr_y, DEC);
			DBGSerial.print(" ");
			DBGSerial.print((int)cpr_z, DEC);
			DBGSerial.print("_B:");
			DBGSerial.print((int)LegRotX, DEC);
			DBGSerial.print(" ");
			DBGSerial.print((int)LegRotY, DEC);
			DBGSerial.print(" ");
			DBGSerial.print((int)LegRotZ, DEC);
			
		//}
#endif
#ifdef QUADMODE
  }
#endif  

}  

//--------------------------------------------------------------------
//[BalanceBody]
void BalanceBody(void)
{
	
    TotalTransZ = TotalTransZ/BalanceDivFactor ;
    TotalTransX = TotalTransX/BalanceDivFactor;
    TotalTransY = TotalTransY/BalanceDivFactor;
		if (g_InControlState.GaitType == 3){//Reduce the TotalTransY a little when tripod walking
			TotalTransY = 5* TotalTransY / 6;
		}
		if (g_InControlState.BalanceMode == 1){

			if (TotalYBal1 > 0)        //Rotate balance circle by +/- 180 deg
				TotalYBal1 -= 180.0;
			else
				TotalYBal1 += 180.0;

			if (TotalZBal1 < -180.0)    //Compensate for extreme balance positions that causes overflow
				TotalZBal1 += 360;
			if (TotalZBal1 < -180.0)    //Compensate for extreme balance positions that causes overflow
				TotalZBal1 += 360;

			if (TotalXBal1 < -180.0)    //Compensate for extreme balance positions that causes overflow
				TotalXBal1 += 360;
			if (TotalXBal1 < -180.0)    //Compensate for extreme balance positions that causes overflow
				TotalXBal1 += 360;

			//Balance rotation
			TotalYBal1 = -TotalYBal1 / BalanceDivFactor;
			TotalXBal1 = -TotalXBal1 / BalanceDivFactor;
			TotalZBal1 = TotalZBal1 / BalanceDivFactor;
		}
		if (g_InControlState.BalanceMode == 0){//Low balance mode
			TotalTransY = 0;
			
		}
		/*else if (g_InControlState.BalanceMode == 1){//Half balance mode
			TotalTransY = 0;// a more reduced balance mode
		}*/
    //For safety, restrict the Balance values. Also avoid rapid changes.

#ifdef DEBUG_BAL_ONELEG
    //if (g_fDebugOutput) {
      DBGSerial.print(" ||T: ");
			DBGSerial.print((int)TotalTransX, DEC);
      DBGSerial.print(" ");
			DBGSerial.print((int)TotalTransY, DEC);
      DBGSerial.print(" ");
			DBGSerial.print((int)TotalTransZ, DEC);
      DBGSerial.print(" TB: ");
			DBGSerial.print((int)TotalXBal1, DEC);
      DBGSerial.print(" ");
			DBGSerial.print((int)TotalYBal1, DEC);
      DBGSerial.print(" ");
			DBGSerial.println((int)TotalZBal1, DEC);
    //}
#endif
			
}



//=============================================================================
// GetSinCos:  Get the sinus and cosinus from the angle +/- multiple circles
// angleDeg     - Input Angle in degrees
// cosA         - Output Sinus of angleDeg
// sinA         - Output Cosinus of angleDeg
//=============================================================================

void GetSinCos(float angleDeg){
  float absangleDeg; // Absolute value of the Angle in Degrees
  float angleRad;

  // Get the absolute value of angleDeg
  absangleDeg = abs ( angleDeg );

  // Shift rotation to a full circle of 360 deg -> angleDeg // 360
  if ( angleDeg < 0.0 )
  {
    angleDeg = 360.0 - ( absangleDeg - 360.0 *  ( ( int ) ( absangleDeg / 360.0 ) ) ); // Negative values
  }    

  else
  {
    angleDeg = absangleDeg - 360.0 * ( ( int ) ( absangleDeg / 360.0 ) ); // Positive values
  }

  if ( angleDeg < 180.0 )
  {
    angleDeg = angleDeg - 90.0;
    angleRad =  angleDeg * PI / 180.0; // Convert degree to radials
    sinA =  cosf ( angleRad );
    cosA = -sinf ( angleRad );
  }

  else
  {
    angleDeg = angleDeg - 270.0; // Subtract 270 to shift range
    angleRad = angleDeg * PI / 180.0; // Convert degree to radials

    sinA = -cosf ( angleRad );
    cosA =  sinf ( angleRad );
  }
}    
//--------------------------------------------------------------------
//[LegYawRotate]
//Perform a single leg rotation using a matrix
void LegYawRotate(float PosX, float PosZ, float YawAngle, byte LegIndex){
	//Could reuse BodyFK though with some mods.
	float            cpr_x;                  //Final X value for centerpoint of rotation
	float            cpr_z;                  //Final Z value for centerpoint of rotation
	float						LegRotPosX;
	float						LegRotPosZ;
	//Calculating totals from center of the body to the feet 104 104
	cpr_x = (short)pgm_read_word(&cOffsetX[LegIndex]) + PosX;
	cpr_z = (short)pgm_read_word(&cOffsetZ[LegIndex]) + PosZ;
	GetSinCos(YawAngle);
	//Calcualtion of rotation matrix :
	//Y rotation only(Yaw)
	//LegRotPosX = (CPR_X - (CPR_X*CosA - CPR_Z*SinA))
	//LegRotPosZ = (CPR_Z - (CPR_X*SinA + CPR_Z*CosA))
	LegRotPosX = cpr_x - (cpr_x*cosA - cpr_z*sinA); //Really don't need the LegRotPosX variable here, just put it in the LegRotPos calc
	LegRotPosZ = cpr_z - (cpr_x*sinA + cpr_z*cosA);
	//LegPosX[LegIndex] += LegRotPosX; //
	//LegPosZ[LegIndex] -= LegRotPosZ; //
	LegRotYxPos[LegIndex] = LegRotPosX;
	LegRotYzPos[LegIndex] = LegRotPosZ;
	/*DBGSerial.print(cpr_x,DEC);
	DBGSerial.print(" ");
	DBGSerial.print(cpr_z, DEC);
	DBGSerial.print(" ");
	DBGSerial.print(LegRotPosX, DEC);
	DBGSerial.print(" ");
	DBGSerial.print(LegRotPosZ, DEC);
	DBGSerial.print(" ");
	DBGSerial.println(YawAngle, DEC);*/
	
}

//--------------------------------------------------------------------
//(BODY FORWARD KINEMATICS) 
//BodyRotX         - Global Input pitch of the body 
//BodyRotY         - Global Input rotation of the body 
//BodyRotZ         - Global Input roll of the body 
//RotationY         - Input Rotation for the gait 
//PosX            - Input position of the feet X 
//PosZ            - Input position of the feet Z 
//SinB                  - Sin buffer for BodyRotX
//CosB               - Cos buffer for BodyRotX
//SinG                  - Sin buffer for BodyRotZ
//CosG               - Cos buffer for BodyRotZ
//BodyFKPosX         - Output Position X of feet with Rotation 
//BodyFKPosY         - Output Position Y of feet with Rotation 
//BodyFKPosZ         - Output Position Z of feet with Rotation
void BodyFK (float PosX, float PosZ, float PosY, float RotationY, byte BodyIKLeg, boolean Mode)
{
  float            sinB;                   //Sin buffer for BodyRotX calculations
  float            cosB;                   //Cos buffer for BodyRotX calculations
  float            sinG;                   //Sin buffer for BodyRotZ calculations
  float            cosG;                   //Cos buffer for BodyRotZ calculations
  float            cpr_x;                  //Final X value for centerpoint of rotation
  float            cpr_y;                   //Final Y value for centerpoint of rotation
  float            cpr_z;                   //Final Z value for centerpoint of rotation

	//Three options/mode:
	//Mode = 0 : Do Balance rotation only, cpr_xyz = default
	//Mode = 1 : Do all the other rotations; GaitRotationY, Body rotation, DW rotation
	//(3. All rotations at the same time (TA-mode off) )

  //Successive global rotation matrix: 
  //Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate 
  //Sinus Alfa = SinA, cosinus Alfa = cosA. and so on... 

  //Calculate sinus and cosinus for each rotation: 
	if (!Mode) {
		//Calculating totals from center of the body to the feet, no offset when doing balance rotation only
		cpr_x = (short)pgm_read_word(&cOffsetX[BodyIKLeg]) + PosX;

		cpr_y = PosY;         //Define centerpoint for rotation along the Y-axis

		cpr_z = (short)pgm_read_word(&cOffsetZ[BodyIKLeg]) + PosZ;

		GetSinCos(TotalXBal1);//Zenta, Fixed. The balance factor should not be divided by 10. Maybe remove the /10 and adjust the Control values instead
	}
	else{
		//Calculating totals from center of the body to the feet 
		cpr_x = (short)pgm_read_word(&cOffsetX[BodyIKLeg]) + PosX + g_InControlState.BodyRotOffset.x;

		cpr_y = PosY + g_InControlState.BodyRotOffset.y;         //Define centerpoint for rotation along the Y-axis

		cpr_z = (short)pgm_read_word(&cOffsetZ[BodyIKLeg]) + PosZ + g_InControlState.BodyRotOffset.z;

		GetSinCos(g_InControlState.BodyRot1.x / 10.0);//GetSinCos((g_InControlState.BodyRot1.x / 10.0) + TotalXBal1);
	}
	sinG = sinA;
  cosG = cosA;

	if (!Mode) {
		GetSinCos(TotalZBal1);//Zenta, Same fix here 
	}
	else{
		GetSinCos(g_InControlState.BodyRot1.z / 10.0);//GetSinCos((g_InControlState.BodyRot1.z / 10.0) + TotalZBal1);
	}
  sinB = sinA;
  cosB = cosA;

#ifdef OPT_WALK_UPSIDE_DOWN
	if (!Mode) {
		GetSinCos(TotalYBal1);//Zenta, And here
	}
	else {
		if (g_fRobotUpsideDown)
			GetSinCos(((g_InControlState.BodyRot1.y + g_InControlState.DWrotY + (-RotationY * c1DEC)) / 10.0));
		else
			GetSinCos(((g_InControlState.BodyRot1.y + g_InControlState.DWrotY + (RotationY * c1DEC)) / 10.0));
	}
#else
	if (!Mode) {
		GetSinCos(TotalYBal1);//Zenta, And here
	}
	else{
		GetSinCos(((g_InControlState.BodyRot1.y + g_InControlState.DWrotY + (RotationY*c1DEC)) / 10.0));//GetSinCos(((g_InControlState.BodyRot1.y + g_InControlState.DWrotY + (RotationY*c1DEC)) / 10.0) + TotalYBal1);
	}
#endif
  

  // Calculation of rotation matrix:
	if (!Mode) {//Do Balance rotations
		BalFKPosX = cpr_x - (cpr_x * cosA * cosB - cpr_z * cosB * sinA + cpr_y * sinB);
		BalFKPosZ = cpr_z - (cpr_x * cosG * sinA + cpr_x * cosA * sinB * sinG + cpr_z * cosA * cosG - cpr_z * sinA * sinB * sinG - cpr_y * cosB * sinG);
		BalFKPosY = cpr_y - (cpr_x * sinA * sinG - cpr_x * cosA * cosG * sinB + cpr_z * cosA * sinG + cpr_z * cosG * sinA * sinB + cpr_y * cosB * cosG);
	}
	else{
		BodyFKPosX = cpr_x - (cpr_x * cosA * cosB - cpr_z * cosB * sinA + cpr_y * sinB);
		BodyFKPosZ = cpr_z - (cpr_x * cosG * sinA + cpr_x * cosA * sinB * sinG + cpr_z * cosA * cosG - cpr_z * sinA * sinB * sinG - cpr_y * cosB * sinG);
		BodyFKPosY = cpr_y - (cpr_x * sinA * sinG - cpr_x * cosA * cosG * sinB + cpr_z * cosA * sinG + cpr_z * cosG * sinA * sinB + cpr_y * cosB * cosG);
	}
}  


//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//feetPosX            - Input position of the Feet X
//feetPosY            - Input position of the Feet Y
//feetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//FemurAngle           - Output Angle of Femur in degrees
//TibiaAngle           - Output Angle of Tibia in degrees
//CoxaAngle            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
#define square(x) ((x)*(x))

void LegIK (float feetPosX, float feetPosY, float feetPosZ, int legIndex)
{
  float            IKSW2;                 //Length between Shoulder and Wrist, decimals = 2
  float            IKA14;                  //Angle of the line S>W with respect to the ground in radians, decimals = 4
  float            IKA24;                  //Angle of the line S>W with respect to the femur in radians, decimals = 4
  float            feetPosXZ;              //Diagonal direction from Input X and Z
	if (legIndex < 3) {//Subtract on left legs
		ABSFootPosX[legIndex] = (short)pgm_read_word(&cOffsetX[legIndex]) - feetPosX;
	}
	else{
		ABSFootPosX[legIndex] = (short)pgm_read_word(&cOffsetX[legIndex]) + feetPosX; //Use this value to check if the legs get in conflict
	}
	ABSFootPosZ[legIndex] = (short)pgm_read_word(&cOffsetZ[legIndex]) + feetPosZ;
#ifdef Debug_ABSPos
	DBGSerial.print(" [");
	DBGSerial.print(legIndex, DEC);
	DBGSerial.print("] X:");
	DBGSerial.print((int)ABSFootPosX[legIndex],DEC);
	DBGSerial.print(" Z:");
	DBGSerial.print((int)ABSFootPosZ[legIndex], DEC);
	if (legIndex == 5){
		DBGSerial.println(" ");
	}
#endif
#ifdef c4DOF
  // these were shorts...
  float            TarsOffsetXZ;           //Vector value \ ;
  float            TarsOffsetY;            //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
  float            TarsToGroundAngle = 0;  //Angle between tars and ground. Note: the angle are 0 when the tars are perpendicular to the ground
  float            TGA_A_H4;
  float            TGA_B_H3;
#else
#define TarsOffsetXZ 0      // Vector value
#define TarsOffsetY  0      //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors
#endif
  //Calculate IKCoxaAngle and feetPosXZ
  CoxaAngle[legIndex] = atan2f( feetPosZ, feetPosX ) * 180.0 / PI + cCoxaAngle[legIndex];

  //Length between the Coxa and tars [foot]
  feetPosXZ = sqrtf ( square(feetPosX) + square(feetPosZ) );
#ifdef c4DOF
  // Some legs may have the 4th DOF and some may not, so handle this here...
  //Calc the TarsToGroundAngle:
  if (TARSLENGTH(legIndex)) {    // We allow mix of 3 and 4 DOF legs...
    TarsToGroundAngle = -cTarsConst + cTarsMulti*feetPosY + (feetPosXZ*cTarsFactorA) - (feetPosXZ*feetPosY)/cTarsFactorB;
#ifdef DEBUG
    if (g_fDebugOutput && g_InControlState.fRobotOn) {
      DBGSerial.print(" @");
      DBGSerial.print(TarsToGroundAngle, 4);
    }   
#endif 
    if (feetPosY < 0)                   //Always compensate TarsToGroundAngle when feetPosY it goes below zero
      TarsToGroundAngle = TarsToGroundAngle - (feetPosY*cTarsFactorC);     //TGA base, overall rule
    if (TarsToGroundAngle > 40)
      TGA_B_H3 = 20 + (TarsToGroundAngle/2);
    else
      TGA_B_H3 = TarsToGroundAngle;

    if (TarsToGroundAngle > 30)
      TGA_A_H4 = 24 + (TarsToGroundAngle/5);
    else
      TGA_A_H4 = TarsToGroundAngle;

    if (feetPosY > 0)                   //Only compensate the TarsToGroundAngle when it exceed 30 deg (A, H4 PEP note)
      TarsToGroundAngle = TGA_A_H4;
    else if (((feetPosY <= 0) & (feetPosY > -10))) // linear transition between case H3 and H4 (from PEP: H4-K5*(H3-H4))
      TarsToGroundAngle = TGA_A_H4 - (feetPosY*(TGA_B_H3-TGA_A_H4));
    else                                  //feetPosY <= -10, Only compensate TGA1 when it exceed 40 deg
    TarsToGroundAngle = TGA_B_H3;

    //Calc Tars Offsets:
    TarsOffsetXZ = sin(TarsToGroundAngle*PI/180.0) * TARSLENGTH(legIndex);
    TarsOffsetY = cos(TarsToGroundAngle*PI/180.0) * TARSLENGTH(legIndex);
  } 
  else {
    TarsOffsetXZ = 0;
    TarsOffsetY = 0;
  }
#ifdef DEBUG
  if (g_fDebugOutput && g_InControlState.fRobotOn) {
    DBGSerial.print(" ");
    DBGSerial.print(TarsOffsetXZ, 4);
    DBGSerial.print(" ");
    DBGSerial.print(TarsOffsetY, 4);
  }    
#endif
#endif

  //Using GetAtan2 for solving IKA1 and IKSW
  //IKA14 - Angle between SW line and the ground in radians
  IKA14 = atan2f(feetPosXZ-COXALENGTH(legIndex)-TarsOffsetXZ, feetPosY-TarsOffsetY);

  //IKSW2 - Length between femur axis and tars
  IKSW2 = sqrtf ( square(feetPosXZ-COXALENGTH(legIndex)-TarsOffsetXZ) + square(feetPosY-TarsOffsetY ) );

	//Set the Solution quality
	//Zenta, finally using this for something useful ..
	//Have one IKSolution indicator for each leg
	if (IKSW2 < (FEMURLENGTH(legIndex) + TIBIALENGTH(legIndex) - 15))//maybe try other values?
		IKSolution[legIndex] = 0;
	else
	{
		if (IKSW2 < (FEMURLENGTH(legIndex) + TIBIALENGTH(legIndex)))
			IKSolution[legIndex] = 1;
		else
			IKSolution[legIndex] = 2;
	}

	if (IKSW2 > (FEMURLENGTH(legIndex) + TIBIALENGTH(legIndex))) IKSW2 = (FEMURLENGTH(legIndex) + TIBIALENGTH(legIndex));// Zenta, dirty fix. Should find a better way. But it works

  //IKA2 - Angle of the line S>W with respect to the femur in radians
  IKA24 = acosf ((square(FEMURLENGTH(legIndex)) - square(TIBIALENGTH(legIndex)) + square(IKSW2))
    / (2 * FEMURLENGTH(legIndex) * IKSW2));
#ifdef DEBUG_IK
	if ((legIndex == 3) && g_fDebugOutput && g_InControlState.fRobotOn) {
		DBGSerial.print(" IKSW:");
		DBGSerial.print(IKSW2, 4);
    DBGSerial.print(" ");
    DBGSerial.print(IKA14, 4);
    DBGSerial.print(" ");
    DBGSerial.print(IKA24, 4);
  }
#endif
  //IKFemurAngle
#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown)
	  FemurAngle[legIndex] = (IKA14 + IKA24) * 180.0 / PI - 90.0 + CFEMURHORNOFFSET1(legIndex) / 10.0;   //Inverted, up side down
  else
	  FemurAngle[legIndex] = -(IKA14 + IKA24) * 180.0 / PI + 90.0 + CFEMURHORNOFFSET1(legIndex) / 10.0;   //Normal
#else
  FemurAngle[legIndex] = -(IKA14 + IKA24) * 180.0 / PI + 90.0 + CFEMURHORNOFFSET1(legIndex)/10.0;   //Normal
#endif  


  //IKTibiaAngle
  IKA24 = acosf ((square(FEMURLENGTH(legIndex)) + square(TIBIALENGTH(legIndex)) - square(IKSW2)) 
    / (2 * FEMURLENGTH(legIndex) * TIBIALENGTH(legIndex)));   // rused IKA24
#ifdef DEBUG
  if (g_fDebugOutput && g_InControlState.fRobotOn) {
    DBGSerial.print("=");
    DBGSerial.print(IKA24, 4);
  }
#endif

  
#ifdef OPT_WALK_UPSIDE_DOWN
  if (g_fRobotUpsideDown)
	  TibiaAngle[legIndex] = (180.0 - IKA24*180.0 / PI + CTIBIAHORNOFFSET1(legIndex) / 10.0);//Full range tibia, wrong side (up side down)
  else
	  TibiaAngle[legIndex] = -(180.0 - IKA24*180.0 / PI + CTIBIAHORNOFFSET1(legIndex) / 10.0);//Full range tibia, right side (up side up)
#else
  TibiaAngle[legIndex] = -(90.0 - IKA24*180.0/PI + CTIBIAHORNOFFSET1(legIndex)/10.0); //Normal Tibia, should work for PhantomX V2 ?
#endif

#ifdef c4DOF
  //Tars angle
  if (TARSLENGTH(legIndex)) {    // We allow mix of 3 and 4 DOF legs...
    TarsAngle[legIndex] = (TarsToGroundAngle + FemurAngle[legIndex] - TibiaAngle[legIndex])
      + CTARSHORNOFFSET1(legIndex)/10.0;
  }
#endif

  
#ifdef DEBUG
  if (g_fDebugOutput && (g_InControlState.fRobotOn || g_InControlState.fPrev_RobotOn)) {
    DBGSerial.print("(");
    DBGSerial.print(feetPosX, 4);
    DBGSerial.print(",");
    DBGSerial.print(feetPosY, 4);
    DBGSerial.print(",");
    DBGSerial.print(feetPosZ, 4);
    DBGSerial.print(")=<");
    DBGSerial.print(CoxaAngle[legIndex], 2);
    DBGSerial.print(",");
    DBGSerial.print(FemurAngle[legIndex], 2);
    DBGSerial.print(",");
    DBGSerial.print(TibiaAngle[legIndex], 2 );
#ifdef c4DOF
    DBGSerial.print(",");
    DBGSerial.print(TarsAngle[legIndex], 2);
#endif
    DBGSerial.print(">");
		DBGSerial.print(IKSolution[legIndex], DEC);
    if (legIndex == (CNT_LEGS-1))
      DBGSerial.println();
  }
#endif  
}

//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
float CheckServoAngleBounds(short sID,  float sVal, float sMin, float sMax) {

    // Pull into simple function as so I can report errors on debug 
    // Note ID is bogus, but something to let me know which one.


	if (sVal < sMin) {
#ifdef DEBUG_CHKANGLE
		if (g_fDebugOutput) {
			DBGSerial.print(sID, 4);
			DBGSerial.print(" ");
			DBGSerial.print(sVal, 4);
			DBGSerial.print("<");
			DBGSerial.println(sMin, 4);
		}
#endif
        return sMin;
    }

    if (sVal > sMax) {
#ifdef DEBUG_CHKANGLE
			if (g_fDebugOutput) {
				DBGSerial.print(sID, DEC);
				DBGSerial.print(" ");
				DBGSerial.print(sVal, DEC);
				DBGSerial.print(">");
				DBGSerial.println(sMax, DEC);
			}
#endif
        return sMax;
    }
    return sVal;
  
}

//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void CheckAngles(void)
{
#ifndef SERVOS_DO_MINMAX
  short s = 0;      // BUGBUG just some index so we can get a hint who errored out
  for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++)
  {
        CoxaAngle[LegIndex]  = CheckServoAngleBounds(s++, CoxaAngle[LegIndex], cCoxaMin[LegIndex], cCoxaMax[LegIndex]);
        FemurAngle[LegIndex] = CheckServoAngleBounds(s++, FemurAngle[LegIndex], cFemurMin[LegIndex], cFemurMax[LegIndex]);
        TibiaAngle[LegIndex] = CheckServoAngleBounds(s++, TibiaAngle[LegIndex], cTibiaMin[LegIndex], cTibiaMax[LegIndex]);
#ifdef c4DOF
        if ((byte)(TARSLENGTH(legIndex))) {    // We allow mix of 3 and 4 DOF legs...
            TarsAngle[LegIndex] = CheckServoAngleBounds(s++, TarsAngle[LegIndex], cTarsMin[LegIndex], cTarsMax[LegIndex]);
    }
#endif
  }
#endif  
}


//--------------------------------------------------------------------
// SmoothControl (From Zenta) -  This function makes the body 
//            rotation and translation much smoother 
//--------------------------------------------------------------------
float SmoothControl (short CtrlMoveInp, float CtrlMoveOut, byte CtrlDivider)
{
	if (CtrlDivider == 1) return (float)CtrlMoveInp;//No smoothing at all
  if (CtrlMoveOut < ((float)CtrlMoveInp - 0.1))
		return CtrlMoveOut + abs((CtrlMoveOut - (float)CtrlMoveInp) / CtrlDivider);
	else if (CtrlMoveOut >((float)CtrlMoveInp + 0.1))
		return CtrlMoveOut - abs((CtrlMoveOut - (float)CtrlMoveInp) / CtrlDivider);

	return (float)CtrlMoveInp;
}


//--------------------------------------------------------------------
// GetLegsXZLength - 
//--------------------------------------------------------------------
word g_wLegsXZLength = 0xffff;
word GetLegsXZLength(void) 
{
    // Could save away or could do a little math on one leg... 
    if (g_wLegsXZLength != 0xffff)
        return g_wLegsXZLength;
        
    return sqrtf ( square(LegPosX[0]) + square (LegPosZ[0]) );
}


//--------------------------------------------------------------------
// AdjustLegPositions() - Will adjust the init leg positions to the
//      width passed in.
//--------------------------------------------------------------------
#ifndef MIN_XZ_LEG_ADJUST 
#define MIN_XZ_LEG_ADJUST (COXALENGTH(0))      // don't go inside coxa...
#endif

#ifndef MAX_XZ_LEG_ADJUST
#define MAX_XZ_LEG_ADJUST   (COXALENGTH(0)+TIBIALENGTH(0) + FEMURLENGTH(0)/4) 
#endif

void AdjustLegPositions(float XZLength) 
{
    //now lets see what happens when we change the leg positions...
    if (XZLength > MAX_XZ_LEG_ADJUST)
        XZLength = MAX_XZ_LEG_ADJUST;
    if (XZLength < MIN_XZ_LEG_ADJUST)
        XZLength = MIN_XZ_LEG_ADJUST;
        
    // see if same length as when we came in
    if (XZLength == g_wLegsXZLength)
        return;

    g_wLegsXZLength = XZLength;
    
        
    for (uint8_t LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        DBGSerial.print("(");
        DBGSerial.print(LegPosX[LegIndex], DEC);
        DBGSerial.print(",");
        DBGSerial.print(LegPosZ[LegIndex], DEC);
        DBGSerial.print(")->");
      }
#endif
#ifdef OPT_DYNAMIC_ADJUST_LEGS
      GetSinCos(g_InControlState.aCoxaInitAngle[LegIndex]);
#else
#ifdef cRRInitCoxaAngle    // We can set different angles for the legs than just where they servo horns are set...
      GetSinCos(cCoxaInitAngle[LegIndex]);
#else
      GetSinCos((short)pgm_read_word(&cCoxaAngle[LegIndex]));
#endif
#endif      
      LegPosX[LegIndex] = cosA * XZLength;  //Set start positions for each leg
      LegPosZ[LegIndex] = -sinA * XZLength;
#ifdef DEBUG
      if (g_fDebugOutput) {
        DBGSerial.print("(");
	DBGSerial.print(LegPosX[LegIndex], 4);
        DBGSerial.print(",");
	DBGSerial.print(LegPosZ[LegIndex], 4);
        DBGSerial.print(") ");
      }
#endif
    }
#ifdef DEBUG
    if (g_fDebugOutput) {
      DBGSerial.println("");
    }
#endif
   
}

//--------------------------------------------------------------------
// ResetLegInitAngles - This is used when we allow the user to 
// adjust the leg position angles.  This resets to what it was when the
// the program was started.
//--------------------------------------------------------------------
void ResetLegInitAngles(void)
{
#ifdef OPT_DYNAMIC_ADJUST_LEGS
    for (int LegIndex=0; LegIndex < CNT_LEGS; LegIndex++) {
#ifdef cRRInitCoxaAngle    // We can set different angles for the legs than just where they servo horns are set...
            g_InControlState.aCoxaInitAngle[LegIndex] = (short)(cCoxaInitAngle[LegIndex]);
#else
            g_InControlState.aCoxaInitAngle[LegIndex] = (short)(cCoxaAngle[LegIndex]);
#endif
    }
    g_wLegsXZLength = 0xffff;
#endif      
}

//--------------------------------------------------------------------
// ResetLegInitAngles - This is used when we allow the user to 
//--------------------------------------------------------------------
void RotateLegInitAngles (int iDeltaAngle)
{
#ifdef OPT_DYNAMIC_ADJUST_LEGS
    for (int LegIndex=0; LegIndex < CNT_LEGS; LegIndex++) {
        // We will use the cCoxaAngle array to know which direction the legs logically are
        // If the initial angle is 0 don't mess with.  Hex middle legs...
        if ((short)(cCoxaAngle[LegIndex]) > 0) 
            g_InControlState.aCoxaInitAngle[LegIndex] += iDeltaAngle;
         else if ((short)(cCoxaAngle[LegIndex]) < 0)
            g_InControlState.aCoxaInitAngle[LegIndex] -= iDeltaAngle;
        
        // Make sure we don't exceed some min/max angles.
        // Right now hard coded to +-70 degrees... Should probably load
        if (g_InControlState.aCoxaInitAngle[LegIndex] > 700)
            g_InControlState.aCoxaInitAngle[LegIndex] = 700;
        else if (g_InControlState.aCoxaInitAngle[LegIndex] < -700)
            g_InControlState.aCoxaInitAngle[LegIndex] = -700;
    }
    g_wLegsXZLength = 0xffff;
#endif
}


//--------------------------------------------------------------------
// AdjustLegPositionsToBodyHeight() - Will try to adjust the position of the legs
//     to be appropriate for the current y location of the body...
//--------------------------------------------------------------------

uint8_t g_iLegInitIndex = 0x00;    // remember which index we are currently using...

void AdjustLegPositionsToBodyHeight()
{
#ifdef CNT_HEX_INITS
  // Lets see which of our units we should use...
  // Note: We will also limit our body height here...
  if (g_InControlState.BodyPos.y > (short)(g_abHexMaxBodyY[CNT_HEX_INITS-1]))
    g_InControlState.BodyPos.y =  (short)(g_abHexMaxBodyY[CNT_HEX_INITS-1]);

  uint8_t i;
  word XZLength1 = (g_abHexIntXZ[CNT_HEX_INITS-1]);
  for(i = 0; i < (CNT_HEX_INITS-1); i++) {    // Don't need to look at last entry as we already init to assume this one...
    if (g_InControlState.BodyPos.y <= (short)(g_abHexMaxBodyY[i])) {
      XZLength1 = (g_abHexIntXZ[i]);
      break;
    }
  }
  if (i != g_iLegInitIndex) { 
    g_iLegInitIndex = i;  // remember the current index...
    
    // Call off to helper function to do the work.
#ifdef DEBUG_BUG
    if (g_fDebugOutput) {
        DBGSerial.print("ALPTBH: ");
        DBGSerial.print(g_InControlState.BodyPos.y, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(XZLength1, DEC);
    }
#endif    
    AdjustLegPositions(XZLength1);
  }
#endif // CNT_HEX_INITS

}

// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency)
{

#if !(defined __MK20DX256__ || defined __MK64FX512__ || defined __MK66FX1M0__)

#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  *pin_port &= ~(pin_mask);  // keep pin low after stop
#else
// The tone command does sort of work, but does not play multiple sounds smoothly
//  tone(SOUND_PIN, frequency, duration);  // Try the arduino library
//  delay(duration);
  // Try to get something working on DUE...
  long toggle_count = 0;
  long lusDelayPerHalfCycle;
  boolean fHigh = false;
  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);
  digitalWrite(SOUND_PIN, LOW);
  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    fHigh  = !fHigh;
    digitalWrite(SOUND_PIN, fHigh? LOW : HIGH);
    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  digitalWrite(SOUND_PIN, LOW);

#endif
}

void MSound(byte cNotes, ...)
{
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...)
{
};
#endif

#ifdef OPT_TERMINAL_MONITOR
#ifdef OPT_DUMP_EEPROM
extern void DumpEEPROMCmd(byte *pszCmdLine);
#endif
#ifdef QUADMODE
extern void UpdateGaitCmd(byte *pszCmdLine);
#endif
#ifdef OPT_DYNAMIC_ADJUST_LEGS
extern void UpdateInitialPosAndAngCmd(byte *pszCmdLine);
#endif

//==============================================================================
// TerminalMonitor - Simple background task checks to see if the user is asking
//    us to do anything, like update debug levels ore the like.
//==============================================================================
boolean TerminalMonitor(void)
{
  byte szCmdLine[20];  // currently pretty simple command lines...
  byte ich;
  int ch;
  // See if we need to output a prompt.
  if (g_fShowDebugPrompt) {
    DBGSerial.println(F("Arduino Phoenix Monitor"));
    DBGSerial.println(F("D - Toggle debug on or off"));
#ifdef OPT_DUMP_EEPROM
    DBGSerial.println(F("E - Dump EEPROM"));
#endif
#ifdef QUADMODE
//	DBGSerial.println(F("B <percent>"));
    DBGSerial.println(F("G ST NL RR RF LR LF"));
#endif
#ifdef OPT_DYNAMIC_ADJUST_LEGS
    DBGSerial.println(F("I pos ang"));
#endif
#ifdef OPT_TERMINAL_MONITOR_IC    // Allow the input controller to define stuff as well
    InputController::controller()->ShowTerminalCommandList(); 
#endif      

    // Let the Servo driver show it's own set of commands...
    ServoDriver::driver()->ShowTerminalCommandList();
    g_fShowDebugPrompt = false;
  }

  // First check to see if there is any characters to process.
  if ((ich = DBGSerial.available())) {
    ich = 0;
    // For now assume we receive a packet of data from serial monitor, as the user has
    // to click the send button...
    for (ich=0; ich < sizeof(szCmdLine); ich++) {
      ch = DBGSerial.read();        // get the next character
      if ((ch == -1) || ((ch >= 10) && (ch <= 15)))
        break;
      szCmdLine[ich] = ch;
    }
    szCmdLine[ich] = '\0';    // go ahead and null terminate it...
    
    // Remove any extra EOL characters that may have been added
    for (;;) {
      ch = DBGSerial.peek();
      if ((ch >= 10) && (ch <= 15))
        DBGSerial.read();
      else
        break;
    }
    if (ich) {
      DBGSerial.print(F("Serial Cmd Line:"));        
      DBGSerial.write(szCmdLine, ich);
      DBGSerial.println(F("<eol>"));
    }
    // So see what are command is.
    if (!ich)  {
      g_fShowDebugPrompt = true;
    } 
    else if ((ich == 1) && ((szCmdLine[0] == 'd') || (szCmdLine[0] == 'D'))) {
      g_fDebugOutput = !g_fDebugOutput;
      if (g_fDebugOutput) 
        DBGSerial.println(F("Debug is on"));
      else
        DBGSerial.println(F("Debug is off"));
    } 
#ifdef OPT_DUMP_EEPROM
    else if (((szCmdLine[0] == 'e') || (szCmdLine[0] == 'E'))) {
      DumpEEPROMCmd(szCmdLine);
    } 
#endif
#ifdef QUADMODE
    else if (((szCmdLine[0] == 'g') || (szCmdLine[0] == 'G'))) {
      UpdateGaitCmd(szCmdLine);
    } 
#endif
#ifdef OPT_DYNAMIC_ADJUST_LEGS
    else if (((szCmdLine[0] == 'i') || (szCmdLine[0] == 'I'))) {
      UpdateInitialPosAndAngCmd(szCmdLine);
    } 
#endif
#ifdef OPT_TERMINAL_MONITOR_IC    // Allow the input controller to define stuff as well
    else if (InputController::controller()->ProcessTerminalCommand(szCmdLine, ich)) 
      ;  // See if the Input controller has added commands...
#endif      

    else
    {
      ServoDriver::driver()->ProcessTerminalCommand(szCmdLine, ich);
    }

    return true;
  }
  return false;
}


//--------------------------------------------------------------------
// DumpEEPROM
//--------------------------------------------------------------------
#ifdef OPT_DUMP_EEPROM
byte g_bEEPromDumpMode = 0;  // assume mode 0 - hex dump
word g_wEEPromDumpStart = 0;  // where to start dumps from
byte g_bEEPromDumpCnt = 16;  // how much to dump at a time

void DumpEEPROM() {
  byte i;
  word wDumpCnt = g_bEEPromDumpCnt;

  while (wDumpCnt) {
    DBGSerial.print(g_wEEPromDumpStart, HEX);
    DBGSerial.print(" - ");

    // First in Hex
    for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
      byte b;
      b = EEPROM.read(g_wEEPromDumpStart+i);
      DBGSerial.print(b, HEX);
      DBGSerial.print(" ");
    }
    // Next in Ascii
    DBGSerial.print(" : ");
    for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
      byte b;
      b = EEPROM.read(g_wEEPromDumpStart+i);
      if ((b > 0x1f) && (b < 0x7f))
        DBGSerial.write(b);
      else
        DBGSerial.print(".");
    }
    DBGSerial.println("");
    g_wEEPromDumpStart += i;  // how many bytes we output
    wDumpCnt -= i;            // How many more to go...
  } 

}
#endif

//--------------------------------------------------------------------
// GetCmdLineNum - passed pointer to pointer so we can update...
//--------------------------------------------------------------------
long GetCmdLineNum(byte **ppszCmdLine) {
  byte *psz = *ppszCmdLine;
  long iVal = 0;
  int iSign = 1;

  // Ignore any blanks
  while (*psz == ' ')
    psz++;

  // See if Hex value passed in
  if ((*psz == '0') && ((*(psz+1) == 'x') || (*(psz+1) == 'X'))) {
    // Hex mode
    psz += 2;  // get over 0x
    for (;;) {
      if ((*psz >= '0') && (*psz <= '9'))
        iVal = iVal * 16 + *psz++ - '0';
      else if ((*psz >= 'a') && (*psz <= 'f'))
        iVal = iVal * 16 + *psz++ - 'a' + 10;
      else if ((*psz >= 'A') && (*psz <= 'F'))
        iVal = iVal * 16 + *psz++ - 'A' + 10;
      else
        break;
    }

  }
  else {
    // decimal mode
    if (*psz == '-') {
        iSign = -1;
        psz++;
    }    
        
    while ((*psz >= '0') && (*psz <= '9'))
      iVal = iVal * 10 + *psz++ - '0';
  }
  *ppszCmdLine = psz;    // update command line pointer
  return iSign * iVal;

}

#ifdef OPT_DUMP_EEPROM
//--------------------------------------------------------------------
// DumpEEPROMCmd
//--------------------------------------------------------------------
void DumpEEPROMCmd(byte *pszCmdLine) {
  // first byte can be H for hex or W for words...
  if (!*++pszCmdLine)  // Need to get past the command letter first...
    DumpEEPROM();
  else if ((*pszCmdLine == 'h') || (*pszCmdLine == 'H')) 
    g_bEEPromDumpMode = 0;
  else if ((*pszCmdLine == 'w') || (*pszCmdLine == 'W')) 
    g_bEEPromDumpMode = 0;

  else {
    // First argument should be the start location to dump
    g_wEEPromDumpStart = GetCmdLineNum(&pszCmdLine);

    // If the next byte is an "=" may try to do updates...
    if (*pszCmdLine == '=') {
      // make sure we don't get stuck in a loop...
      byte *psz = pszCmdLine;
      word w;
      while (*psz) {
        w = GetCmdLineNum(&psz);
        if (psz == pszCmdLine)
          break;  // not valid stuff so bail!
        pszCmdLine = psz;  // remember how far we got...

        EEPROM.write(g_wEEPromDumpStart++, w & 0xff);
      }
    }
    else {
      if (*pszCmdLine == ' ') { // A blank assume we have a count...
        g_bEEPromDumpCnt = GetCmdLineNum(&pszCmdLine);
      }
    }
    DumpEEPROM();
  }
}
#endif

#ifdef QUADMODE
//--------------------------------------------------------------------
// UpdateGaitCmd
//--------------------------------------------------------------------
void UpdateGaitCmd(byte *pszCmdLine) {
  // If no other parameters, show current state
  if (!*++pszCmdLine) {  // Need to get past the command letter first...
    DBGSerial.print("St: ");
    DBGSerial.print(g_InControlState.gaitCur.StepsInGait, DEC);
    DBGSerial.print(" ");
    DBGSerial.print(g_InControlState.gaitCur.NrLiftedPos, DEC);
    DBGSerial.print(" ");
    DBGSerial.print(g_InControlState.gaitCur.GaitLegNr[cRR], DEC);
    DBGSerial.print(" ");
    DBGSerial.print(g_InControlState.gaitCur.GaitLegNr[cRF], DEC);
    DBGSerial.print(" ");
    DBGSerial.print(g_InControlState.gaitCur.GaitLegNr[cLR], DEC);
    DBGSerial.print(" ");
    DBGSerial.println(g_InControlState.gaitCur.GaitLegNr[cLF], DEC);
  }
  else {
    //Argument should be New percentage
    word wStepsInGait = GetCmdLineNum(&pszCmdLine);
    word wLifted = GetCmdLineNum(&pszCmdLine);
    
    // first pass only pass in number of steps and maybe Lifted pos
    if (wStepsInGait) {
        if (wLifted) {
            // UPdated the lifted so lets update some of the gait properties
            g_InControlState.gaitCur.NrLiftedPos = wLifted;
            g_InControlState.gaitCur.FrontDownPos = (wLifted+1)/2;
            g_InControlState.gaitCur.LiftDivFactor = (wLifted > 4)? 4 : 2;
        }

        // Assume the ordering of the gait legs here and equal spaced
        g_InControlState.gaitCur.StepsInGait = wStepsInGait;
        g_InControlState.gaitCur.TLDivFactor = g_InControlState.gaitCur.StepsInGait-g_InControlState.gaitCur.NrLiftedPos;
            
        // See if user did pass in leg positions...
        g_InControlState.gaitCur.GaitLegNr[cRR] = GetCmdLineNum(&pszCmdLine);
        if (g_InControlState.gaitCur.GaitLegNr[cRR]) {
            g_InControlState.gaitCur.GaitLegNr[cRF] = GetCmdLineNum(&pszCmdLine);
            g_InControlState.gaitCur.GaitLegNr[cLR] = GetCmdLineNum(&pszCmdLine);
            g_InControlState.gaitCur.GaitLegNr[cLF] = GetCmdLineNum(&pszCmdLine);
        }
        else {
            wStepsInGait /= 4;  // equal spacing.
            g_InControlState.gaitCur.GaitLegNr[cRR] = wStepsInGait / 2;
            g_InControlState.gaitCur.GaitLegNr[cRF] = g_InControlState.gaitCur.GaitLegNr[cRR] + wStepsInGait;
            g_InControlState.gaitCur.GaitLegNr[cLR] = g_InControlState.gaitCur.GaitLegNr[cRF] + wStepsInGait;
            g_InControlState.gaitCur.GaitLegNr[cLF] = g_InControlState.gaitCur.GaitLegNr[cLR] + wStepsInGait;
        }
    
        //g_InControlState.gaitCur.HalfLiftHeight = 3;
        //g_InControlState.gaitCur.NomGaitSpeed = DEFAULT_GAIT_SPEED;
    }   
  }
}
#endif //Quad Mode

//--------------------------------------------------------------------
// UpdateGaitCmd
//--------------------------------------------------------------------
#ifdef OPT_DYNAMIC_ADJUST_LEGS
void UpdateInitialPosAndAngCmd(byte *pszCmdLine) {
  // If no other parameters, show current state
  if (!*++pszCmdLine) {  // Need to get past the command letter first...
	DBGSerial.print("Len: ");
	DBGSerial.print(GetLegsXZLength() , DEC);
	DBGSerial.print(" Angs: ");
    for(int LegIndex=0; LegIndex < CNT_LEGS; LegIndex++) {
        DBGSerial.print(g_InControlState.aCoxaInitAngle[LegIndex], DEC);
        DBGSerial.print(" ");
    }
    DBGSerial.println();
  }
  else {
	// Get the new leg positions
    word wNewLegsXZPos = GetCmdLineNum(&pszCmdLine);
    if (*pszCmdLine) {
      int  iDeltaAngle = GetCmdLineNum(&pszCmdLine);
      RotateLegInitAngles(iDeltaAngle);
    }  
    AdjustLegPositions(wNewLegsXZPos);

  }
}
#endif

#endif
