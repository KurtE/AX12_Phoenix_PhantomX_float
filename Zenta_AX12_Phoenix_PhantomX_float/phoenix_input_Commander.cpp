//#define DEBUG_COMMANDER
//TO DO:
//Updated function explanation
//Define max Strafe length, rotation angles and translation length, defined in the Hex_Cfg and a default #ifndef here?
//Maybe just send raw control values to the main code and do the mapping there or here?
//Single leg mode and dual mode
//Adjustible init positions, front and rear, extend or retract
//Auto body rotation?
//Tapping leg random, use an autofunction in the single leg mode?
//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: Arbotix Commander DIY version Using two 3axis joysticks with one button on top, 2 sliderpots, 16key keypad, 6 function buttons and one power/function button
// Orientation of the 6 function buttons (same as on the Arbotix Commander): L6, L5, L4 - R3, R2, R1
//
//There are two main modes that can't be combined; Walking mode and Single Leg mode
//Button R3 are used to toogle between Walk
//

//
//====================================================================
// [Include files]
#include <Arduino.h>
#include "Hex_Cfg.h"
#if defined(USE_COMMANDER)  

#include "phoenix_input_Commander.h"

//[CONSTANTS]
enum {
  WALKMODE=0, TRANSLATEMODE, ROTATEMODE, //Only two modes, walk and single leg
#ifdef OPT_SINGLELEG      
  SINGLELEGMODE, 
#endif
  MODECNT};
const __FlashStringHelper* const Mode_table[] PROGMEM = { F("Crawling Mode"), F("Translate Mode"), F("Rotate Mode"), F("Single Leg Mode") }; 

enum {
  NORM_NORM=0, NORM_LONG, HIGH_NORM, HIGH_LONG};


#define ARBOTIX_TO  5000        // if we don't get a valid message in this number of mills turn off

#ifndef XBeeSerial
SoftwareSerial XBeeSerial(cXBEE_IN, cXBEE_OUT);
#endif
#ifndef USER
#define USER 13
#endif

//=============================================================================
// I have included a modified version of the commander object here as I wish to
// decouple the usage of the joysticks from the names as well as on other robots
// I may not be using Serial as the the serial port I am communicating with
// So I also removed the Southpaw code.
//=============================================================================
/*
  Commander.h - Library for interfacing with ArbotiX Commander
 Copyright (c) 2009-2012 Michael E. Ferguson.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


/* bitmasks for buttons array 
And how the buttons are placed on the 3Dprinted remote */
#define BUT_R1      0x01	//Right lower inner button
#define BUT_R2      0x02	//Right lower outer button 
#define BUT_R3      0x04	//Right upper outer button 
#define BUT_L4      0x08	//Left upper outer button
#define BUT_L5      0x10	//Left lower outer button
#define BUT_L6      0x20	//Left lower inner button
#define BUT_RT      0x40	//Right upper inner button
#define BUT_LT      0x80	//Left upper inner button

//bugbug: todo this needs to be deleted when finished updating
/*Keypad definitions (ASCII values)*/
//#define CKEY_0				48	//For key 1,2,3.. just add 
//#define	CKEY_A				65	//For key B,C,D.. just add
//#define CKEY_Asterix	42	//* key NOT used to control robot, programming mode for the controller
//#define KEY_Hash		35	//# key
//#define KEY_PowerBtn 80 //P char Power button on remote

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

const char Gait_0[] PROGMEM = "Wave gait";   // define gait names, max 20 chars
const char Gait_1[] PROGMEM = "Ripple gait";
const char Gait_2[] PROGMEM = "Tripple gait";
const char Gait_3[] PROGMEM = "Tripod gait";

const char* const Gait_table[] PROGMEM = { Gait_0, Gait_1, Gait_2, Gait_3 };//A table to hold the names

const char LegH_0[] PROGMEM = "Max leg height";   
const char LegH_1[] PROGMEM = "Med High leg height";
const char LegH_2[] PROGMEM = "Med Low leg height";
const char LegH_3[] PROGMEM = "Low leg height";

const char* const LegH_table[] PROGMEM = { LegH_0, LegH_1, LegH_2, LegH_3 };//A table to hold the names

/* the Commander will send out a frame at about 30hz, this class helps decipher the output. */
class Commander
{    
public:
  Commander(); 
  void begin(unsigned long baud);
  int ReadMsgs();         // must be called regularly to clean out Serial buffer

  // joystick values are -125 to 125
  signed char rightV;      // vertical stick movement = forward speed
  signed char rightH;      // horizontal stick movement = sideways or angular speed
  signed char leftV;      // vertical stick movement = tilt    
  signed char leftH;      // horizontal stick movement = pan (when we run out of pan, turn body?)
	//signed char rightT;			// Zenta right 3DOF joystick top potmeter
	//signed char leftT;			// Zenta left 3DOF joystick top potmeter
	//unsigned char Rslider;		// Zenta, upper right slider adjusting body height
	//unsigned char Lslider;		// Zenta, upper left slider adjusting body Z-translate forward backward
	//unsigned char LowerRslider;		// Zenta, lower right slider adjust speedcontrol
	//unsigned char LowerLslider;		// Zenta, lower left slider adjust InputTimedelay. Variable gaitspeed depending on stride length (any slider position above center) or manually when slider is below the center position
  // buttons are 0 or 1 (PRESSED), and bitmapped
  unsigned char buttons;  // 
  //unsigned char ext;      // Extended function set, Zenta using this for the keypad value
													//ext values: ASCII values: 0-9 = 48-57, A-D = 65-68, * = 42, # = 35

	//Added for update
	static byte    _buttonsPrev;
	static byte    _extPrev;

    // Hooks are used as callbacks for button presses -- NOT IMPLEMENT YET

private:
  // internal variables used for reading messages
  unsigned char vals[13];  // (Zenta 7 -> 11.)temporary values, moved after we confirm checksum
  int index;              // -1 = waiting for new packet
  int checksum;
};


//=============================================================================
// Global - Local to this file only...
//=============================================================================
		Commander _command = Commander();

// some external or forward function references.

const short cLegLiftHeight[] PROGMEM = {
	MaxLegLiftHeight, MedHighLegLiftHeight, MedLegLiftHeight, MinLegLiftHeight };

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void CommanderInputController::Init(void)
{
	// moved from phoenix_code.cpp
		SmDiv = 10; //Default SmootControl divider

  _bodyYOffset = 0;//Zenta, Start raising up after wakeuproutine
  _bodyYShift = 0;
	_bodyYpos = 0;
	_bodyZShift = 0;
	_bodyZpos = 0;
	_isRightSliderInitated = 0;
	_isLeftSliderInitated = 0;
#ifdef DBGSerial  
  DBGSerial.print("Commander Init: ");
  DBGSerial.println(XBEE_BAUD, DEC);
#endif  
  _command.begin(XBEE_BAUD);
  GPSeq = 0;  // init to something...

  _controlMode = WALKMODE;
  _heightSpeedMode = NORM_NORM;
	g_InhibitMovement = true;//Do not allow body movement and walking at startup
  //    DoubleHeightOn = false;
  _doubleTravelOn = true;//Zenta want this to start with
  _bJoystickWalkMode = 0;
	_delayedWalkMode = false;
	MSound(1, 250, 500);//Some sound to indicate init is done
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void CommanderInputController::AllowControllerInterrupts(boolean fAllow)
{
  // We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
void CommanderInputController::ControlInput(void)
{
	// See if we have a new _command available...
	if (_command.ReadMsgs() > 0){
		// If we receive a valid message than turn robot on...
		boolean fAdjustLegPositions = false;
		short sLegInitXZAdjust = 0;
		short sLegInitAngleAdjust = 0;
		
		if (!g_InControlState.fRobotOn ) {
			g_InControlState.fRobotOn = true;
			fAdjustLegPositions = true;
			g_WakeUpState = true;//Start the wakeup routine
			//delay(10000);//Testing a bug that occour after powerup. Robot turns on and of and then on again. After programming first time it work fine. Then bug start after powerup
			g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..
		}

		if (!g_WakeUpState) {//Don't take care of controller inputs until the WakeUpState is over (false)
			// [SWITCH MODES]
			// Cycle through modes...
			if ((_command.buttons & BUT_R3) && !(_buttonsPrev & BUT_R3)) {
				if (++_controlMode >= MODECNT) {
					_controlMode = WALKMODE;    // cycled back around...
					MSound(2, 50, 2000, 50, 3000);
				}
				else {
					MSound(1, 50, 2000);
				}
				if (_controlMode != SINGLELEGMODE) {
					strcpy(g_InControlState.DataPack, (const char *)Mode_table[_controlMode]);
			  }

#ifdef OPT_SINGLELEG      
				if (_controlMode == SINGLELEGMODE) {
					g_InControlState.SelectedLeg = 2;//Zenta made the front right as default at start
					strcpy(g_InControlState.DataPack, "Single Leg Mode");
				}
#endif
				g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
				g_InControlState.lWhenWeLastSetDatamode = millis();
			}


			//Stand up, sit down 
    		if ((_command.buttons & BUT_R1) && !(_buttonsPrev & BUT_R1)) {
				if (_bodyYOffset > 0) {
					_bodyYOffset = 0;
					g_InhibitMovement = true;//Do not allow body movement and walking
					strcpy(g_InControlState.DataPack, "Resting position");
				}
				else {
					//in other this is set to 32
					_bodyYOffset = 80;//Zenta a little higher for avoiding the out of range issue on a symmetric MKI PhanomX
					g_InhibitMovement = false; //Allow body movement and walking
					strcpy(g_InControlState.DataPack, "Ready for action!");
				}
				g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
				g_InControlState.lWhenWeLastSetDatamode = millis();
				g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..

				fAdjustLegPositions = false;//Zenta setting this to false removes a bug
				_fDynamicLegXZLength = false;
			}

			// We will use L6 with the Right joystick to control both body offset as well as Speed...
			// We move each pass through this by a percentage of how far we are from center in each direction
			// We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
			// which give an idea, but only for those whoes robot has a speaker
			//int lx = _command.leftH;  just a note to self
			//int ly = _command.leftV;

			if (_command.buttons & BUT_L6 ) {
			  // raise or lower the robot on the joystick up /down
			  // Maybe should have Min/Max
			  int delta = _command.rightV/25;   
			  if (delta) {
				_bodyYOffset = max(min(_bodyYOffset + delta, MAX_BODY_Y), 0);
				fAdjustLegPositions = true;
			  }

			  // Also use right Horizontal to manually adjust the initial leg positions.
			  sLegInitXZAdjust = _command.leftH/10;        // play with this.
			  sLegInitAngleAdjust = _command.leftV/8;

			  // Likewise for Speed control
			  delta = _command.rightH / 16;   // 
			  if ((delta < 0) && g_InControlState.SpeedControl) {
				if ((word)(-delta) <  g_InControlState.SpeedControl)
				  g_InControlState.SpeedControl += delta;
				else 
				  g_InControlState.SpeedControl = 0;
				MSound( 1, 50, 1000+g_InControlState.SpeedControl);  
			  }
			  if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
				g_InControlState.SpeedControl += delta;
				if (g_InControlState.SpeedControl > 2000)
				  g_InControlState.SpeedControl = 2000;
				MSound( 1, 50, 1000+g_InControlState.SpeedControl); 
				}
				
			  _command.rightH = 0; // don't walk when adjusting the speed here...
			}
			
#ifdef DBGSerial
			if ((_command.buttons & BUT_R3) && !(_buttonsPrev & BUT_R3)) {
			  MSound(1, 50, 2000);
			  g_fDebugOutput = !g_fDebugOutput;
			}
#endif    


#ifdef OPT_SINGLELEG
			//Common control functions for both walking and single leg
			if ((_controlMode == WALKMODE) || (_controlMode == SINGLELEGMODE)) {
#else
			if (_controlMode == WALKMODE){
#endif
				//Deleted Keypad stuff for now may put back in later

				//Switch between two balance methods
				if ((_command.buttons & BUT_R2) && !(_buttonsPrev & BUT_R2)) {
					g_InControlState.BalanceMode++;
					if (g_InControlState.BalanceMode < 2) {//toogle between two modes
						MSound(1, 250, 1500);
						strcpy(g_InControlState.DataPack, "Balance Mode ON");
					}
					else {
						g_InControlState.BalanceMode = 0;
						MSound(2, 100, 2000, 50, 4000);
						strcpy(g_InControlState.DataPack, "Balance Mode OFF");
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}


				// Switch between rotation and Body translation using Right Top joystick button
				if ((_command.buttons & BUT_RT) && !(_buttonsPrev & BUT_RT)) {
					_RtopStickWalkMode = !_RtopStickWalkMode;
					MSound(1, 50, 2000 + _RtopStickWalkMode * 250);
					if (_RtopStickWalkMode) {
						strcpy(g_InControlState.DataPack, "R_Translation Mode");
					}
					else{
						strcpy(g_InControlState.DataPack, "Rotation Mode");
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}

				//uses same joystick controls as the PS4.
				if (_RtopStickWalkMode){//Body translation
					g_InControlState.BodyPos.x = SmoothControl(((_command.rightH) * 2 / 3), g_InControlState.BodyPos.x, SmDiv);
					_bodyZShift = SmoothControl(((_command.leftV) * 2 / 3), _bodyZShift, SmDiv);
					_bodyYShift = SmoothControl(((_command.rightV) * 2), _bodyYShift, SmDiv);
				}
				else{//Default body rotation
					g_InControlState.BodyRot1.x = SmoothControl((_command.rightV) * 2, g_InControlState.BodyRot1.x, SmDiv);//g_InControlState.BodyRot1.x = (_command.rightV) * 2;//Zenta *2
					g_InControlState.BodyRot1.y = SmoothControl((_command.leftV) * 3, g_InControlState.BodyRot1.y, SmDiv);//g_InControlState.BodyRot1.y = (_command.rightT) * 2;
					g_InControlState.BodyRot1.z = SmoothControl((-_command.rightH) * 2, g_InControlState.BodyRot1.z, SmDiv);//g_InControlState.BodyRot1.z = (-_command.rightH) * 2;//Zenta, *2
				}

			} //Common functions end

			//[Walk functions]
			if (_controlMode == WALKMODE) {
//#ifdef UseFootSensors	not implmented for commander
	
				bool gait_changed = false;
				if ((_command.buttons & BUT_L5) && !(_buttonsPrev & BUT_L5)) 
				{ // Select chooses different gait
					g_InControlState.GaitType++;                    // Go to the next gait...
					if (g_InControlState.GaitType < NumOfGaits) {                 // Make sure we did not exceed number of gaits...
						MSound(1, 50, 2000);
					} else {
						MSound(2, 50, 2000, 50, 2250);
						g_InControlState.GaitType = 0;
					}
					gait_changed = true;
				}
				if (gait_changed) {
					//strcpy_P(g_InControlState.DataPack, (char*)pgm_read_word(&(Gait_table[Index])));
					SetControllerMsg(1, (const char *)Gait_table[g_InControlState.GaitType]);
							g_InControlState.DataMode = 1;
							g_InControlState.lWhenWeLastSetDatamode = millis();
							MSound(1, 50, 2000);
				}
	
				//Toogle dampen down speed. Might run this permanently?
				if ((_command.buttons & BUT_L4) && !(_buttonsPrev & BUT_L4)) {
					g_InControlState.DampDwnSpeed = !g_InControlState.DampDwnSpeed;
					if (g_InControlState.DampDwnSpeed) {
						MSound(1, 250, 1500);
						strcpy(g_InControlState.DataPack, "Damping Down ON");
					}
					else {
						MSound(2, 100, 2000, 50, 4000);
						strcpy(g_InControlState.DataPack, "Damping Down OFF");
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}


				// Switch between Walking and Body translation using Left Top joystick button
				if ((_command.buttons & BUT_LT) && !(_buttonsPrev & BUT_LT)) {

					_LtopStickWalkMode = !_LtopStickWalkMode;
					MSound(1, 50, 2000 + _LtopStickWalkMode * 250);
					if (_LtopStickWalkMode) {
						strcpy(g_InControlState.DataPack, "L_Translation Mode");
					}
					else{
						strcpy(g_InControlState.DataPack, "Crawling Mode");
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}
				
				
				if (_LtopStickWalkMode){//Body Translation 
					g_InControlState.BodyPos.x = SmoothControl((_command.leftH) / 2, g_InControlState.BodyPos.x, SmDiv);
					_bodyZShift = SmoothControl((_command.leftV) / 2, _bodyZShift, SmDiv);
					_bodyYShift = SmoothControl((_command.rightV) / 2, _bodyYShift, SmDiv);
					if (_RtopStickWalkMode) {
						_RtopStickWalkMode = !_RtopStickWalkMode;//Force right joystick to control body rotation when doing translation on left stick
					}
				} 
				else{//Default Walking
#ifdef MXPhoenix
					g_InControlState.TravelLength.x = (float)((int)_command.leftH);// *5 / 7; //Left Stick Right/Left about +/- 90mm
					g_InControlState.TravelLength.z = (float)(-(int)_command.leftV);// *5 / 7; //Left Stick Up/Down about +/- 90mm
					g_InControlState.TravelLength.y = (float)(-_command.rightH) / 4;// / 3; //Left Stick Top Pot /5
#else
					g_InControlState.TravelLength.x = (float)((int)_command.leftH) * 5 / 7; //Left Stick Right/Left about +/- 90mm
					g_InControlState.TravelLength.z = (float)(-(int)_command.leftV) * 5 / 7;//Left Stick Up/Down about +/- 90mm
					g_InControlState.TravelLength.y = (float)(-_command.rightH) / 5;//Left Stick Top Pot /5
#endif
						//Calculate walking time delay
				    g_InControlState.InputTimeDelay = 128 - max(max(abs(_command.rightH), abs(_command.rightV)), abs(_command.rightH));
				}
			}

			_bodyYShift = 0;

			if (_controlMode == TRANSLATEMODE) {
				g_InControlState.BodyPos.x = SmoothControl(((_command.leftH) * 2 / 3), g_InControlState.BodyPos.x, SmDiv);
				g_InControlState.BodyPos.z = SmoothControl(((_command.leftV) * 2 / 3), g_InControlState.BodyPos.z, SmDiv);
				g_InControlState.BodyRot1.y = SmoothControl(((_command.rightH) * 2), g_InControlState.BodyRot1.y, SmDiv);

				//      g_InControlState.BodyPos.x = (lx)/2;
				//      g_InControlState.BodyPos.z = -(ly)/3;
				//      g_InControlState.BodyRot1.y = (rx)*2;
				_bodyYShift = float(-(_command.rightV-127) / 2); //Zenta should be joystick1.getAxis(AXIS_RY) - 127
			}

			//[Rotate functions]
			if (_controlMode == ROTATEMODE) {
				g_InControlState.BodyRot1.x = (_command.leftV);
				g_InControlState.BodyRot1.y = (_command.rightH) * 2;
				g_InControlState.BodyRot1.z = (_command.leftH);
				_bodyYShift = (-(_command.rightV-127) / 2); //Zenta Should be -127 or just replace with ry
			}

			//[Single leg functions]
#ifdef OPT_SINGLELEG      
			if (_controlMode == SINGLELEGMODE) {
				//Switch leg for single leg control
				if ((_command.buttons & BUT_L4) && !(_buttonsPrev & BUT_L4) && !g_InControlState.fSLHold) {
					
					if (g_InControlState.SelectedLeg == 5){ //Only toogle between the two front legs
						g_InControlState.SelectedLeg = 2;//Right Leg
						strcpy(g_InControlState.DataPack, "Right Single Leg");
						MSound(1, 50, 2000);
					}
					else{
						g_InControlState.SelectedLeg = 5;//Left Leg
						strcpy(g_InControlState.DataPack, "Left Single Leg");
						MSound(2, 50, 2000, 50, 2250);
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}
				//Zenta, Fixed. replaced byte with float, also increased range
				
				if ((_command.leftV >= 0) || (g_InControlState.SLLeg.y <= 0)){//Only allow leg to move while it is lifted
					if (_command.leftV < 0) _command.leftV = 0;//Not allowing negative value
#ifdef MXPhoenix
					g_InControlState.SLLeg.y = SmoothControl((-(int)_command.leftV) * 3, g_InControlState.SLLeg.y, SmDiv); //Using left vertical for leg lifting instead, increase range even more in balancemode
					//g_InControlState.SLLeg.x = (float)((int)_command.leftH); //NOT using X control anymore, Yawrot is better for the front legs
					g_InControlState.SLLeg.z = SmoothControl((-(int)_command.leftH), g_InControlState.SLLeg.z, SmDiv); //Left Stick Up/Down
					g_InControlState.SLyawRot = SmoothControl((-_command.rightH) / 4, g_InControlState.SLyawRot, SmDiv);//WIP test!!!!
#else
					g_InControlState.SLLeg.y = SmoothControl((-(int)_command.leftV) * 3, g_InControlState.SLLeg.y, SmDiv); //Using left vertical for leg lifting instead, increase range even more in balancemode
					//g_InControlState.SLLeg.x = (float)((int)_command.leftH); //NOT using X control anymore, Yawrot is better for the front legs
					g_InControlState.SLLeg.z = SmoothControl((-(int)_command.leftH), g_InControlState.SLLeg.z, SmDiv); //Left Stick Up/Down
					g_InControlState.SLyawRot = SmoothControl((-_command.rightH) / 4, g_InControlState.SLyawRot, SmDiv);//WIP test!!!!
#endif
				}
				// Hold single leg in place
				if ((_command.buttons & BUT_LT) && !(_buttonsPrev & BUT_LT)) {
					MSound(1, 50, 2000);
					if (!g_InControlState.fSLHold && !_wantToEndSLHold){
						_prevYposSLHold = g_InControlState.SLLeg.y; //Save the Ypos when holding it
						g_InControlState.fSLHold = true;
					}
					else {
						_wantToEndSLHold = true; //We want to change state
					}
				}
				if (_wantToEndSLHold && (g_InControlState.SLLeg.y <= _prevYposSLHold)){//make sure the joystick is in right position before
					g_InControlState.fSLHold = false;
					_wantToEndSLHold = false;
				}
				

			}
#endif

			//Calculate g_InControlState.BodyPos.y
			g_InControlState.BodyPos.y = max(min(_bodyYOffset + _bodyYShift + _bodyYpos, MAX_BODY_Y), 0);//Make sure we don't get beyond the limits, should probably do more of that..
			g_InControlState.BodyPos.z = max(min(_bodyZShift + _bodyZpos, MAX_BODY_Z), MIN_BODY_Z);
			//DBGSerial.print("BPY:");
			//DBGSerial.println((int)g_InControlState.BodyPos.y, DEC);

			if (sLegInitXZAdjust || sLegInitAngleAdjust) {
				// User asked for manual leg adjustment - only do when we have finished any previous adjustment

				if (!g_InControlState.ForceGaitStepCnt) {
					if (sLegInitXZAdjust)
						_fDynamicLegXZLength = true;

					sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
					// Handle maybe change angles...
					if (sLegInitAngleAdjust)
						RotateLegInitAngles(sLegInitAngleAdjust);

					// Give system time to process previous calls
					AdjustLegPositions(sLegInitXZAdjust);
				}
			}

			if (fAdjustLegPositions && !_fDynamicLegXZLength)
				AdjustLegPositionsToBodyHeight();    // Put main workings into main program file

			// Save away the buttons state as to not process the same press twice.
			_buttonsPrev = _command.buttons;
		}

    _buttonsPrev = _command.buttons;
    _ulLastMsgTime = millis();
  } 
	else {
		if (!g_WakeUpState){//At the moment we can't turn off robot during the WakeUpState, is that a problem?
			// We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
			if (g_InControlState.fRobotOn) {
				if ((millis() - _ulLastMsgTime) > ARBOTIX_TO) {
					controllerTurnRobotOff();
					DBGSerial.println("Turning OFF!");//bug we should not be here straight after wakeup!
					//For some reason I've a bug that make the robot turn off during wakeup BUG BUG
				}
			}
		}
  }
}

//==============================================================================
// controllerTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void CommanderInputController::controllerTurnRobotOff(void)
{
  //Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  _bodyYOffset = 0;
  _bodyYShift = 0;
#ifdef OPT_SINGLELEG      
  g_InControlState.SelectedLeg = 255;
#endif
  g_InControlState.fRobotOn = 0;

#ifdef cTurretRotPin
  g_InControlState.TurretRotAngle1 = cTurretRotInit;      // Rotation of turrent in 10ths of degree
  g_InControlState.TurretTiltAngle1 = cTurretTiltInit;    // the tile for the turret
#endif

  _fDynamicLegXZLength = false; // also make sure the robot is back in normal leg init mode...
}
//================================================================================
#ifdef OPT_TERMINAL_MONITOR_IC
// Optional stuff to allow me to have Input device debug support
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void CommanderInputController::ShowTerminalCommandList(void) 
{
  DBGSerial.println(F("X - Show XBee Info"));
}

//==============================================================================
// ProcessTerminalCommand: The terminal monitor will call this to see if the
//     _command the user entered was one added by the servo driver.
//==============================================================================
void PrintXBeeIDInfo(const char *pszID) {
  char ab[20];
  int cbRead;
  while (XBeeSerial.read() != -1)
    ;  // Flush out anything still pending. 
  XBeeSerial.print("AT");  
  XBeeSerial.println(pszID);  // Lets print out the ID;
  XBeeSerial.flush();
  cbRead = XBeeSerial.readBytesUntil('\r', ab, sizeof(ab));
  if (cbRead) {
    DBGSerial.print(pszID);
    DBGSerial.print(": ");
    DBGSerial.write(ab, cbRead);
    DBGSerial.println();
  }
}  

boolean CommanderInputController::ProcessTerminalCommand(byte *psz, byte bLen)
{
  if ((bLen == 1) && ((*psz == 'x') || (*psz == 'X'))) {
    char ab[10];
    delay(15);  // see if we have fast _command mode enabled.
    XBeeSerial.print(F("+++")); 
    XBeeSerial.flush();
    XBeeSerial.setTimeout(20);  // give a little extra time
    if (XBeeSerial.readBytesUntil('\r', ab, 10) > 0) {
      // Ok we entered _command mode, lets print out a few things about the XBee
      PrintXBeeIDInfo("MY");
      PrintXBeeIDInfo("DL");
      PrintXBeeIDInfo("ID");
      PrintXBeeIDInfo("EA");
      PrintXBeeIDInfo("EC");

      XBeeSerial.println("ATCN");  // exit _command mode
      XBeeSerial.readBytesUntil('\r', ab, sizeof(ab));
    } 
    else {
      DBGSerial.println("XBee Failed to enter _command mode");
    }    

    return true;  
  } 
  return false;

}
#endif
//===============================================================================

//==============================================================================
// The below class code is based on the commander class by Michael Ferguson... 
// I included and updated for my own usage...  As I may not always use 
// Serial and I wish to decouple usage of joysticks from the names...
//==============================================================================

/*
  Commander.cpp - Library for interfacing with ArbotiX Commander
 Copyright (c) 2009-2012 Michael E. Ferguson.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

//==============================================================================
// Commander::Commander - Constructor
//==============================================================================
Commander::Commander(){
  index = -1;
}

//==============================================================================
// Commander::begin 
//==============================================================================
void Commander::begin(unsigned long baud){
  char ab[10];
  // Sometimes when we power up the XBee comes up at 9600 in _command mode
  // There is an OK<cr>.  So check for this and try to exit
#ifdef NOT_SURE_WHY_NEEDED_SOMETIMES
  XBeeSerial.begin(9600);  
  XBeeSerial.println(F("ATCN"));  // Tell it to bail quickly
  delay(25);
  XBeeSerial.end();
  delay(25);
#endif  
  XBeeSerial.begin(baud);
  pinMode(USER, OUTPUT);
#ifdef CHECK_AND_CONFIG_XBEE
  while (XBeeSerial.read() != -1)
    ;  // flush anything out...
  // First lets see if we have a real short _command time
  delay(15);  // see if we have fast _command mode enabled.
  XBeeSerial.print(F("+++")); 
  XBeeSerial.flush();
  XBeeSerial.setTimeout(20);  // give a little extra time
  if (XBeeSerial.readBytesUntil('\r', ab, 10) > 0) {
    // Ok we entered _command mode, lets print out a few things about the XBee
    XBeeSerial.println(F("ATCN"));	          // and exit _command mode
    return;  // bail out quick
  }
  // Else see if maybe properly configured but not quick _command mode.
  delay(1100);
  while (XBeeSerial.read() != -1)
    ;  // flush anything out...
  XBeeSerial.print(F("+++"));
  XBeeSerial.setTimeout(1100);  // little over a second
  if (XBeeSerial.readBytesUntil('\r', ab, 10) > 0) {
    // Note: we could check a few more things here if we run into issues.  Like: MY!=0
    // or MY != DL
    XBeeSerial.println(F("ATGT 5"));              // Set a quick _command mode
    XBeeSerial.println(F("ATWR"));	          // Write out the changes
    XBeeSerial.println(F("ATCN"));	          // and exit _command mode
    return;  // It is already at 38400, so assume already init.
  }
  // Failed, so check to see if we can communicate at 9600 properly configured but not quick
  XBeeSerial.end();
  XBeeSerial.begin(9600);
  while (XBeeSerial.read() != -1)
    ;  // flush anything out...

  delay(1100);
  XBeeSerial.print(F("+++"));
  if (XBeeSerial.readBytesUntil('\r', ab, 10) == 0) {
    // failed blink fast
    for(int i=0;i<50;i++) {
      digitalWrite(USER, !digitalRead(USER));
      delay(50);
    }  // Loop awhile
  } 
  else {
    // So we entered _command mode, lets set the appropriate stuff. 
    XBeeSerial.println(F("ATBD 5"));  //7=115200  5=38400
    XBeeSerial.print(F("ATID "));
    XBeeSerial.println(DEFAULT_ID, HEX);

    XBeeSerial.print(F("ATMY "));
    XBeeSerial.println(DEFAULT_MY, HEX);

    XBeeSerial.println(F("ATDH 0"));
    XBeeSerial.print(F("ATDL "));
    XBeeSerial.println(DEFAULT_DL, HEX);

    XBeeSerial.println(F("ATGT 5"));    // Set a quick _command mode
    XBeeSerial.println(F("ATWR"));	// Write out the changes
    XBeeSerial.println(F("ATCN"));	// and exit _command mode
    XBeeSerial.flush();              // make sure all has been output
    // lets do a quick and dirty test
    delay(250);  // Wait a bit for responses..
  }
  XBeeSerial.end();
  delay(10);
  XBeeSerial.begin(38400); //38400
#endif  

}

//==============================================================================
// ReadMsgs
//==============================================================================
int Commander::ReadMsgs(){
  while(XBeeSerial.available() > 0){
    if(index == -1){         // looking for new packet
      if(XBeeSerial.read() == 0xff){
        index = 0;
        checksum = 0;
      }
    }
    else if(index == 0){
      vals[index] = (unsigned char) XBeeSerial.read();
      if(vals[index] != 0xff){            
        checksum += (int) vals[index];
        index++;
      }
    }
    else{
      vals[index] = (unsigned char) XBeeSerial.read();
      checksum += (int) vals[index];
      index++;
      if(index == 7){ // packet complete
        if(checksum%256 != 255){
#ifdef DEBUG_COMMANDER
#ifdef DBGSerial  
          if (g_fDebugOutput) {
            DBGSerial.println("Packet Error");
          }
#endif          
#endif
          // packet error!
          index = -1;
          return 0;
        }
        else{
          digitalWrite(USER, digitalRead(USER)? LOW : HIGH);
          rightV = (signed char)( (int)vals[0]-128 );
          rightH = (signed char)( (int)vals[1]-128 );
          leftV = (signed char)( (int)vals[2]-128 );
          leftH = (signed char)( (int)vals[3]-128 );
          buttons = vals[4];
#ifdef DEBUG_COMMANDER
#ifdef DBGSerial  
          if (g_fDebugOutput) {
            DBGSerial.print(buttons, HEX);
            DBGSerial.print(" : ");
            DBGSerial.print(rightV, DEC);
            DBGSerial.print(" ");
            DBGSerial.print(rightH, DEC);
						DBGSerial.print(" ");
            DBGSerial.print(leftV, DEC);
						DBGSerial.print(" ");
            DBGSerial.println(leftH, DEC);
          }
#endif
#endif
        }
        index = -1;
        while (XBeeSerial.read() != -1)
          ;
        return 1;
      }
    }
  }
  return 0;
}
//==============================================================================
//==============================================================================
//Send message back to remote
bool CommanderInputController::SendMsgs(byte Voltage, byte CMD, char Data[21]){
#ifdef DBGSerial
	if (CMD) {
		DBGSerial.printf("%u %u:%s\n", Voltage, CMD, Data);
	}
#endif
	// TODO, output to optional display
	// Tell caller OK to clear out this message now.
	return true;
}
/*// Compute our checksum...
  bChksum = (int)g_bButtons;
  bChksum += (int)key;
  for (i=0; i < NUMANALOGS; i++) 
    bChksum += (int)g_abJoyVals[i];
  bChksum = (byte)(255 - (byte)(bChksum%256));*/
//Code to send data:
/* XBeeSerial.write((byte)0xff);
    for (i=0; i < NUMANALOGS; i++)  {
      XBeeSerial.write((byte)g_abJoyVals[i]);
    }
    XBeeSerial.write((byte)g_bButtons);
    
    XBeeSerial.write((byte)key); //simply replacing the extra with keypad values
    //XBeeSerial.write((byte)0);        // extra, not used..

    XBeeSerial.write((byte)bChksum);*/
#endif // USE_COMMANDER
