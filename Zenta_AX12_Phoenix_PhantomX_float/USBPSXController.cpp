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
//Hardware Setup: Sony PS4 or PS3 connected to Teensy using either direct wired or bluetooth
//

//
//====================================================================
// [Include files]
#if defined(TEENSYDUINO)
#include <Arduino.h>
#include "Hex_Cfg.h"

// Only include code if we using
#if defined(USE_USB_JOYSTICK) 
#include "phoenix_float.h"
#include "USBPSXController.h"
#include <USBHost_t36.h>


//[CONSTANTS]
enum {
	WALKMODE = 0, TRANSLATEMODE, ROTATEMODE, //Only two modes, walk and single leg
#ifdef OPT_SINGLELEG
	SINGLELEGMODE,
#endif
	MODECNT
};
enum {
	NORM_NORM = 0, NORM_LONG, HIGH_NORM, HIGH_LONG
};


#define ARBOTIX_TO  5000        // if we don't get a valid message in this number of mills turn off
#define cTravelDeadZone 6


//=============================================================================
// Global - Local to this file only...
//=============================================================================
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
JoystickController joystick1(myusb);
KeyboardController keyboard1(myusb);

#if defined(BLUETOOTH)
	//BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
	BluetoothController bluet(myusb);   // version assumes it already was paired

  USBDriver* drivers[] = { &hub1, &hub2, &joystick1, &keyboard1, &bluet, &hid1, &hid2, &hid3 };
  
  #define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
  const char* driver_names[CNT_DEVICES] = { "Hub1", "Hub2", "JOY1D", "KB1", "Bluet", "HID1" , "HID2", "HID3" };
  
  bool driver_active[CNT_DEVICES] = { false, false, false, false, false };
#else
  BluetoothController bluet(myusb);   // version assumes it already was paired

  USBDriver* drivers[] = { &hub1, &hub2, &joystick1, &keyboard1, &hid1, &hid2, &hid3 };
  
  #define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
  const char* driver_names[CNT_DEVICES] = { "Hub1", "Hub2", "JOY1D", "KB1", "HID1" , "HID2", "HID3" };
  
  bool driver_active[CNT_DEVICES] = { false, false, false, false };
#endif

// Lets also look at HID Input devices
USBHIDInput* hiddrivers[] = { &joystick1 };

#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char* hid_driver_names[CNT_DEVICES] = { "Joystick1" };

bool hid_driver_active[CNT_DEVICES] = { false };
#if defined(BLUETOOTH)
	BTHIDInput* bthiddrivers[] = { &joystick1 };
	#define CNT_BTHIDDEVICES (sizeof(bthiddrivers)/sizeof(bthiddrivers[0]))
	const char* bthid_driver_names[CNT_HIDDEVICES] = { "joystick"};
	bool bthid_driver_active[CNT_HIDDEVICES] = { false };
#endif

const __FlashStringHelper* const Mode_table[] PROGMEM = { F("Crawling Mode"), F("Translate Mode"), F("Rotate Mode"), F("Single Leg Mode") }; 
const __FlashStringHelper* const Gait_table[] PROGMEM = { F("Wave gait"), F("Ripple gait"), F("Tripple gait"), F("Tripod gait") };//A table to hold the names
const __FlashStringHelper* const LegH_table[] PROGMEM = { F("Max leg height"), F("Med High leg height"), F("Med Low leg height"), F("Low leg height") };//A table to hold the names



// some external or forward function references.
const short cLegLiftHeight[] PROGMEM = {
	MaxLegLiftHeight, MedHighLegLiftHeight, MedLegLiftHeight, MinLegLiftHeight
};


const uint32_t USBPSXController::PS3_BTNS[]  = { 0x400, 0x100, 0x2, 0x800, 0x200, 0x4,
                                                       0x1000, 0x8000, 0x4000, 0x2000,
                                                       0x10000, 0x1, 0x8,
                                                       // UP  DN    LFT   RHT
                                                       0x10, 0x40, 0x80, 0x20
                                                     };

const uint32_t USBPSXController::PS4_BTNS[] = {
	0x10, 0x40, 0x400, 0x20, 0x80, 0x800,
	0x8, 0x1, 0x2, 0x4,
	0x1000, 0x200, 0x100,       // PS, options, Share
	0x10000, 0x40000, 0x80000, 0x20000       // HAT
};
const uint32_t USBPSXController::PS4_MAP_HAT_MAP[]  = {
	//0x10, 0x30, 0x20, 0x60, 0x40, 0xc0, 0x80, 0x90, 0x00 };
	0x10000, 0x30000, 0x20000, 0x60000, 0x40000, 0xC0000, 0x80000, 0x90000, 0x0
};

static bool joystick_ps4_bt = false;

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void USBPSXController::Init(void)
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
	DBGSerial.println("USB Joystick Init: ");
#endif

	if (!g_myusb_begun) {
		myusb.begin();
		keyboard1.attachPress(OnPress);
		g_myusb_begun = true;		
	}

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
void USBPSXController::AllowControllerInterrupts(boolean fAllow)
{
	// We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
void USBPSXController::ControlInput(void)
{
	// See if we have a new command available...
	// Make sure USB gets chance to process stuff.
	myusb.Task();
	// check to see if the device list has changed:
	UpdateActiveDeviceInfo();
	//  processPS3MotionTimer();  - not sure yet support this yet.

	if (joystick1.available()) {
		// If we receive a valid message than turn robot on...
		boolean fAdjustLegPositions = false;
		short sLegInitXZAdjust = 0;
		short sLegInitAngleAdjust = 0;

		if (_first_joystick_message) {
			DBGSerial.printf("*** First Joystick message %x:%x ***\n",
			                 joystick1.idVendor(), joystick1.idProduct());
			_first_joystick_message = false;

			const uint8_t* psz = joystick1.manufacturer();
			if (psz && *psz) DBGSerial.printf("  manufacturer: %s\n", psz);
			const uint8_t* pszProduct = joystick1.product();
			if (pszProduct && *pszProduct) DBGSerial.printf("  product: %s\n", pszProduct);
			psz = joystick1.serialNumber();
			if (psz && *psz) DBGSerial.printf("  Serial: %s\n", psz);
			
			if (pszProduct && strncmp((const char *)pszProduct, "Wireless Controller", 19) == 0) {
				BTN_MASKS = PS4_BTNS;
				joystick_ps4_bt = true;
				DBGSerial.println("      BlueTooth PS4");
			}

			// lets try to reduce number of fields that update
			//joystick1.axisChangeNotifyMask(0xFFFFFl);
		}
		_buttons = joystick1.getButtons();
		// [SWITCH MODES]
		// We will use L1 button with the Right joystick to control both body offset as well as Speed...
		// We move each pass through this by a percentage of how far we are from center in each direction
		// We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
		// which give an idea, but only for those whoes robot has a speaker
		int lx = joystick1.getAxis(AXIS_LX) - 127;
		int ly = joystick1.getAxis(AXIS_LY) - 127;
		int rx = joystick1.getAxis(AXIS_RX) - 127;
		int ry = joystick1.getAxis(AXIS_RY) - 127;
		
		if(abs(lx) < cTravelDeadZone) lx = 0;
		if(abs(ly) < cTravelDeadZone) ly = 0;
		if(abs(rx) < cTravelDeadZone) rx = 0;
		if(abs(ry) < cTravelDeadZone) ry = 0;

#if defined(BLUETOOTH)
		if (joystick_ps4_bt || joystick1.joystickType() == JoystickController::PS4) {
			int hat = joystick_ps4_bt? joystick1.getAxis(10) : joystick1.getAxis(9);  // get hat - up/dwn buttons
#else
		if (joystick1.joystickType() == JoystickController::PS4) {
			int hat = joystick1.getAxis(9);  // get hat - up/dwn buttons
#endif
			if ((hat >= 0) && (hat < 8)) _buttons |= PS4_MAP_HAT_MAP[hat];
			BTN_MASKS = PS4_BTNS;	// should have been set earlier, but just in case...
		}
		else {
			BTN_MASKS = PS3_BTNS;
		}
#ifdef DBGSerial
		// do after we map the hat in for PS4
		if (_fDebugJoystick) {
			DBGSerial.printf("(%d)BTNS: %x LX: %d, LY: %d, RX: %d, RY: %d LT: %d RT: %d\r\n", joystick1.joystickType(), _buttons,
			                 lx, ly, rx, ry, joystick1.getAxis(AXIS_LT), joystick1.getAxis(AXIS_RT));
		}
#endif


#if defined(BLUETOOTH)
		if (ButtonPressed(BUT_PS3)) {
			if ((joystick1.joystickType() == JoystickController::PS3) &&
			    (_buttons & (BTN_MASKS[BUT_L1] | BTN_MASKS[BUT_R1]))) {
				// PS button just pressed and select button pressed act like PS4 share like...
				// Note: you can use either R1 or L1 with the PS button, to work with Sony Move Navigation...
				DBGSerial.print("\nPS3 Pairing Request");
				if (!_last_bdaddr[0] && !_last_bdaddr[1] && !_last_bdaddr[2] && !_last_bdaddr[3] && !_last_bdaddr[4] && !_last_bdaddr[5]) {
					DBGSerial.println(" - failed - no Bluetooth adapter has been plugged in");
				}
				else if (!hiddrivers[0]) {  // Kludge see if we are connected as HID?
					DBGSerial.println(" - failed - PS3 device not plugged into USB");
				}
				else {
					DBGSerial.printf(" - Attempt pair to: %x:%x:%x:%x:%x:%x\n", _last_bdaddr[0], _last_bdaddr[1],
					                 _last_bdaddr[2], _last_bdaddr[3], _last_bdaddr[4], _last_bdaddr[5]);

					if (!joystick1.PS3Pair(_last_bdaddr)) {
						DBGSerial.println("  Pairing call Failed");
					}	else {
						DBGSerial.println("  Pairing complete (I hope), make sure Bluetooth adapter is plugged in and try PS3 without USB");
					}
				}
			}
			else {
				// Maybe lets toggle the
				if (!g_InControlState.fRobotOn) {
					g_InControlState.fRobotOn = true;
					fAdjustLegPositions = true;
					g_WakeUpState = true;//Start the wakeup routine
					SetControllerMsg(1, "(0)Robot Power On.....");

					//delay(10000);//Testing a bug that occour after powerup. Robot turns on and of and then on again. After programming first time it work fine. Then bug start after powerup
					g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..
				}
				else {
					controllerTurnRobotOff();
					SetControllerMsg(1, "(0)Robot Power Off.....");
				}
			}
		}
#else
		if (ButtonPressed(BUT_PS3)) {
				if (!g_InControlState.fRobotOn) {
					g_InControlState.fRobotOn = true;
					fAdjustLegPositions = true;
					SetControllerMsg(1, "Robot Power On.....");
					g_WakeUpState = true;//Start the wakeup routine

					//delay(10000);//Testing a bug that occour after powerup. Robot turns on and of and then on again. After programming first time it work fine. Then bug start after powerup
					g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..
				}
				else {
					controllerTurnRobotOff();
					SetControllerMsg(1, "Robot Power Off.....");
				}
		}
#endif

			if (ButtonPressed(BUT_X)) {
				MSound(1, 50, 2000);
				_fDebugJoystick = !_fDebugJoystick;
			}

		if (g_InControlState.fRobotOn && !g_WakeUpState) {	//Don't take care of controller inputs if we are not on or in until the WakeUpState is over (false)

			// Cycle through modes...
			// experiment use start/options button to cycle modes.
			if (ButtonPressed(BUT_START)) {
				if (++_controlMode >= MODECNT) {
					_controlMode = WALKMODE;    // cycled back around...
					MSound(2, 50, 2000, 50, 3000);
				}
				else {
					MSound(1, 50, 2000);
				}
				SetControllerMsg(1, (const char *)Mode_table[_controlMode]);
			}

			//[Common functions]
			//Switch Balance mode on/off 
			if (ButtonPressed(BUT_SQ)) {
				g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
				if (g_InControlState.BalanceMode) {
					MSound(1, 250, 1500);
					SetControllerMsg(1, "Balance Mode: On ...");
				}
				else {
					MSound(2, 100, 2000, 50, 4000);
					SetControllerMsg(1, "Balance Mode: Off ..");
				}
			}

			//Stand up, sit down Triangle like PS2
			if (ButtonPressed(BUT_TRI)) {
				if (_bodyYOffset > 0) {
					_bodyYOffset = 0;
					g_InhibitMovement = true;//Do not allow body movement and walking
					SetControllerMsg(1, "Resting position");
				}
				else {
					_bodyYOffset = STAND_BODY_Y;//Zenta a little higher for avoiding the out of range issue on a symmetric MKI PhanomX
					g_InhibitMovement = false; //Allow body movement and walking
					SetControllerMsg(1, "Ready for action!");
				}
				g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..

				fAdjustLegPositions = false;//Zenta setting this to false removes a bug
				_fDynamicLegXZLength = false;
			}

			if (ButtonPressed(BUT_HAT_DOWN)) {
				if (_bodyYOffset < 10) {
					_bodyYOffset = 0;
					g_InhibitMovement = true;//Do not allow body movement and walking
					SetControllerMsg(1, "Resting position");
				} else {
					_bodyYOffset -= 10;
				}
				g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..
			}

			if (ButtonPressed(BUT_HAT_UP)) {
				if (_bodyYOffset < 10) {
					_bodyYOffset = STAND_BODY_Y;	 // stand up default position.
					g_InhibitMovement = false;//Do not allow body movement and walking
					SetControllerMsg(1, "Ready for action!");
				} else {
					_bodyYOffset += 10;
					if (_bodyYOffset > MAX_BODY_Y) _bodyYOffset = MAX_BODY_Y;
				}
				g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..
			}

			if (ButtonPressed(BUT_L2)) { // Choose Speed mode
				MSound(1, 50, 2000);
				if (_doubleTravelOn) {
					_doubleTravelOn = false;
					SetControllerMsg(1, "Double Travel Off ..");
				} else {
					_doubleTravelOn = true;
					SetControllerMsg(1, "Double Travel On ...");
				}
			}
			if (ButtonPressed(BUT_R2)) { // Choose leg height
				MSound(1, 50, 2000);
				_heightSpeedMode++;
				if (_heightSpeedMode >= (sizeof(LegH_table)/sizeof(LegH_table[0])))
					_heightSpeedMode = 0; 
				g_InControlState.LegLiftHeight = pgm_read_word(&cLegLiftHeight[_heightSpeedMode]);//Key A = MaxLegLiftHeight
				SetControllerMsg(1, (const char*)LegH_table[_heightSpeedMode]);
			}

			//----------------------------------------------------
			// Adjust with L1 button pressed
			//----------------------------------------------------
			if (ButtonDown(BUT_L1)) {
				int delta = ry / 25;
				if (delta) {
					_bodyYOffset = max(min(_bodyYOffset + delta, MAX_BODY_Y), 0);
					fAdjustLegPositions = true;
				}

				// Also use right Horizontal to manually adjust the initial leg positions.
				sLegInitXZAdjust = lx / 10;        // play with this.
				sLegInitAngleAdjust = ly / 8;
				lx = 0;
				ly = 0;

				// Likewise for Speed control
				delta = rx / 16;   //
				if ((delta < 0) && g_InControlState.SpeedControl) {
					if ((word)(-delta) < g_InControlState.SpeedControl)
						g_InControlState.SpeedControl += delta;
					else
						g_InControlState.SpeedControl = 0;
					MSound(1, 50, 1000 + g_InControlState.SpeedControl);
				}
				if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
					g_InControlState.SpeedControl += delta;
					if (g_InControlState.SpeedControl > 2000)
						g_InControlState.SpeedControl = 2000;
					MSound(1, 50, 1000 + g_InControlState.SpeedControl);
				}

				rx = 0; // don't walk when adjusting the speed here...

			}
			//----------------------------------------------------
			//----------------------------------------------------

#ifdef OPT_SINGLELEG
			//Common control functions for both walking and single leg
			if ((_controlMode == WALKMODE) || (_controlMode == SINGLELEGMODE)) {
#else
			if (_controlMode == WALKMODE) {
#endif
				bool gait_changed = false;
				if (ButtonPressed(BUT_SELECT)) { // Select chooses different gait
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
					//Serial.println((const char *)Gait_table[g_InControlState.GaitType]);
				}

/// TEST BT KEYPAD
				_keypad_button = keyboard1.getKey();
				if ((_keypad_button >= '1') && (_keypad_button <= '4')) {
					g_InControlState.GaitType = _keypad_button - '1';
					gait_changed = true;
					MSound(1, 50, 2000);
					Serial.printf("Gait Type #: %d\n", g_InControlState.GaitType );
				}
				if (gait_changed) {
					//strcpy_P(g_InControlState.DataPack, (char*)pgm_read_word(&(Gait_table[Index])));
					strcpy(g_InControlState.DataPack, (const char *)Gait_table[g_InControlState.GaitType]);
					g_InControlState.DataMode = 1;
					g_InControlState.lWhenWeLastSetDatamode = millis();
					Serial.printf("Gait Selected: %s\n", (const char *)Gait_table[g_InControlState.GaitType]) ;
				}

				// Was 7 on Zentas....
				if (_keypad_button == '0') {
					if (SmDiv > 20) {
						SmDiv = 1;
						MSound(1, 50, 1000);
						strcpy(g_InControlState.DataPack, "Raw and fast control");
						Serial.printf("Gait Control: Raw and fast control\n");
					} else {
						SmDiv *= 3;
						SmDiv += 7;
						MSound(1, 50, 1500 + SmDiv * 20);
						strcpy(g_InControlState.DataPack, "Smooth control");
						if (SmDiv > 20) strcpy(g_InControlState.DataPack, "Super Smooth ctrl!");
						Serial.printf("Gait Control: Smooth control\n");
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}
				// Was A-D on zentas...
				if ((_keypad_button >= '5') && (_keypad_button <= '8')) {
					int8_t Index = _keypad_button - '5';
					g_InControlState.LegLiftHeight = pgm_read_word(&cLegLiftHeight[Index]);//Key A = MaxLegLiftHeight
					strcpy(g_InControlState.DataPack, (char*)LegH_table[Index]);
					MSound(1, 50, 2000);
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}
				if (_keypad_button == '#') {
					if (g_InControlState.BodyRotOffset.y == 0) {
						g_InControlState.BodyRotOffset.y = 200;
						strcpy(g_InControlState.DataPack, "YRotation offset =20");
						Serial.printf("YRotation offset =20\n");
					}
					else {
						g_InControlState.BodyRotOffset.y = 0;
						strcpy(g_InControlState.DataPack, "YRotation offset = 0");
						Serial.printf("YRotation offset = 0\n");

					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
					MSound(1, 50, 2000);
				}

/////END TEST BT KEYPAD

	
				// Switch between Walk method 1 && Walk method 2
				if (ButtonPressed(BUT_CIRC)) { // Change walking mode
	#ifdef cTurretRotPin
					if ((++_bJoystickWalkMode) > 2)
	#else
					if ((++_bJoystickWalkMode) > 1)
	#endif 
						_bJoystickWalkMode = 0;
					MSound(1, 50, 2000 + _bJoystickWalkMode * 250);
					Serial.printf("Walkmethod %d Selected ........\n", _bJoystickWalkMode );
				}

			}//Common functions end

			//[Walk functions]
			if (_controlMode == WALKMODE) {
				switch (_bJoystickWalkMode) {
				case 0:
					g_InControlState.TravelLength.x = float(-lx) * (5.0 / 7.0)  ;
					g_InControlState.TravelLength.z = float(-ly) * (5.0 / 7.0);
					g_InControlState.TravelLength.y = float(-rx) / 5.0; //Right Stick Left/Right 
					break;
				case 1:
					g_InControlState.TravelLength.z = (float)(ry)  * (5.0 / 7.0); //Right Stick Up/Down  
					g_InControlState.TravelLength.y = float(-rx) / 5.0; //Right Stick Left/Right 
					break;
	#ifdef cTurretRotPin
				case 2:
					g_InControlState.TravelLength.x = -lx;
					g_InControlState.TravelLength.z = -ly;

					// Will use Right now stick to control turret.
					g_InControlState.TurretRotAngle1 = max(min(g_InControlState.TurretRotAngle1 + rx / 5, cTurretRotMax1), cTurretRotMin1);      // Rotation of turret in 10ths of degree
					g_InControlState.TurretTiltAngle1 = max(min(g_InControlState.TurretTiltAngle1 + joystick1.getAxis(AXIS_RY) / 5, cTurretTiltMax1), cTurretTiltMin1);  // tilt of turret in 10ths of degree
	#endif
				}

				if (!_doubleTravelOn) {  //(Double travel length)
					g_InControlState.TravelLength.x /= 2.0;
					g_InControlState.TravelLength.z /= 2.0;
				}
				g_InControlState.InputTimeDelay = 128 - max(abs(lx), abs(ly));
			}

			_bodyYShift = 0;

			if (_controlMode == TRANSLATEMODE) {
				g_InControlState.BodyPos.x = SmoothControl(((lx) * 2 / 3), g_InControlState.BodyPos.x, SmDiv);
				g_InControlState.BodyPos.z = SmoothControl(((ly) * 2 / 3), g_InControlState.BodyPos.z, SmDiv);
				g_InControlState.BodyRot1.y = SmoothControl(((rx) * 2), g_InControlState.BodyRot1.y, SmDiv);

				//      g_InControlState.BodyPos.x = (lx)/2;
				//      g_InControlState.BodyPos.z = -(ly)/3;
				//      g_InControlState.BodyRot1.y = (rx)*2;
				_bodyYShift = float(-(joystick1.getAxis(AXIS_RY)-127) / 2); //Zenta should be joystick1.getAxis(AXIS_RY) - 127
			}

			//[Rotate functions]
			if (_controlMode == ROTATEMODE) {
				g_InControlState.BodyRot1.x = (ly);
				g_InControlState.BodyRot1.y = (rx) * 2;
				g_InControlState.BodyRot1.z = (lx);
				_bodyYShift = (-(joystick1.getAxis(AXIS_RY)-127) / 2); //Zenta Should be -127 or just replace with ry
			}
		//[Single leg functions]

			//[Single leg functions]
#ifdef OPT_SINGLELEG
			if (_controlMode == SINGLELEGMODE) {
				//Switch leg for single leg control
				if (ButtonPressed(BUT_SELECT)) { // Select chooses different leg

					if (g_InControlState.SelectedLeg == 5) { //Only toogle between the two front legs
						g_InControlState.SelectedLeg = 2;//Right Leg
						SetControllerMsg(1, "Right Single Leg");
						MSound(1, 50, 2000);
					}
					else {
						g_InControlState.SelectedLeg = 5;//Left Leg
						SetControllerMsg(1, "Left Single Leg");
						MSound(2, 50, 2000, 50, 2250);
					}
				}
				//Zenta, Fixed. replaced byte with float, also increased range

				if ((ly >= 0) || (g_InControlState.SLLeg.y <= 0)) { //Only allow leg to move while it is lifted
					if (ly < 0) ly = 0;//Not allowing negative value
#ifdef MXPhoenix
					g_InControlState.SLLeg.y = SmoothControl((-(int)ly) * 3, g_InControlState.SLLeg.y, SmDiv); //Using left vertical for leg lifting instead, increase range even more in balancemode
					//g_InControlState.SLLeg.x = (float)((int)command.leftH); //NOT using X control anymore, Yawrot is better for the front legs
					g_InControlState.SLLeg.z = SmoothControl((-(int)lx), g_InControlState.SLLeg.z, SmDiv); //Left Stick Up/Down
					g_InControlState.SLyawRot = SmoothControl((-ry) / 4, g_InControlState.SLyawRot, SmDiv);//WIP test!!!!
#else
					g_InControlState.SLLeg.y = SmoothControl((-(int)ly) * 3, g_InControlState.SLLeg.y, SmDiv); //Using left vertical for leg lifting instead, increase range even more in balancemode
					//g_InControlState.SLLeg.x = (float)((int)command.leftH); //NOT using X control anymore, Yawrot is better for the front legs
					g_InControlState.SLLeg.z = SmoothControl((-(int)lx), g_InControlState.SLLeg.z, SmDiv); //Left Stick Up/Down
					g_InControlState.SLyawRot = SmoothControl((-(int)ry) / 4, g_InControlState.SLyawRot, SmDiv);//WIP test!!!!
#endif
				}
				// Hold single leg in place
				if ((_buttons & BTN_MASKS[BUT_R2]) && !(_buttons_prev & BTN_MASKS[BUT_R2])) {
					MSound(1, 50, 2000);
					if (!g_InControlState.fSLHold && !_wantToEndSLHold) {
						_prevYposSLHold = g_InControlState.SLLeg.y; //Save the Ypos when holding it
						g_InControlState.fSLHold = true;
					}
					else {
						_wantToEndSLHold = true; //We want to change state
					}
				}
				if (_wantToEndSLHold && (g_InControlState.SLLeg.y <= _prevYposSLHold)) { //make sure the joystick is in right position before
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
			_buttons_prev = _buttons;
		}
		_ulLastMsgTime = millis();
	}
	else {
		if (!g_WakeUpState) { //At the moment we can't turn off robot during the WakeUpState, is that a problem?
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
// CommanderTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void USBPSXController::controllerTurnRobotOff(void)
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

//=============================================================================
// UpdateActiveDeviceInfo
//=============================================================================
void USBPSXController::UpdateActiveDeviceInfo() {
	for (uint8_t i = 0; i < CNT_DEVICES; i++) {
		if (*drivers[i] != driver_active[i]) {
			if (driver_active[i]) {
				Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
				driver_active[i] = false;
			}
			else {
				Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
				driver_active[i] = true;

				const uint8_t* psz = drivers[i]->manufacturer();
				if (psz && *psz) DBGSerial.printf("  manufacturer: %s\n", psz);
				psz = drivers[i]->product();
				if (psz && *psz) DBGSerial.printf("  product: %s\n", psz);
				psz = drivers[i]->serialNumber();
				if (psz && *psz) DBGSerial.printf("  Serial: %s\n", psz);

				if (drivers[i] == &bluet) {
					const uint8_t* bdaddr = bluet.myBDAddr();
					// remember it...
					DBGSerial.printf("  BDADDR: %x:%x:%x:%x:%x:%x\n", bdaddr[0], bdaddr[1], bdaddr[2], bdaddr[3], bdaddr[4], bdaddr[5]);
					for (uint8_t i = 0; i < 6; i++) _last_bdaddr[i] = bdaddr[i];
				}
			}
		}
	}

	for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
		if (*hiddrivers[i] != hid_driver_active[i]) {
			if (hid_driver_active[i]) {
				DBGSerial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
				hid_driver_active[i] = false;
			}
			else {
				DBGSerial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
				hid_driver_active[i] = true;

				const uint8_t* psz = hiddrivers[i]->manufacturer();
				if (psz && *psz) DBGSerial.printf("  manufacturer: %s\n", psz);
				psz = hiddrivers[i]->product();
				if (psz && *psz) DBGSerial.printf("  product: %s\n", psz);
				psz = hiddrivers[i]->serialNumber();
				if (psz && *psz) DBGSerial.printf("  Serial: %s\n", psz);

				// See if this is our joystick object...
				if (hiddrivers[i] == &joystick1) {
					DBGSerial.printf("  Joystick type: %d\n", joystick1.joystickType());
					switch (joystick1.joystickType()) {
					case JoystickController::PS4:
						BTN_MASKS = PS4_BTNS;
						break;
					default:
					case JoystickController::PS3:
						BTN_MASKS = PS3_BTNS;
						break;
					}

#ifdef LATER
					if (joystick1.joystickType() == JoystickController::PS3_MOTION) {
						DBGSerial.println("  PS3 Motion detected");
						PS3_MOTION_timer = millis();  // set time for last event
						PS3_MOTION_tried_to_pair_state = 0;
					}
#endif
				}

			}
		}
	}
	
#if defined(BLUETOOTH)
	// Then Bluetooth devices
	for (uint8_t i = 0; i < CNT_BTHIDDEVICES; i++) {
		if (*bthiddrivers[i] != bthid_driver_active[i]) {
			if (bthid_driver_active[i]) {
				DBGSerial.printf("*** BTHID Device %s - disconnected ***\n", hid_driver_names[i]);
				bthid_driver_active[i] = false;
			}
			else {
				DBGSerial.printf("*** BTHID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
				bthid_driver_active[i] = true;

				const uint8_t* psz = bthiddrivers[i]->manufacturer();
				if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
				const uint8_t *pszProduct = bthiddrivers[i]->product();
				if (pszProduct && *pszProduct) Serial.printf("  product: %s\n", pszProduct);
				psz = bthiddrivers[i]->serialNumber();
				if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
				// See if this is our joystick object...
				if (hiddrivers[i] == &joystick1) {
					joystick_ps4_bt = false; // lets assume not...
					DBGSerial.printf("  Joystick type: %d\n", joystick1.joystickType());
					switch (joystick1.joystickType()) {
					case JoystickController::PS4:
						BTN_MASKS = PS4_BTNS;
						break;
					default:
					case JoystickController::PS3:
						BTN_MASKS = PS3_BTNS;
						break;
					}
					// See if this is a BT 
					if (pszProduct && strncmp((const char *)pszProduct, "Wireless Controller", 19) == 0) {
						BTN_MASKS = PS4_BTNS;
						joystick_ps4_bt = true;
						DBGSerial.println("      BlueTooth PS4");
					}
				}
			}
		}
	}
#endif
}

void USBPSXController::OnPress(int key)
{
  Serial.print("key based on getKey ");
  //Serial.println((char)keyboard1.getKey());

   _keypad_button = keyboard1.getKey();
   Serial.println( (char) _keypad_button);
}


//================================================================================
#ifdef OPT_TERMINAL_MONITOR_IC
// Optional stuff to allow me to have Input device debug support
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void USBPSXController::ShowTerminalCommandList(void)
{
	DBGSerial.println(F("J - Show Joystick data"));
}

boolean USBPSXController::ProcessTerminalCommand(byte *psz, byte bLen)
{
	if ((bLen == 1) && ((*psz == 'j') || (*psz == 'J'))) {
		_fDebugJoystick = !_fDebugJoystick;
		if (_fDebugJoystick) DBGSerial.println("\n*** Joystick Debug ON ***");
		else DBGSerial.println("\n*** Joystick Debug OFF ***");
		return true;
	}
	return false;
}
#endif

//==============================================================================
//==============================================================================
//Send message back to remote
bool USBPSXController::SendMsgs(byte Voltage, byte CMD, char Data[21]) {
#ifdef DBGSerial
	if (CMD) {
		DBGSerial.printf("%u %u:%s\n", Voltage, CMD, Data);
	}
#endif
	// TODO, output to optional display
	// Tell caller OK to clear out this message now.
	return true;
}


#endif
#endif // USE_USB_JOYSTICK
