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
#ifndef _phoenix_input_DIY_Commander_h_
#define _phoenix_input_DIY_Commander_h_


#include "phoenix_float.h"

class CommanderInputController : public InputController
{
  	public:

	  virtual void     Init(void);
	  virtual void     ControlInput(void);
	  virtual void     AllowControllerInterrupts(boolean fAllow);
	  virtual void     SendMsgs(byte Voltage, byte CMD, char Data[21]);

	#ifdef OPT_TERMINAL_MONITOR_IC  // Allow Input controller to define stuff as well
	  virtual void            ShowTerminalCommandList(void);
	  virtual boolean         ProcessTerminalCommand(byte *psz, byte bLen);
	#endif
// lets define Buttons and Axis mapping for different joysticks.

	private:
		short  		_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet


		float   _bodyYOffset; //Relative body height position (offset)
		float   _bodyYShift; //Body height adjusted by joystick
		float	_bodyYpos;	//Body height adjusted by right sliderpot
		float   _bodyZShift; //Body Z translation adjusted by joystick
		float	_bodyZpos;	//Body Z position adjusted by left sliderpot
		byte    _controlMode;
		byte    _heightSpeedMode;
		bool    _doubleTravelOn;
		byte    _bJoystickWalkMode;
		bool	_LtopStickWalkMode; //Toogle walking and body translation on left joystick
		bool	_RtopStickWalkMode; //Toogle body rotation and body translation on right joystick
		bool	_delayedWalkMode;	//Toogle normal walk and "look around then walk effect".
		float   _prevYposSLHold;		//Saves the current leg height during holding the single leg position
		bool	_wantToEndSLHold;	//Set true if we want to stop holding the position
		bool	_isRightSliderInitated; //Set true if the right slider pot has been centered after startup
		bool	_isLeftSliderInitated; //Set true if the left slider pot has been centered after startup
		bool 	_fDebugJoystick = false;	// are
		boolean _fDynamicLegXZLength = false;  // Has the user dynamically adjusted the Leg XZ init pos (width)
		uint32_t _ulLastMsgTime;

		byte        GPSeq;             //Number of the sequence
		byte		SmDiv;  //"Smooth division" factor for the smooth control function, Make this a variable??
		byte    _buttonsPrev;
		byte    _extPrev;
		void controllerTurnRobotOff();
		void UpdateActiveDeviceInfo();
//		inline bool ButtonPressed(uint8_t button_index) {return ((_buttons & BTN_MASKS[button_index]) && !(_buttons_prev & BTN_MASKS[button_index]));}
//		inline bool ButtonDown(uint8_t button_index) {return (_buttons & BTN_MASKS[button_index]);}
};
   
#endif
