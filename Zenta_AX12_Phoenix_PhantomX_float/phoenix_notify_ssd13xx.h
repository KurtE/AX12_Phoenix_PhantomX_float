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
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
//==============================================================================
#ifndef _PHOENIX_NOTIFY_SSD13XX_
#define _PHOENIX_NOTIFY_SSD13XX_

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
class SSD13XXNotification: public UserNotification {
public:

	//ST7735 Will expand to ST7789
	SSD13XXNotification(uint8_t w, uint8_t h, TwoWire *twi = &Wire, uint8_t i2caddr=0x3d);

	virtual void     Init(void) override;
	virtual bool     notify(byte Voltage, byte CMD, char Data[21]) override;
private:
	uint8_t _w, _h;
	TwoWire *_twi; 
	uint8_t _i2caddr;
	Adafruit_SSD1306 _tft;
} ;


#endif
