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
#ifndef _PHOENIX_NOTIFY_ST77XX_
#define _PHOENIX_NOTIFY_ST77XX_
#include <ST7735_t3.h>
#include <ST7789_t3.h>

class ST7XXNotification: public UserNotification {
public:

	//ST7735 Will expand to ST7789
	ST7XXNotification(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST = -1, uint8_t BL=-1);

	virtual void     Init(void) override;
	virtual bool     notify(byte Voltage, byte CMD, char Data[21]) override;
private:
	uint8_t _cs, _rs, _sid, _sclk, _rst, _bl;
	ST7735_t3 _tft;
} ;


#endif
