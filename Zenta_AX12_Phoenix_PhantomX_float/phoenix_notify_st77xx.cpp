#include <Arduino.h>
#include "Hex_Cfg.h"
#include "phoenix_float.h"
#include "phoenix_notify_st77xx.h"

ST7XXNotification::ST7XXNotification(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST, uint8_t BL):
	_cs(CS),_rs(RS), _sid(SID), _sclk(SCLK), _rst(RST), _bl(BL),_tft(_cs, _rs, _sid, _sclk, _rst)
{
}

void ST7XXNotification::Init(void)
{

	#ifdef TFT_SD_CS
	pinMode(TFT_SD_CS, OUTPUT);
	digitalWriteFast(TFT_SD_CS, HIGH);
	#endif

	DBGSerial.println("Before TFT.INITR");
	// Again need to parameter 
 	//_tft.initR(INITR_MINI160x80);
    _tft.init(135, 240);           // Init ST7789 240x135

#ifdef TFT_BL
	DBGSerial.println("Turn on Backlight");
    pinMode(TFT_BL, OUTPUT);
    digitalWriteFast(TFT_BL, HIGH);
#endif
    // have some fun display logo
    _tft.fillScreen(ST77XX_RED);
    delay(1000);
    _tft.fillScreen(ST77XX_GREEN);
    delay(1000);
    _tft.fillScreen(ST77XX_BLUE);
    delay(1000);
    _tft.fillScreen(ST77XX_BLACK);

    // add us to the notification list
    UserNotification::addNotificationObject(this);

}

bool ST7XXNotification::notify(byte Voltage, byte CMD, char Data[21])
{
	return true;
}
