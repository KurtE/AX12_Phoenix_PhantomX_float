#include <Arduino.h>
#include "Hex_Cfg.h"
#include "phoenix_float.h"
#include "phoenix_notify_ssd13xx.h"


SSD13XXNotification::SSD13XXNotification(uint8_t w, uint8_t h, TwoWire *twi, uint8_t i2caddr) :
    _w(w), _h(h), _twi(twi), _i2caddr(i2caddr), 
    _tft(w, h, twi)
{

}

void SSD13XXNotification::Init(void)
{

    Serial.println("SSD13XXNotification::Init");
    if(!_tft.begin(SSD1306_SWITCHCAPVCC, _i2caddr)) {
        Serial.println(F("SSD1306 begin failed"));
    }

    _tft.display();
    delay(2000); // Pause for 2 seconds


    //see if I get anything
    _tft.clearDisplay();
    _tft.setTextSize(1);      // Normal 1:1 pixel scale
    _tft.setTextColor(SSD1306_WHITE); // Draw white text
    _tft.setCursor(0, 0);     // Start at top-left corner

    _tft.print("Start");
    _tft.display();
    // add us to the notification list
    UserNotification::addNotificationObject(this);

}

bool SSD13XXNotification::notify(byte Voltage, byte CMD, char Data[21])
{
    if (CMD > 0) {
        _tft.clearDisplay();
        _tft.setTextSize(1);      // Normal 1:1 pixel scale
        _tft.setTextColor(SSD1306_WHITE); // Draw white text
        _tft.setCursor(0, 0);     // Start at top-left corner

        _tft.print(Data);
        _tft.display();        
    }
	return true;
}
