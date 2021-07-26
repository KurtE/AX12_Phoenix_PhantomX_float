
/*
  Commander.pde - default firmware for arbotiX Commander (V1.1)
 Copyright (c) 2009, 2010 Michael E. Ferguson.  All right reserved.
 
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
 
 Arduino IDE Configuration:
 
 Mucked with by KurtE to start toward a new version...
 
 */

//=============================================================================
// Options
//=============================================================================
#define CHECK_AND_CONFIG_XBEE
#define OPT_ORIG_RANGE_COMPAT
#define DEBUG

#define XBEE Serial1

//=============================================================================
// Defines
//=============================================================================

#define RIGHT_V   0
#define RIGHT_H   1
#define LEFT_V    2
#define LEFT_H    3

#define BUT_L6    13
#define BUT_L5    12
#define BUT_L4    11
#define BUT_LT    9

#define BUT_R3    7
#define BUT_R2    6
#define BUT_R1    5
#define BUT_RT    8

#define BUT_RIGHT 3
#define BUT_LEFT  2

#define USER      10

// These values are used as part of the XBee Configure code and can be changed
// to whatever values you prefer. 
#define DEFAULT_MY 0x102  // Swap My/DL on 2nd unit
#define DEFAULT_DL 0x101
#define DEFAULT_ID 0x3332

static const boolean  g_afAnalogInvert[] = {    // some of our analog values are inverted from what we want
  true,false,true,false};       // Do we use the mid points when we normalize
static const word g_awAnalogMins[] = {
  0*8,0*8,0*8,0*8};  // Will have a calibrate later where we read these in (if necessary)
static const word g_awAnalogMaxs[] = {
  1023*8,1023*8,1023*8,1023*8}; // dito

#ifdef OPT_ORIG_RANGE_COMPAT      
uint8_t g_fLimitRange = false;
#endif  


#define FRAME_LEN 33         // 30hz
//=============================================================================
// Global Variables
//=============================================================================
// Analog information
#define NUMANALOGS    4
word              g_aawRawAnalog[8][NUMANALOGS];              // Keep 8 raw values for each analog input
word              g_awRawBatAnalog[8];
word              g_awRawSums[NUMANALOGS];                    // sum of the raw values.
word              g_awAnalogMids[NUMANALOGS];                 // This is our observed center points at init time. 
byte              g_iRaw;                                     // which raw value are we going to read into

byte              g_bButtons;
byte              g_abJoyVals[NUMANALOGS];                     // Joystick values.

#ifdef DEBUG
boolean           g_fDebugOutput;          // Are we doing debug output?
byte              g_bButtonsPrev;
byte              g_abJoyValsPrev[NUMANALOGS];                // Joystick values.
#endif

unsigned long ltime;         // last time we sent data

int iLoopCnt;

void setup(){
  // I init buttons first as my check state of some buttons in other init code to allow special case
  // init of the system.
  InitButtons();

  InitXBee();

  ltime = millis();
  pinMode(USER,OUTPUT);    // user LED

  InitJoysticks();          // Initialize the joysticks.

  iLoopCnt = 0;
  // one of the top buttons held down at startup go into debug mode...
#ifdef DEBUG  
  g_fDebugOutput = (digitalRead(BUT_LT) == LOW) || (digitalRead(BUT_RT) == LOW); 
#endif
}

void loop(){
  byte i;
  int bChksum;
  ReadJoysticks();
  ReadButtons();
  if(iLoopCnt > 10){
    digitalWrite(10,HIGH-digitalRead(10));
    iLoopCnt=0;
  }
  iLoopCnt++;

  // Compute our checksum...
  bChksum = (int)g_bButtons;
  for (i=0; i < NUMANALOGS; i++) 
    bChksum += (int)g_abJoyVals[i];
  bChksum = (byte)(255 - (byte)(bChksum%256));
#ifdef DEBUG
  // Debug Code...
  if (g_fDebugOutput) {
    if ((g_bButtons != g_bButtonsPrev) || memcmp(&g_abJoyVals, &g_abJoyValsPrev, sizeof(g_abJoyVals))) {
      g_bButtonsPrev = g_bButtons;
      memcpy(&g_abJoyValsPrev, &g_abJoyVals, sizeof(g_abJoyVals)); 
      Serial.print(g_bButtons, HEX);
      Serial.print(F(" : "));
      for (i=0; i < NUMANALOGS; i++)  {
        Serial.print((int)g_abJoyVals[i]-128, DEC);
        Serial.print(F(" "));
      }
      Serial.println(bChksum, DEC);
    }
  } 
  else {
#endif
    XBEE.write((byte)0xff);
    for (i=0; i < NUMANALOGS; i++)  {
      XBEE.write((byte)g_abJoyVals[i]);
    }
    XBEE.write((byte)g_bButtons);
    XBEE.write((byte)0);        // extra
    XBEE.write((byte)bChksum);

#ifdef DEBUG
  }
#endif
  delay (FRAME_LEN);
}

//=============================================================================
// InitJoysticks: Initialize our Joysticks and other analog devices
//=============================================================================
void InitJoysticks(void) {          // Initialize the joysticks.
  byte i;
  word w;
  // Later may need to read in Min/Max values for each of the Anlog inputs from EEPROM.
#ifdef OPT_ORIG_RANGE_COMPAT      
  if(digitalRead(BUT_L6) == LOW) {
    g_fLimitRange = true;
  }

#endif  


  // We need to prefill our array of raw items and sums.  The first read for each analog
  // may be trash, but that will get removed on our first real read.
  for (i=0; i<NUMANALOGS; i++) {
    g_awRawSums[i] = 0;        // initialize to zero
    w =  analogRead(i);    // first read is not valid
    for (g_iRaw=0; g_iRaw < 8; g_iRaw++) {
      g_aawRawAnalog[g_iRaw][i] = analogRead(i);
      g_awRawSums[i] += g_aawRawAnalog[g_iRaw][i];
      delay(1);
    }
    // Save away these sums as our mid points...
    g_awAnalogMids[i] = g_awRawSums[i];
  }
}
//=============================================================================
// InitButtons: Initialize the buttons
//=============================================================================
void InitButtons(void) {          // Initialize the joysticks.

  // pullups for buttons
#ifdef INPUT_PULLUP
  //    pinMode(BUT_L6, INPUT_PULLUP);
  digitalWrite(BUT_L6, HIGH);
  pinMode(BUT_L5, INPUT_PULLUP);
  pinMode(BUT_L4, INPUT_PULLUP);
  pinMode(BUT_LT, INPUT_PULLUP);
  pinMode(BUT_R3, INPUT_PULLUP);
  pinMode(BUT_R2, INPUT_PULLUP);
  pinMode(BUT_R1, INPUT_PULLUP);
  pinMode(BUT_RT, INPUT_PULLUP);

  digitalWrite(BUT_RIGHT, HIGH);
  digitalWrite(BUT_LEFT, HIGH);
  //    pinMode(BUT_RIGHT, INPUT_PULLUP);
  //    pinMode(BUT_LEFT, INPUT_PULLUP);
#else
  digitalWrite(BUT_L6, HIGH);
  digitalWrite(BUT_L5, HIGH);
  digitalWrite(BUT_L4, HIGH);
  digitalWrite(BUT_LT, HIGH);
  digitalWrite(BUT_R3, HIGH);
  digitalWrite(BUT_R2, HIGH);
  digitalWrite(BUT_R1, HIGH);
  digitalWrite(BUT_RT, HIGH);

  digitalWrite(BUT_RIGHT, HIGH);
  digitalWrite(BUT_LEFT, HIGH);
#endif
}
//=============================================================================
// ReadJoysticks: Initialize our Joysticks and other analog devices
//=============================================================================
void ReadJoysticks() {          // Initialize the joysticks.
  byte i;

  // Calculate which index of our raw array we should now reuse.
  g_iRaw = (g_iRaw+1) & 0x7;

  // Now lets read in the next analog reading and smooth the values
  // Note: The original commander code did not allow a full range, so we will not as well
  // See if I get away with 1-254
  for (i=0; i<NUMANALOGS; i++) {
    g_awRawSums[i] -= g_aawRawAnalog[g_iRaw][i];        // remove the value we overwrite from sum
    g_aawRawAnalog[g_iRaw][i] = analogRead(i);
    g_awRawSums[i] += g_aawRawAnalog[g_iRaw][i];        // Add the new value in

    // Lets calculate our calibrated values from 0-255 whith 128 as center point
    if (g_awRawSums[i] <= g_awAnalogMins[i])
      g_abJoyVals[i] = 1;                // Take care of out of range low
    else if (g_awRawSums[i] >= g_awAnalogMaxs[i])
      g_abJoyVals[i] = 254;              // out of range high
    else if (g_awRawSums[i] <= g_awAnalogMids[i])
      g_abJoyVals[i] = map(g_awRawSums[i], g_awAnalogMins[i], g_awAnalogMids[i], 1, 128);
    else
      g_abJoyVals[i] = map(g_awRawSums[i], g_awAnalogMids[i], g_awAnalogMaxs[i], 128, 254);

    if (g_afAnalogInvert[i]) 
      g_abJoyVals[i] = 255 - g_abJoyVals[i];
#ifdef OPT_ORIG_RANGE_COMPAT      
    if (g_fLimitRange)
      g_abJoyVals[i] = map(g_abJoyVals[i], 0, 254, 128-102, 128+102);
#endif
  }
}
//=============================================================================
// ReadButtons: Read in all of the buttons...
//=============================================================================
void ReadButtons(void)
{
  g_bButtons = 0;
  if(digitalRead(BUT_R1) == LOW) g_bButtons += 1;
  if(digitalRead(BUT_R2) == LOW) g_bButtons += 2;
  if(digitalRead(BUT_R3) == LOW) g_bButtons += 4;
  if(digitalRead(BUT_L4) == LOW) g_bButtons += 8;
  if(digitalRead(BUT_L5) == LOW) g_bButtons += 16;
  if(digitalRead(BUT_L6) == LOW) g_bButtons += 32;
  if(digitalRead(BUT_RT) == LOW) g_bButtons += 64;
  if(digitalRead(BUT_LT) == LOW) g_bButtons += 128;
}

void InitXBee() 
{
  // This may not happen on this Arduino, but on some systems have issue that 
  // when first powered up XBee comes up in command mode and then you have to wait
  // for the command mode timeout before it starts to work.  Quick hack, start at 
  // 9600 baud, output ATCN to exit, then open up at right baud rate. 
  // Verify the XBee is configured properly.  See if we can talk to it at 38400
  XBEE.begin(9600);
  XBEE.println(F("ATCN"));
  XBEE.flush();
  delay(10);
  XBEE.end();

  // Now open at the real speed. 
  XBEE.begin(38400);

#ifdef CHECK_AND_CONFIG_XBEE
  char ab[10];
  while (XBEE.read() != -1)
    ;  // flush anything out...
  // First lets see if we have a real short command time
  delay(10);  // see if we have fast command mode enabled.
  XBEE.print(F("+++")); 
  XBEE.flush();
  XBEE.setTimeout(20);  // give a little extra time
  if (XBEE.readBytesUntil('\r', ab, 10) > 0) {
    XBEE.println(F("ATCN"));	          // and exit command mode
    return;  // bail out quick
  }
  // Else see if maybe properly configured but not quick command mode.
  delay(1000);
  XBEE.print(F("+++"));
  XBEE.setTimeout(1100);  // little over a second
  if (XBEE.readBytesUntil('\r', ab, 10) > 0) {
    // Note: we could check a few more things here if we run into issues.  Like: MY!=0
    // or MY != DL
    XBEE.println(F("ATGT 5"));              // Set a quick command mode
    XBEE.println(F("ATWR"));	          // Write out the changes
    XBEE.println(F("ATCN"));	          // and exit command mode
    return;  // It is already at 38400, so assume already init.
  }
  // Failed, so check to see if we can communicate at 9600
  XBEE.end();
  XBEE.begin(9600);
  while (XBEE.read() != -1)
    ;  // flush anything out...

  delay(2000);
  XBEE.print(F("+++"));
  if (XBEE.readBytesUntil('\r', ab, 10) == 0) {
    // failed blink fast
    for(int i=0;i<50;i++) {
      digitalWrite(USER, !digitalRead(USER));
      delay(100);
    }  // loop forever
  } 
  else {

    // So we entered command mode, lets set the appropriate stuff. 
    XBEE.println(F("ATBD 5"));  // 38400
    XBEE.print(F("ATID "));
    XBEE.println(DEFAULT_ID, HEX);

    XBEE.print(F("ATMY "));
    XBEE.println(DEFAULT_MY, HEX);

    XBEE.println(F("ATDH 0"));
    XBEE.print(F("ATDL "));
    XBEE.println(DEFAULT_DL, HEX);

    XBEE.println(F("ATGT 5"));    // Set a quick command mode
    XBEE.println(F("ATWR"));	// Write out the changes
    XBEE.println(F("ATCN"));	// and exit command mode
    XBEE.flush();              // make sure all has been output
  }
  delay(250);  // Wait a bit for responses..
  while(XBEE.read() != -1)
    ;   // flush all the input.

  XBEE.begin(38400);
#endif  
}
