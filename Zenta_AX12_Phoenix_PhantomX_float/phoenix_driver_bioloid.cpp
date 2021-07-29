//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver - This version is setup to use Bioloid type servos using the
// This version was converted over to use the Robotis library Dynamixel2Arduino
// It however still uses a version of the bioloid Interpolation code
//====================================================================
#include <Arduino.h>
#include "Hex_Cfg.h"
#include "phoenix_float.h"
#include "phoenix_driver_bioloid.h"
#define DEBUG_WakeUp_Pos


#ifndef SERVO_CENTER_VALUE
#if (ServoRes > 1024) //A bit dirty method.. Are we using MX servos or not. Probably go another route if we are going to combine AX and MX servos
#define SERVO_TIC_PER_DEG       (ServoRes / 360) //11.37 //(MX) = 4096/360
#else
#define SERVO_TIC_PER_DEG       (ServoRes/300)  //(AX) = 1024/300
#endif
#define SERVO_CENTER_VALUE      (ServoRes/2)  //512(AX)//2048(MX) half of our full range
#endif

#ifndef DXL_BAUD
#define DXL_BAUD 1000000
#endif

#ifndef GOAL_POSITION_REG
#define GOAL_POSITION_REG          30 // default for AX servos.... 
#endif

#define VOLTAGE_MIN_TIME_BETWEEN_CALLS 250

#define VOLTAGE_MAX_TIME_BETWEEN_CALLS 1000    // call at least once per second...
#define VOLTAGE_TIME_TO_ERROR          3000    // Error out if no valid item is returned in 3 seconds...
#include <Dynamixel2Arduino.h>
#include "BioloidInterpolate.h"

#if defined(USE_USB_SERIAL_DXL)
// BUGBUG need to figure out if are configured to build with the USB Host code yet or not...
#include <USBHost_t36.h>
extern USBHost myusb;

USBSerial userial(myusb);  // Maybe see if we can use the big buffer one or not...
#endif


#ifdef DBGSerial
//#define DEBUG
// Only allow debug stuff to be turned on if we have a debug serial port to output to...
//#define DEBUG_SERVOS
#endif

#ifdef DEBUG_SERVOS
#define ServosEnabled   (g_fEnableServos)
#else
#define ServosEnabled  (true)      // always true compiler should remove if...
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
#ifdef QUADMODE
static const byte cPinTable[] PROGMEM = {
  cRRCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLFCoxaPin,
  cRRFemurPin, cRFFemurPin, cLRFemurPin, cLFFemurPin,
  cRRTibiaPin, cRFTibiaPin, cLRTibiaPin, cLFTibiaPin
#ifdef c4DOF
  , cRRTarsPin,  cRFTarsPin,  cLRTarsPin,  cLFTarsPin
#endif
#ifdef cTurretRotPin
  , cTurretRotPin, cTurretTiltPin
#endif
};
#else
static const byte cPinTable[] PROGMEM = {
  cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin,
  cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin,
  cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin
#ifdef c4DOF
  , cRRTarsPin, cRMTarsPin, cRFTarsPin, cLRTarsPin, cLMTarsPin, cLFTarsPin
#endif
#ifdef cTurretRotPin
  , cTurretRotPin, cTurretTiltPin
#endif
};
#endif
#define FIRSTCOXAPIN     0
#define FIRSTFEMURPIN    (CNT_LEGS)
#define FIRSTTIBIAPIN    (CNT_LEGS*2)
#ifdef c4DOF
#define FIRSTTARSPIN     (CNT_LEGS*3)
#define FIRSTTURRETPIN   (CNT_LEGS*4)
#else
#define FIRSTTURRETPIN   (CNT_LEGS*3)
#endif
//---------------------------------------------------------------
// Not sure yet if I will use the controller class or not, but...
#if defined(TEENSYDUINO)
class TeensySerialPortHandler : public DYNAMIXEL::SerialPortHandler
{
  public:
    TeensySerialPortHandler(HardwareSerial& port, const int dir_pin = -1)
      : SerialPortHandler(port, dir_pin), port_(port), dir_pin_(dir_pin) {}
    virtual void begin(unsigned long baud) override
    {
      baud_ = baud;

      if (dir_pin_ != -1) {
        port_.begin(baud_);
        port_.transmitterEnable(dir_pin_);
      } else {
        port_.begin(baud_, SERIAL_HALF_DUPLEX);
      }

      setOpenState(true);
    }

    virtual void end(void) override
    {
      port_.end();
      setOpenState(false);
    }

  private:
    HardwareSerial& port_;
    const int dir_pin_;
    uint32_t baud_;
};


Dynamixel2Arduino dxl;
TeensySerialPortHandler dxl_port(DXL_SERIAL, DXL_DIR_PIN);
#else
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // boards like opencm and openCR...
#endif
BioloidInterpolate bioloid(dxl, GOAL_POSITION_REG);

//---------------------------------------------------------------

// Some forward references
#ifdef OPT_FIND_SERVO_OFFSETS
extern void FindServoOffsets();
#endif

#if defined(ARDUINO_OpenCM904)
extern void printMemoryUsage();
#else
void printMemoryUsage() {}
#endif

//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void DynamixelServoDriver::Init(void) {
#ifdef DBGSerial
  DBGSerial.println("At start of DynamixelServoDriver::Init");
#endif
  printMemoryUsage();
  // First lets get the actual servo positions for all of our servos...
  //  pinMode(0, OUTPUT);
#if defined(USE_USB_SERIAL_DXL)
  if (!g_myusb_begun) {
    myusb.begin();
    g_myusb_begun = true;
  }

#ifdef DBGSerial
  DBGSerial.println(F("Waiting for USB Serial"));
#endif
  elapsedMillis em;
  while (!userial && (em < 2000)) yield();
  if (userial) {
#ifdef DBGSerial
    DBGSerial.println(F("\n*** userial connected ***"));
    const uint8_t* psz = userial.manufacturer();
    if (psz && *psz) {
      DBGSerial.print("  manufacturer: ");
      DBGSerial.println(psz);
    }
    const uint8_t* pszProduct = userial.product();
    if (pszProduct && *pszProduct) {
      DBGSerial.print("  product: ");
      DBGSerial.println(psz);
    }
    psz = userial.serialNumber();
    if (psz && *psz) DBGSerial.print("  Serial: "); DBGSerial.println(psz);
  }
#endif
}
userial.begin(DXL_BAUD);  // need to start off this one manually.
bioloid.begin(DXL_BAUD, &userial);

#else
#ifdef TEENSYDUINO
  dxl.setPort(dxl_port);
#endif
  dxl.begin(DXL_BAUD);
#if !defined(DXL_DIR_PIN) && (DXL_DIR_PIN != -1)
#ifndef DXL_SERIAL
#define DXL_SERIAL
#endif
  DXL_SERIAL.begin(DXL_BAUD, SERIAL_HALF_DUPLEX);
#endif
  dxl.setPortProtocolVersion((DYNAMIXEL_PROTOCOL == 2) ? 2.0 : 1.0);
  bioloid.begin();
#endif
#ifdef DBGSerial
#define xstr(s) str(s)
#define str(s) #s
DBGSerial.print("DXL_SERIAL: " ); DBGSerial.print(xstr(DXL_SERIAL));
DBGSerial.print(" DIR PIN: "); DBGSerial.print(DXL_DIR_PIN, DEC);
DBGSerial.print(" Protocol: "); DBGSerial.print(DYNAMIXEL_PROTOCOL, DEC);
DBGSerial.print(" Baud: "); DBGSerial.println(DXL_BAUD);
DBGSerial.println("After dxl/bioloid begin");
printMemoryUsage();
#endif
_fServosFree = true;
bioloid.poseSize = NUMSERVOS;
#ifdef OPT_CHECK_SERVO_RESET
uint16_t w;
float   f;
int     count_missing = 0;
int     missing_servo = -1;
bool    servo_1_in_table = false;

#ifdef DBGSerial
f = dxl.getPresentPosition(2);
DBGSerial.print("Get position 2: "); DBGSerial.flush();
DBGSerial.print(f, 3);
DBGSerial.println("After first getPresentPosition call");
printMemoryUsage();
#endif

for (int i = 0; i < NUMSERVOS; i++) {
  // Set the id
  uint8_t servo_id = cPinTable[i];
  bioloid.setId(i, servo_id);

  if (cPinTable[i] == 1)
    servo_1_in_table = true;

  // Now try to get it's position
  f = dxl.getPresentPosition(servo_id);
  if ((f == 0.0) && (dxl.getLastLibErrCode() != DXL_LIB_OK)) {
    // Try a second time to make sure.
    delay(25);
    f = dxl.getPresentPosition(servo_id);
    if ((f == 0.0) && (dxl.getLastLibErrCode() != DXL_LIB_OK)) {
      // We have a failure
#ifdef DBGSerial
      DBGSerial.print(F("Servo("));
      DBGSerial.print(i, DEC);
      DBGSerial.print(F("): "));
      DBGSerial.print(servo_id, DEC);
      DBGSerial.println(F(" not found"));
#endif
      if (++count_missing == 1)
        missing_servo = servo_id;
    }
  }
  delay(25);
}

// Now see if we should try to recover from a potential servo that renumbered itself back to 1.
#ifdef DBGSerial
if (count_missing) {
  DBGSerial.print(F("ERROR: Servo driver init: "));
  DBGSerial.print(count_missing, DEC);
  DBGSerial.println(F(" servos missing"));
}
#endif

if (count_missing && !servo_1_in_table) {
  // Lets see if Servo 1 exists...
  w = static_cast<uint16_t>(dxl.getPresentPosition(1));
  if ((w != 0) || (dxl.getLastLibErrCode() == DXL_LIB_OK)) {
    if (count_missing == 1) {
#ifdef DBGSerial
      DBGSerial.print(F("Servo recovery: Servo 1 found - setting id to "));
      DBGSerial.println(missing_servo, DEC);
#endif
      dxl.setID(1, missing_servo);
      //ax12SetRegister(1, AX_ID, missing_servo);
    } else {
#ifdef DBGSerial
      DBGSerial.println(F("Servo recovery: Servo 1 found - Multiple missing led on 1 set"));
#endif
      dxl.ledOn(missing_servo);
      //ax12SetRegister(1, ControlTableItem::LED, 1);
      //ax12ReadPacket(6);  // get the response...
    }
  } else {
#ifdef DBGSerial
    DBGSerial.println(F("Servo recovery: Servo 1 NOT found"));
#endif
  }
}

#else
bioloid.readPose();
#endif
#ifdef cVoltagePin
for (byte i = 0; i < 8; i++)
  GetBatteryVoltage();  // init the voltage pin
#endif

_fAXSpeedControl = false;//Zenta, the speed experiment is turned off


// Currently have Turret pins not necessarily same as numerical order so
// Maybe should do for all pins and then set the positions by index instead
// of having it do a simple search on each pin...
#ifdef cTurretRotPin
bioloid.setId(FIRSTTURRETPIN, cTurretRotPin);
bioloid.setId(FIRSTTURRETPIN + 1, cTurretTiltPin);
#endif

// Added - try to speed things up later if we do a query...
SetRegOnAllServos(ControlTableItem::RETURN_DELAY_TIME, 0);  // tell servos to give us back their info as quick as they can...

}


//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef cVoltagePin
word  g_awVoltages[8] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
word  g_wVoltageSum = 0;
byte  g_iVoltages = 0;

word DynamixelServoDriver::GetBatteryVoltage(void) {
  g_iVoltages++;
  g_iVoltages &= 0x7;  // setup index to our array...
  g_wVoltageSum -= g_awVoltages[g_iVoltages];
  g_awVoltages[g_iVoltages] = analogRead(cVoltagePin);
  g_wVoltageSum += g_awVoltages[g_iVoltages];


#ifdef CVREF
  return ((long)((long)g_wVoltageSum * CVREF * (CVADR1 + CVADR2)) / (long)(8192 * (long)CVADR2)); //8192
#else
  return ((long)((long)g_wVoltageSum * 125 * (CVADR1 + CVADR2)) / (long)(2048 * (long)CVADR2));
#endif
}

#else
word g_wLastVoltage = 0xffff;    // save the last voltage we retrieved...
byte g_bLegVoltage = 0;   // what leg did we last check?
unsigned long g_ulTimeLastBatteryVoltage;

word DynamixelServoDriver::GetBatteryVoltage(void) {
  // In this case, we have to ask a servo for it's current voltage level, which is a lot more overhead than simply doing
  // one AtoD operation.  So we will limit when we actually do this to maybe a few times per second.
  // Also if interpolating, the code will try to only call us when it thinks it won't interfer with timing of interpolation.
  unsigned long ulDeltaTime = millis() - g_ulTimeLastBatteryVoltage;
  if (g_wLastVoltage != 0xffff) {
    if ( (ulDeltaTime < VOLTAGE_MIN_TIME_BETWEEN_CALLS)
         || (bioloid.interpolating &&  (ulDeltaTime < VOLTAGE_MAX_TIME_BETWEEN_CALLS)))
      return g_wLastVoltage;
  }

  // Lets cycle through the Tibia servos asking for voltages as they may be the ones doing the most work...
  register word wVoltage = dxl.readControlTableItem(ControlTableItem::PRESENT_VOLTAGE, pgm_read_byte(&cPinTable[FIRSTTIBIAPIN + g_bLegVoltage]));

  if (++g_bLegVoltage >= CNT_LEGS)
    g_bLegVoltage = 0;
  if (wVoltage != 0xffff) {
    g_ulTimeLastBatteryVoltage = millis();
    g_wLastVoltage = wVoltage * 10;
    return g_wLastVoltage;
  }

  // Allow it to error our a few times, but if the time until we get a valid response exceeds some time limit then error out.
  if (ulDeltaTime < VOLTAGE_TIME_TO_ERROR)
    return g_wLastVoltage;
  return 0;

}
#endif


//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void DynamixelServoDriver::BeginServoUpdate(void)    // Start the update
{
  MakeSureServosAreOn();
  if (ServosEnabled) {
    DebugToggle(A4);
    if (_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // If we are trying our own Servo control need to save away the new positions...
      for (byte i = 0; i < NUMSERVOS; i++) {
        _awCurAXPos[i] = _awGoalAXPos[i];
      }
#endif
    }
    else
      bioloid.interpolateStep(true);    // Make sure we call at least once

  }
}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#ifdef c4DOF
void DynamixelServoDriver::OutputServoInfoForLeg(int LegIndex, float CoxaAngle, float FemurAngle, float TibiaAngle, float TarsAngle)
#else
void DynamixelServoDriver::OutputServoInfoForLeg(int LegIndex, float CoxaAngle, float FemurAngle, float TibiaAngle)
#endif
{
  word    wCoxaSDV;        // Coxa value in servo driver units
  word    wFemurSDV;        //
  word    wTibiaSDV;        //
#ifdef c4DOF
  word    wTarsSDV;        //
#endif
  // The Main code now takes care of the inversion before calling.
  wCoxaSDV = CoxaAngle * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE;
  wFemurSDV = FemurAngle * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE;
  wTibiaSDV = TibiaAngle * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE;
#ifdef c4DOF
  wTarsSDV = TarsAngle * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE;
#endif
  if (ServosEnabled) {
    if (_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // Save away the new positions...
      _awGoalAXPos[FIRSTCOXAPIN + LegIndex] = wCoxaSDV;  // What order should we store these values?
      _awGoalAXPos[FIRSTFEMURPIN + LegIndex] = wFemurSDV;
      _awGoalAXPos[FIRSTTIBIAPIN + LegIndex] = wTibiaSDV;
#ifdef c4DOF
      g_awGoalAXTarsPos[FIRSTTARSPIN + LegIndex] = wTarsSDV;
#endif
#endif
    }
    else {
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTCOXAPIN + LegIndex]), wCoxaSDV);
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTFEMURPIN + LegIndex]), wFemurSDV);
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTIBIAPIN + LegIndex]), wTibiaSDV);
#ifdef c4DOF
      if ((byte)pgm_read_byte(&cTarsLength[LegIndex]))   // We allow mix of 3 and 4 DOF legs...
        bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTARSPIN + LegIndex]), wTarsSDV);
#endif
    }
  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput) {
    DBGSerial.print(LegIndex, DEC);
    DBGSerial.print(F("("));
    DBGSerial.print(sCoxaAngle1, DEC);
    DBGSerial.print(F("="));
    DBGSerial.print(wCoxaSDV, DEC);
    DBGSerial.print(F("),("));
    DBGSerial.print(sFemurAngle1, DEC);
    DBGSerial.print(F("="));
    DBGSerial.print(wFemurSDV, DEC);
    DBGSerial.print(F("),("));
    DBGSerial.print(F("("));
    DBGSerial.print(sTibiaAngle1, DEC);
    DBGSerial.print(F("="));
    DBGSerial.print(wTibiaSDV, DEC);
    DBGSerial.print(F(") :"));
  }
#endif
  InputController::controller()->AllowControllerInterrupts(true);    // Ok for hserial again...
}


//==============================================================================
// Calculate servo speeds to achieve desired pose timing
// We make the following assumptions:
// AX-12 speed is 59rpm @ 12V which corresponds to 0.170s/60deg
// The AX-12 manual states this as the 'no load speed' at 12V
// The Moving Speed control table entry states that 0x3FF = 114rpm
// and according to Robotis this means 0x212 = 59rpm and anything greater 0x212 is also 59rpm
#ifdef USE_AX12_SPEED_CONTROL
word CalculateAX12MoveSpeed(word wCurPos, word wGoalPos, word wTime)
{
  word wTravel;
  uint32_t factor;
  word wSpeed;
  // find the amount of travel for each servo
  if ( wGoalPos > wCurPos) {
    wTravel = wGoalPos - wCurPos;
  }
  else {
    wTravel = wCurPos - wGoalPos;
  }

  // now we can calculate the desired moving speed
  // for 59pm the factor is 847.46 which we round to 848
  // we need to use a temporary 32bit integer to prevent overflow
  factor = (uint32_t) 848 * wTravel;

  wSpeed = (uint16_t) ( factor / wTime );
  // if the desired speed exceeds the maximum, we need to adjust
  if (wSpeed > 1023) wSpeed = 1023;
  // we also use a minimum speed of 26 (5% of 530 the max value for 59RPM)
  if (wSpeed < 26) wSpeed = 26;

  return wSpeed;
}
#endif

//------------------------------------------------------------------------------------------
//[OutputServoInfoForTurret] Set up the outputse servos associated with an optional turret
//         the Leg number passed in.  FIRSTTURRETPIN
//------------------------------------------------------------------------------------------
#ifdef cTurretRotPin
void DynamixelServoDriver::OutputServoInfoForTurret(float RotateAngle, float TiltAngle)
{
  word    wRotateSDV;
  word    wTiltSDV;        //

  // The Main code now takes care of the inversion before calling.
  wRotateSDV = RotateAngle * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE;
  wTiltSDV = TiltAngle * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE;

  if (ServosEnabled) {
    if (_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // Save away the new positions...
      _awGoalAXPos[FIRSTTURRETPIN] = wRotateSDV;    // What order should we store these values?
      _awGoalAXPos[FIRSTTURRETPIN + 1] = wTiltSDV;
#endif
    }
    else {
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTURRETPIN]), wRotateSDV);
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTURRETPIN + 1]), wTiltSDV);
    }
  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput) {
    DBGSerial.print(F("("));
    DBGSerial.print(sRotateAngle1, DEC);
    DBGSerial.print(F("="));
    DBGSerial.print(wRotateSDV, DEC);
    DBGSerial.print(F("),("));
    DBGSerial.print(sTiltAngle1, DEC);
    DBGSerial.print(F("="));
    DBGSerial.print(wTiltSDV, DEC);
    DBGSerial.print(F(") :"));
  }
#endif
}
#endif
//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly
//        get the next command to start
//--------------------------------------------------------------------
void DynamixelServoDriver::CommitServoDriver(word wMoveTime)
{
#ifdef cSSC_BINARYMODE
  byte    abOut[3];
#endif

  InputController::controller()->AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
  if (ServosEnabled) {
    if (_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // Need to first output the header for the Sync Write
      int length = 4 + (NUMSERVOS * 5);   // 5 = id + pos(2byte) + speed(2 bytes)
      int checksum = 254 + length + AX_SYNC_WRITE + 4 + AX_GOAL_POSITION_L;
      word wSpeed;
      setTXall();
      ax12write(0xFF);
      ax12write(0xFF);
      ax12write(0xFE);
      ax12write(length);
      ax12write(AX_SYNC_WRITE);
      ax12write(AX_GOAL_POSITION_L);
      ax12write(4);    // number of bytes per servo (plus the ID...)
      for (int i = 0; i < NUMSERVOS; i++) {
        wSpeed = CalculateAX12MoveSpeed(_awCurAXPos[i], _awGoalAXPos[i], wMoveTime);    // What order should we store these values?
        byte id = pgm_read_byte(&cPinTable[i]);
        checksum += id + (_awGoalAXPos[i] & 0xff) + (_awGoalAXPos[i] >> 8) + (wSpeed >> 8) + (wSpeed & 0xff);
        ax12write(id);
        ax12write(_awGoalAXPos[i] & 0xff);
        ax12write(_awGoalAXPos[i] >> 8);
        ax12write(wSpeed & 0xff);
        ax12write(wSpeed >> 8);

      }
      ax12write(0xff - (checksum % 256));
      setRX(0);

#endif
    }
    else {
      bioloid.interpolateSetup(wMoveTime);
    }

  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput)
    DBGSerial.println(wMoveTime, DEC);
#endif
  InputController::controller()->AllowControllerInterrupts(true);

}
//--------------------------------------------------------------------
//[SetRegOnAllServos] Function that is called to set the state of one
//  register in all of the servos, like Torque on...
//--------------------------------------------------------------------
void DynamixelServoDriver::SetRegOnAllServos(uint8_t bReg, int32_t bVal, bool checkStatusErrors)
{
  // Do quick and dirty...
  for (int i = 0; i < NUMSERVOS; i++) {
    byte id = pgm_read_byte(&cPinTable[i]);
    dxl.writeControlTableItem(bReg, id, bVal);
    if (checkStatusErrors) {
      uint8_t status_error = dxl.getLastStatusPacketError();
      if (status_error) {
#ifdef DBGSerial
        DBGSerial.print("SetRegOnAllServos HW Status error: "); DBGSerial.print(status_error, HEX);
        DBGSerial.print(" on ID:"); DBGSerial.print(id, DEC);
        DBGSerial.println(" - Resetting");
#endif
        dxl.reboot(id);
        delay(100); // give it some time...
        dxl.writeControlTableItem(bReg, id, bVal);  // try setting again
      }
    }

#ifdef DBGSerial
    int error_code = dxl.getLastLibErrCode();
    if (error_code) {
      DBGSerial.print("SetRegOnAllServos failed: id:"); DBGSerial.print(id, DEC);
      DBGSerial.print(" Reg:"); DBGSerial.print(bReg, DEC);
      DBGSerial.print(" Val:"); DBGSerial.print(bVal, DEC);
      DBGSerial.print(" Error:"); DBGSerial.println(error_code, DEC);
    }
#endif
  }
}
//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void DynamixelServoDriver::FreeServos(void)
{
  if (!_fServosFree) {
    InputController::controller()->AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    SetRegOnAllServos2(ControlTableItem::TORQUE_LIMIT, 50);  // reduce to real slow...
    //SetRegOnAllServos(ControlTableItem::TORQUE_ENABLE, 50);  //First reduce the torque to very low
    delay(250);                               //Then a short break until turning off torque
    SetRegOnAllServos(ControlTableItem::TORQUE_ENABLE, 0);  // do this as one statement...
#if 0
    for (byte i = 0; i < NUMSERVOS; i++) {
      Relax(pgm_read_byte(&cPinTable[i]));
    }
#endif
    InputController::controller()->AllowControllerInterrupts(true);
    _fServosFree = true;
#ifdef DBGSerial
    DBGSerial.println(F("(FreeServos)---> Servos Are Free"));
#endif
  }
}

//--------------------------------------------------------------------
//Function that gets called from the main loop if the robot is not logically
//     on.  Gives us a chance to play some...
//--------------------------------------------------------------------
static uint8_t g_iIdleServoNum  = (uint8_t) - 1;
static uint8_t g_iIdleLedState = 1;  // what state to we wish to set...
void DynamixelServoDriver::IdleTime(void)
{
  // Each time we call this set servos LED on or off...
  g_iIdleServoNum++;
  if (g_iIdleServoNum >= NUMSERVOS) {
    g_iIdleServoNum = 0;
    g_iIdleLedState = 1 - g_iIdleLedState;
  }
  if (g_iIdleLedState) dxl.ledOn(cPinTable[g_iIdleServoNum]);
  else dxl.ledOff(cPinTable[g_iIdleServoNum]);
}

//--------------------------------------------------------------------
//[MakeSureServosAreOn] Function that is called to handle when you are
//  transistioning from servos all off to being on.  May need to read
//  in the current pose...
//--------------------------------------------------------------------
void DynamixelServoDriver::MakeSureServosAreOn(void)
{
  if (ServosEnabled) {
    if (!_fServosFree)
      return;    // we are not free

    InputController::controller()->AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    if (_fAXSpeedControl) {
      /*
        for(int i=0;i<NUMSERVOS;i++){
        _awGoalAXPos[i] = dxl.getPresentPosition(cPinTable[i]);
        delay(25);
        }
      */
    }
    else {
      bioloid.readPose();
    }

#ifdef DBGSerial
    DBGSerial.println(F("(MakeSureServosAreOn)---> Setting TorqL to 256\n"));
#endif
    SetRegOnAllServos(ControlTableItem::TORQUE_ENABLE, 1, true);  // Use sync write to do it. Maybe reset as well
    SetRegOnAllServos2(ControlTableItem::TORQUE_LIMIT, 256);//Start with very low torque
    SetRegOnAllServos(ControlTableItem::LED, 0);           // turn off all servos LEDs.

#if 0
    for (byte i = 0; i < NUMSERVOS; i++) {
      TorqueOn(pgm_read_byte(&cPinTable[i]));
    }
#endif
    InputController::controller()->AllowControllerInterrupts(true);
    _fServosFree = false;
  }
}

//==============================================================================
// BackgroundProcess - Allows us to have some background processing for those
//    servo drivers that need us to do things like polling...
//==============================================================================
void  DynamixelServoDriver::BackgroundProcess(void)
{
  if (_fAXSpeedControl)
    return;  // nothing to do in this mode...

  if (ServosEnabled) {
    DebugToggle(A3);

    int iTimeToNextInterpolate __attribute__((unused)) = bioloid.interpolateStep(false);    // Do our background stuff...

    // Hack if we are not interpolating, maybe try to get voltage.  This will acutally only do this
    // a few times per second.
#ifdef cTurnOffVol          // only do if we a turn off voltage is defined
#ifndef cVoltagePin         // and we are not doing AtoD type of conversion...
    if (iTimeToNextInterpolate > VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE )      // At least 4ms until next interpolation.  See how this works...
      GetBatteryVoltage();
#endif
#endif
  }
}

//==============================================================================
// WakeUpRoutine - Wake up robot in a friendly way
//==============================================================================
void DynamixelServoDriver::WakeUpRoutine(void) {
  byte LegIndex;
  int CurrentCoxaPos;//was word, changed to integer to prevent since faulty reading return -1
  int CurrentFemurPos;
  int CurrentTibiaPos;
  boolean PosOK = true;
  uint32_t servos_still_waking_up = 0;
#define PosMargin ((SERVO_TIC_PER_DEG * 3) / 2)
  if (g_WakeUpState) { //Check if all servos has reached their goal position
    //ax12SetRegister(254, ControlTableItem::TORQUE_ENABLE, 1); //Using broadcast instead
    //delay(500); //Waiting half a second test bug bug
    InputController::controller()->AllowControllerInterrupts(false);
    /*for (LegIndex = 0; LegIndex < CNT_LEGS/2; LegIndex++) {//Right legs

      CurrentCoxaPos = ax12GetRegister(pgm_read_byte(&cPinTable[FIRSTCOXAPIN + LegIndex]), AX_PRESENT_POSITION_L, 2);
      CurrentFemurPos = ax12GetRegister(pgm_read_byte(&cPinTable[FIRSTFEMURPIN + LegIndex]), AX_PRESENT_POSITION_L, 2);
      CurrentTibiaPos = ax12GetRegister(pgm_read_byte(&cPinTable[FIRSTTIBIAPIN + LegIndex]), AX_PRESENT_POSITION_L, 2);
      if ((abs((int)CurrentCoxaPos - (int)(-CoxaAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE))> PosMargin) && (CurrentCoxaPos < ServoRes)) {//MUST adjust for the MX! (4096) <1024 (AX) mean that we ignore the faulty readings
        PosOK = false;
      }
      if ((abs((int)CurrentFemurPos - (int)(-FemurAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE))> PosMargin) && (CurrentFemurPos < ServoRes)) {
        PosOK = false;
      }
      if ((abs((int)CurrentTibiaPos - (int)(-TibiaAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE))> PosMargin) && (CurrentTibiaPos < ServoRes)) {
        PosOK = false;
      }

      #ifdef DEBUG_WakeUp_Pos
      DBGSerial.print(CurrentCoxaPos, DEC);
      DBGSerial.print(F("-"));
      DBGSerial.print((int)(-CoxaAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE), DEC);//must invert Right legs
      DBGSerial.print(F(" "));
      DBGSerial.print(CurrentFemurPos, DEC);
      DBGSerial.print(F("-"));
      DBGSerial.print((int)(-FemurAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE), DEC);
      DBGSerial.print(F(" "));
      DBGSerial.print(CurrentTibiaPos, DEC);
      DBGSerial.print(F("-"));
      DBGSerial.print((int)(-TibiaAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE), DEC);
      DBGSerial.print(F(" _ "));
      #endif
      delay(25);
      }*/
    uint32_t servo_leg_mask = 0x1;
    for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++, servo_leg_mask <<= 1) { //for (LegIndex = CNT_LEGS / 2; LegIndex < CNT_LEGS; LegIndex++) {//Left legs
      // if we get read failures.. Try an extra fetch
      CurrentCoxaPos = static_cast<uint16_t>(dxl.getPresentPosition(pgm_read_byte(&cPinTable[FIRSTCOXAPIN + LegIndex])));
      if ((CurrentCoxaPos == 0) && (dxl.getLastLibErrCode() != DXL_LIB_OK)) CurrentCoxaPos = static_cast<uint16_t>(dxl.getPresentPosition(pgm_read_byte(&cPinTable[FIRSTCOXAPIN + LegIndex])));
      CurrentFemurPos = static_cast<uint16_t>(dxl.getPresentPosition(pgm_read_byte(&cPinTable[FIRSTFEMURPIN + LegIndex])));
      if ((CurrentFemurPos == 0) && (dxl.getLastLibErrCode() != DXL_LIB_OK)) CurrentFemurPos = static_cast<uint16_t>(dxl.getPresentPosition(pgm_read_byte(&cPinTable[FIRSTFEMURPIN + LegIndex])));
      CurrentTibiaPos = static_cast<uint16_t>(dxl.getPresentPosition(pgm_read_byte(&cPinTable[FIRSTTIBIAPIN + LegIndex])));
      if ((CurrentTibiaPos == 0) && (dxl.getLastLibErrCode() != DXL_LIB_OK)) CurrentTibiaPos = static_cast<uint16_t>(dxl.getPresentPosition(pgm_read_byte(&cPinTable[FIRSTTIBIAPIN + LegIndex])));


      if ((abs((int)CurrentCoxaPos - (int)(CoxaAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE)) > PosMargin) && ((CurrentCoxaPos < ServoRes)) && (CurrentCoxaPos >= 0)) {
        PosOK = false;
        servos_still_waking_up |= servo_leg_mask;
      }
      if ((abs((int)CurrentFemurPos - (int)(FemurAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE)) > PosMargin) && ((CurrentFemurPos < ServoRes)) && (CurrentFemurPos >= 0)) {
        servos_still_waking_up |= (servo_leg_mask << 8);
        PosOK = false;
      }
      if ((abs((int)CurrentTibiaPos - (int)(TibiaAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE)) > PosMargin) && ((CurrentTibiaPos < ServoRes)) && (CurrentTibiaPos >= 0)) {
        servos_still_waking_up |= (servo_leg_mask << 16);
        PosOK = false;
      }
#ifdef DEBUG_WakeUp_Pos
      DBGSerial.print(CurrentCoxaPos, DEC);
      DBGSerial.print((servos_still_waking_up & servo_leg_mask) ? F("-") : F("="));
      DBGSerial.print((int)(CoxaAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE), DEC);//must invert Right legs
      DBGSerial.print(F(" "));
      DBGSerial.print(CurrentFemurPos, DEC);
      DBGSerial.print((servos_still_waking_up & (servo_leg_mask << 8)) ? F("-") : F("="));
      DBGSerial.print((int)(FemurAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE), DEC);
      DBGSerial.print(F(" "));
      DBGSerial.print(CurrentTibiaPos, DEC);
      DBGSerial.print((servos_still_waking_up & (servo_leg_mask << 16)) ? F("-") : F("="));
      DBGSerial.print((int)(TibiaAngle[LegIndex] * SERVO_TIC_PER_DEG + SERVO_CENTER_VALUE), DEC);
      DBGSerial.print(F(" _ "));
#endif
      delay(25);
    }
    //InputController::controller()->AllowControllerInterrupts(true);
#ifdef DEBUG_WakeUp_Pos
    DBGSerial.print(servos_still_waking_up, HEX);
    DBGSerial.print(F(" "));
    DBGSerial.println(PosOK, DEC);
#endif
    if ((millis() - lWakeUpStartTime) > 6000) {
      SetControllerMsg(1, "Wakeup Timeout!!!");
      MSound(1, 150, 1500);// Make some sound if it takes more than 6 second to get into wakeup position, something is probably wrong..
#ifdef DEBUG_WakeUp_Pos
      DBGSerial.print("*** Wakeup Timeout ***");
      DBGSerial.print("Should we proceed? (Y/...)");
      int ch;
      while (DBGSerial.read() != -1);
      while ((ch = DBGSerial.read()) == -1) ;
      while (DBGSerial.read() != -1);
      if ((ch == 'y') || (ch == 'Y')) PosOK = true;
#endif
      lWakeUpStartTime = millis();  // clear out time
    }
    if (PosOK) { // All servos are in position, ready for turning on full torque!
      //InputController::controller()->AllowControllerInterrupts(false);

      g_WakeUpState = false;
#ifdef SafetyMode
      //SetRegOnAllServos(ControlTableItem::TORQUE_ENABLE, 1);  // Use sync write to do it.
      ax12SetRegister(254, ControlTableItem::TORQUE_ENABLE, 1); //Using broadcast instead
      delay(500); //Waiting half a second test bug bug
      ax12SetRegister2(254, ControlTableItem::TORQUE_LIMIT, 300); //Reduced Torque
      //SetRegOnAllServos2(ControlTableItem::TORQUE_LIMIT, 1023);//Turn on full Torque
      DBGSerial.println(F("(WakeUpRoutine) --> SafetyMode\n"));
#else
      SetRegOnAllServos(ControlTableItem::TORQUE_ENABLE, 1);  // Use sync write to do it.
      delay(500); //Waiting half a second test bug bug
      SetRegOnAllServos2(ControlTableItem::TORQUE_LIMIT, 1023);// Set full torque
      DBGSerial.println(F("(WakeUpRoutine) --> Set torque to max!"));
#endif
      InputController::controller()->AllowControllerInterrupts(true);
      MSound(1, 80, 2000);
      g_InControlState.ForceSlowCycleWait = 2;//Get ready slowly

      SetControllerMsg(1, "Ready!");

      for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
        cInitPosY[LegIndex] = cHexGroundPos;//Lower the legs to ground
      }
    }
  }
}

#ifdef OPT_TERMINAL_MONITOR
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void DynamixelServoDriver::ShowTerminalCommandList(void)
{
  DBGSerial.println(F("V - Voltage"));
  DBGSerial.println(F("M - Toggle Motors on or off"));
  DBGSerial.println(F("F<frame length> - FL in ms"));    // BUGBUG::
  DBGSerial.println(F("A - Toggle AX12 speed control"));
  DBGSerial.println(F("T - Test Servos"));
  DBGSerial.println(F("W<ID> - Wiggle servo ID... "));
  DBGSerial.println(F("I - Set Id <frm> <to"));
  DBGSerial.println(F("S - Track Servos"));
#ifdef OPT_PYPOSE
  DBGSerial.println(F("P<DL PC> - Pypose"));
#endif
#ifdef OPT_FIND_SERVO_OFFSETS
  DBGSerial.println(F("O - Enter Servo offset mode"));
#endif
}

//==============================================================================
// ProcessTerminalCommand: The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
boolean DynamixelServoDriver::ProcessTerminalCommand(byte *psz, byte bLen)
{
  if ((bLen == 1) && ((*psz == 'm') || (*psz == 'M'))) {
    g_fEnableServos = !g_fEnableServos;
    if (g_fEnableServos)
      DBGSerial.println(F("Motors are on"));
    else
      DBGSerial.println(F("Motors are off"));

    return true;
  }
  if ((bLen == 1) && ((*psz == 'v') || (*psz == 'V'))) {
    DBGSerial.print(F("Voltage: "));
    DBGSerial.println(GetBatteryVoltage(), DEC);
#ifdef cVoltagePin
    DBGSerial.print(F("Raw Analog: "));
    DBGSerial.println(analogRead(cVoltagePin));
#endif
    DBGSerial.print(F("From Servo 2: "));
    DBGSerial.println(dxl.readControlTableItem(ControlTableItem::PRESENT_VOLTAGE, 2), DEC);
  }

  if ((bLen == 1) && ((*psz == 't') || (*psz == 'T'))) {
    TCTestServos();
  }
  if ((*psz == 'w') || (*psz == 'W')) {
    TCWiggleServo(++psz);
  }

  if ((*psz == 'i') || (*psz == 'I')) {
    TCSetServoID(++psz);
  }
  if ((*psz == 's') || (*psz == 'S')) {
    TCTrackServos();
  }

  if ((bLen == 1) && ((*psz == 'a') || (*psz == 'A'))) {
    _fAXSpeedControl = !_fAXSpeedControl;
    if (_fAXSpeedControl)
      DBGSerial.println(F("AX12 Speed Control"));
    else
      DBGSerial.println(F("Bioloid Speed"));
  }
  if ((bLen >= 1) && ((*psz == 'f') || (*psz == 'F'))) {
    psz++;  // need to get beyond the first character
    while (*psz == ' ')
      psz++;  // ignore leading blanks...
    byte bFrame = 0;
    while ((*psz >= '0') && (*psz <= '9')) {  // Get the frame count...
      bFrame = bFrame * 10 + *psz++ - '0';
    }
    if (bFrame != 0) {
      DBGSerial.print(F("New Servo Cycles per second: "));
      DBGSerial.println(1000 / bFrame, DEC);
      //extern BioloidControllerEx bioloid;
      //bioloid.frameLength = bFrame;
    }
  }

#ifdef OPT_FIND_SERVO_OFFSETS
  else if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
    FindServoOffsets();
  }
#endif
  return false;

}

//==============================================================================
// TCSetServoID - debug function to update servo numbers.
//==============================================================================
void DynamixelServoDriver::TCSetServoID(byte *psz)
{
  word wFrom = GetCmdLineNum(&psz);
  word wTo = GetCmdLineNum(&psz);

  if (wFrom  && wTo) {
    // Both specified, so lets try
    DBGSerial.print(F("Change Servo from: "));
    DBGSerial.print(wFrom, DEC);
    DBGSerial.print(F(" "));
    DBGSerial.print(wTo, DEC);
    if (dxl.setID(wFrom, wTo)) {
      DBGSerial.println(F(" Succeeded"));
    }
    else
      DBGSerial.println(F(" failed"));
  }
}

//==============================================================================
// TCWiggleServo - debug function wiggle specified servo
//==============================================================================
void DynamixelServoDriver::TCWiggleServo(byte *psz)
{
  int servo_id = GetCmdLineNum(&psz);
  int error_code;

  if (!servo_id)return;
  DBGSerial.print("Try to wiggle: "); DBGSerial.println(servo_id, DEC);
  dxl.torqueOn(servo_id);
  dxl.writeControlTableItem(ControlTableItem::TORQUE_LIMIT, servo_id, 1023);
  float servo_pos = dxl.getPresentPosition(servo_id);
  DBGSerial.print("Start from: "); DBGSerial.println(servo_pos, 1);

  if ((error_code = dxl.getLastLibErrCode())) {
    DBGSerial.print("Failed: "); DBGSerial.println(error_code, DEC);
    return;
  }


  // lets move from current, to CENTER-20 degrees, CENTER+20 degrees, back to center.
  float target_pos = SERVO_CENTER_VALUE - (45 * SERVO_TIC_PER_DEG);
  float target_delta = (float)SERVO_TIC_PER_DEG / 3.0;
  DBGSerial.print("Target: "); 
  DBGSerial.print(target_pos, 1); 
  DBGSerial.print(" delta: "); 
  DBGSerial.println(target_delta, 3);

  // Quick and dirty.
  while (servo_pos > target_pos) {
    servo_pos -= target_delta;
    if (servo_pos < target_pos) servo_pos = target_pos;
    dxl.setGoalPosition(servo_id, servo_pos);
    delay(10);
  };

  target_pos = SERVO_CENTER_VALUE + (45 * SERVO_TIC_PER_DEG);
  DBGSerial.print("Target: "); DBGSerial.print(target_pos, 1); 
  DBGSerial.print(" delta: "); DBGSerial.println(target_delta, 3);

  // Quick and dirty.
  while (servo_pos < target_pos) {
    servo_pos += target_delta;
    if (servo_pos > target_pos) servo_pos = target_pos;
    dxl.setGoalPosition(servo_id, servo_pos);
    delay(10);
  };

  target_pos = SERVO_CENTER_VALUE;
  DBGSerial.print("Target: "); DBGSerial.print(target_pos, 1); 
  DBGSerial.print(" delta: "); DBGSerial.println(target_delta, 3);
  while (servo_pos > target_pos) {
    servo_pos -= target_delta;
    if (servo_pos < target_pos) servo_pos = target_pos;
    dxl.setGoalPosition(servo_id, servo_pos);
    delay(10);
  };
}


//==============================================================================
// TCTestServos - Enum all of the Hexapod servos and show register details
//==============================================================================
void DynamixelServoDriver::TCTestServos()
{
  // Test to see if all servos are responding...
  bool servo_1_in_table = false;
  DBGSerial.println(F("Index\tID\tModel:Firm\tDelay\tPos\tAng\tVoltage\tTemp\tHW"));
  for (int i = 0; i < NUMSERVOS; i++) {
    uint32_t error_code;
    uint32_t errors[10];
    uint8_t error_count = 0;
    int servo_id = pgm_read_byte(&cPinTable[i]);
    if (servo_id == 1) servo_1_in_table = true;
    DBGSerial.print(i, DEC);
    DBGSerial.print(F("\t"));
    DBGSerial.print(servo_id, DEC);
    DBGSerial.print(F("\t"));
    DBGSerial.print(dxl.getModelNumber(servo_id), HEX);
    if ((error_code = dxl.getLastLibErrCode())) {
      errors[error_count] = error_code;
      error_count++;
    }
    DBGSerial.print(F("\t"));
    DBGSerial.print(dxl.readControlTableItem(ControlTableItem::FIRMWARE_VERSION, servo_id), HEX);
    if ((error_code = dxl.getLastLibErrCode())) {
      errors[error_count] = error_code;
      error_count++;
    }
    DBGSerial.print(F("\t"));
    DBGSerial.print(dxl.readControlTableItem(ControlTableItem::RETURN_DELAY_TIME, servo_id), DEC);
    if ((error_code = dxl.getLastLibErrCode())) {
      errors[error_count] = error_code;
      error_count++;
    }
    DBGSerial.print(F("\t"));
    DBGSerial.print(static_cast<uint16_t>(dxl.getPresentPosition(servo_id)), DEC);
    if ((error_code = dxl.getLastLibErrCode())) {
      errors[error_count] = error_code;
      error_count++;
    }
    DBGSerial.print(F("\t"));
    DBGSerial.print(dxl.getPresentPosition(servo_id, UNIT_DEGREE), 2);
    if ((error_code = dxl.getLastLibErrCode())) {
      errors[error_count] = error_code;
      error_count++;
    }
    DBGSerial.print(F("\t"));
    DBGSerial.print(dxl.readControlTableItem(ControlTableItem::PRESENT_VOLTAGE, servo_id), DEC);
    if ((error_code = dxl.getLastLibErrCode())) {
      errors[error_count] = error_code;
      error_count++;
    }
    DBGSerial.print(F("\t"));
    DBGSerial.print(dxl.readControlTableItem( ControlTableItem::PRESENT_TEMPERATURE, servo_id), DEC);
    if ((error_code = dxl.getLastLibErrCode())) {
      errors[error_count] = error_code;
      error_count++;
    }
    DBGSerial.print(F("\t"));
    DBGSerial.print(dxl.getLastStatusPacketError(), HEX);

    DBGSerial.print(F("\t"));
    DBGSerial.print(error_count, DEC);
    DBGSerial.print(F(":"));
    for (uint8_t i = 0; i < error_count; i++) {
      DBGSerial.print(errors[i], HEX);
      DBGSerial.print(" ");
    }
    DBGSerial.println();
    delay(25);
  }
  if (!servo_1_in_table) {
    DBGSerial.print(F("*(1)="));
    DBGSerial.print(static_cast<uint16_t>(dxl.getPresentPosition(1)), DEC);
    if (dxl.getLastLibErrCode()) DBGSerial.println(" ** Not Found **");
    else DBGSerial.print(" ** Found **");
  }
}
//==============================================================================
// TCTrackServos - Lets set a mode to track servos.  Can use to help figure out
// proper initial positions and min/max values...
//==============================================================================
void DynamixelServoDriver::TCTrackServos()
{
  // First read through all of the servos to get their position.
  uint16_t auPos[NUMSERVOS];
  uint16_t  uPos;
  int i;
  boolean fChange;

  // Clear out any pending input characters
  while (DBGSerial.read() != -1)
    ;

  for (i = 0; i < NUMSERVOS; i++) {
    auPos[i] = static_cast<uint16_t>(dxl.getPresentPosition(pgm_read_byte(&cPinTable[i])));
  }

  // Now loop until we get some input on the serial
  while (!DBGSerial.available()) {
    fChange = false;
    for (int i = 0; i < NUMSERVOS; i++) {
      uPos = static_cast<uint16_t>(dxl.getPresentPosition(pgm_read_byte(&cPinTable[i])));
      // Lets put in a littl delta or shows lots
      if (abs(auPos[i] - uPos) > 2) {
        auPos[i] = uPos;
        if (fChange)
          DBGSerial.print(F(", "));
        else
          fChange = true;
        DBGSerial.print(pgm_read_byte(&cPinTable[i]), DEC);
        DBGSerial.print(F(": "));
        DBGSerial.print(uPos, DEC);
        // Convert back to angle.
        float Ang = (float)(uPos - SERVO_CENTER_VALUE) / SERVO_TIC_PER_DEG;
        DBGSerial.print(F("("));
        DBGSerial.print(Ang, 2);
        DBGSerial.print(F(")"));
      }
    }
    if (fChange)
      DBGSerial.println();
    delay(25);
  }

}


#endif

//==============================================================================
//  FindServoOffsets - Find the zero points for each of our servos...
//    Will use the new servo function to set the actual pwm rate and see
//    how well that works...
//==============================================================================
#ifdef OPT_FIND_SERVO_OFFSETS

void FindServoOffsets()
{

}
#endif
