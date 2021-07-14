# AX12_Phoenix_PhantomX_float
Zenta's Phoeniz PhantomX Serial converted to Float
#### Joystick Commands

Command | Description
------------ | -------------
PS | Power On/Off
Cross | Joystick Debug
Triangle | Stand or sit
Up Arrow | go up to stand or plus 10
Dwn Arrow | go down 10
Left Arrow | Slow Down Gait
Right Arrow | Speed Up Gait
Square | Balance Mode On/Off
Start/Options | Motion Select
... | Translate
... | Rotate
... | Single
... | Walk
L1 | Leg Controls
... | Ry - Up/Dwn
... | Rx - Speed Up/Dwn
--- | Lx/Ly  manually adjust the initial leg positions
R1 | Hold Single leg in place when Single leg selected

### Available Commands in Certain Motions Selected

  Leg-Body Axes Schematic
  ![leg-body orientation](https://github.com/mjs513/LSS_Phoenix_BT/blob/main/LSS_PhoenixUSBJoystick/images/Leg-axis-orientation.png)

  ### Single Leg Motion 
Command | Description
------------ | -------------
Select | Left or right front leg
    
  ### Walk Motion
Command | Description
------------ | -------------
Options | Gait Select
... | Tripod 8
... | Tripple 12
... | Tripple 16
... | Wave 24
... | Tripod 6
... |  Ripple 12
R2 | Double Leg Lift
Circle | Walkmode 1 (Auto)
... | Circle (toggle) Walkmode 0 (Manual Control)

  ### Rotate Motion
Command | Description
------------ | -------------
X axis | Use Ly stick
Y axis | Use Rx stick
Z axis | Use Lx stick
  
  ### Translate Motion
Command | Description
------------ | -------------
X axis | Use Lx stick
Y axis | Use Rx stick
Z axis | Use Ly stick

  
### Joystick Bluetooth pairing (PS3 and PS4)

For information on which Bluetooth dongles and joysticks that we have tried and 
hopefully working, see the PJRC Forum thread: https://forum.pjrc.com/threads/49358-T3-6-USB-Host-Bluetooth

To Pair a PS4 to a bluetooth dongle in a relatively easier manor. 

    Edit the LSS_Phoenix_Input_USB_Joystick.cpp to uncomment the line that says pairing and comment out the next line - that line is used when you are already paired.

```
  #if defined(BLUETOOTH)
    //BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
    BluetoothController bluet(myusb);   // version assumes it already was paired
```

Once you boot up with this version of the code:
Hold down the Share button and PS button at the same time, until the PS4, starts a sort of double blink… Hopefully then it pairs. Then assuming it paired, you change the two lines back and rebuild again.

With the BLUETOOTH option, it was needed before on PS4 as a little of the data is different when connected by plugged in versus by BT. In particular the HAT buttons. Some of the current code with BT enabled, tries to deduce this. It assumes the PS4 is BT if the BT object is active otherwise it assumes wired.

Note: I have also cheated in the past and simply paired a PS4 with a BT dongle on my Linux boot… Could probably do as well with windows, but my windows machine has a BT built in…

With PS3 it is even easier… You can use the normal BT build… Then you need to plug in the BT dongle and make sure it starts up, and either remove it or if using hub, plug in the PS3 wired… Hopefully code will have remembered the BT address. Once the joystick has started up wired, I believe you hold down either the L1 or R1 buttons and the PS3 button and it will tell the PS3 to update itself for that BT object… After that simply unplug it, plug back in the BT dongle and press the PS3 button to connect up…
  
For list of known working dongles post #1 (https://forum.pjrc.com/threads/49358-T3-6-USB-Host-Bluetooth) in USBHost_t36 PJRC Forum thread.
  
To give you an idea of how the controller works with the hexapod the attached video is provided.  Forwarned it does use a differnt controller:
