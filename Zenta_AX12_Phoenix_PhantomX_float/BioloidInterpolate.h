/*
  BioloidController.h - ArbotiX Library for Bioloid Pose Engine
  Copyright (c) 2008-2012 Michael E. Ferguson.  All right reserved.

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

// This is a hacked up version to work with Dynamixel2Arduino and only
// the interpolation stuff... 

#ifndef BioloidInterpolate_h
#define BioloidInterpolate_h
#include <Arduino.h>
#include "Hex_Cfg.h"
#include <Dynamixel2Arduino.h>
/* pose engine runs at 30Hz (33ms between frames) 
   recommended values for interpolateSetup are of the form X*BIOLOID_FRAME_LENGTH - 1 */
#define BIOLOID_FRAME_LENGTH      20
/* we need some extra resolution, use 13 bits, rather than 10, during interpolation */
#define BIOLOID_SHIFT             3

/** a structure to hold transitions **/
typedef struct{
    unsigned int * pose;    // addr of pose to transition to 
    int time;               // time for transition
} transition_t; 

#ifndef DXL_SERVO_COUNT
#define DXL_SERVO_COUNT 18
#endif


/** Bioloid Controller Class for mega324p/644p clients. **/
class BioloidInterpolate
{
  public:
    /* For compatibility with legacy code */
    // Changed to two step init...
    BioloidInterpolate(Dynamixel2Arduino &dxl, uint16_t reg_start_addr) : dxl_(dxl), reg_start_addr_(reg_start_addr) {};               // baud usually 1000000
    void begin();

    /* Pose Manipulation */
    void readPose();                            // read a pose in from the servos  
    void writePose();                           // write a pose out to the servos
    void setNextPose(int id, int pos);          // set a servo value in the next pose
    void setNextPoseByIndex(int index, int pos);  // set a servo value by index for next pose
    void setId(int index, int id);              // set the id of a particular storage index
    int getId(int index);                       // get the id of a particular storage index
    
    /* Pose Engine */
    void interpolateSetup(int time);            // calculate speeds for smooth transition
    int interpolateStep(boolean fWait=true);                     // move forward one step in current interpolation  
    unsigned char interpolating;                // are we in an interpolation? 0=No, 1=Yes
    unsigned char runningSeq;                   // are we running a sequence? 0=No, 1=Yes 
    int poseSize = DXL_SERVO_COUNT;                               // how many servos are in this pose, used by Sync()

    // Kurt's Hacks
    uint8_t frameLength;                        // Allow variable frame lengths, to test...

  private:  
    uint16_t pose_[DXL_SERVO_COUNT];                       // the current pose, updated by Step(), set out by Sync()
    uint16_t nextpose_[DXL_SERVO_COUNT];                   // the destination pose, where we put on load
    int16_t  speed_[DXL_SERVO_COUNT];                               // speeds for interpolation 
    //uint8_t  id_[POSE_SIZE];                        // servo id for this index

    // BUGBUG can probably combine with above:
    typedef struct {
    #if DYNAMIXEL_PROTOCOL == 2
      uint32_t goal_position;
    #else
      uint16_t goal_position;
    #endif
    } __attribute__((packed)) sw_data_t;

    sw_data_t sw_data_[DXL_SERVO_COUNT];
    DYNAMIXEL::InfoSyncWriteInst_t sw_infos_;
    DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_[DXL_SERVO_COUNT];


//    unsigned long lastframe_;                   // time last frame was sent out  
    unsigned long nextframe_;                   //    
    transition_t * sequence;                    // sequence we are running
    int transitions;                            // how many transitions we have left to load
    Dynamixel2Arduino &dxl_;                    // Dynamixel object   
    uint16_t  reg_start_addr_;   
};

#endif
