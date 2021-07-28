/*
  BioloidController.cpp - ArbotiX Library for Bioloid Pose Engine
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

#include "BioloidInterpolate.h"

#include <avr/pgmspace.h>


void BioloidInterpolate::begin(){
    int i;
    // setup storage
    sw_infos_.packet.p_buf = nullptr;
    sw_infos_.packet.is_completed = false;

    sw_infos_.addr = reg_start_addr_;
    sw_infos_.addr_length = sizeof(sw_data_t);
    sw_infos_.p_xels = info_xels_sw_;
    sw_infos_.xel_count = poseSize;

    // initialize
    for(i=0;i<poseSize;i++){
        //id_[i] = i+1;
        pose_[i] = 512;
        nextpose_[i] = 512;
        info_xels_sw_[i].id = i+1;
        info_xels_sw_[i].p_data = (uint8_t*)&sw_data_[i].goal_position;
    }
    frameLength = BIOLOID_FRAME_LENGTH;
    interpolating = 0;
    nextframe_ = millis();
}



void BioloidInterpolate::setId(int index, int id){
    info_xels_sw_[index].id = id;
    //id_[index] = id;
}

int BioloidInterpolate::getId(int index){
    //return id_[index];
    return info_xels_sw_[index].id;
}

/* read in current servo positions to the pose. */

void BioloidInterpolate::readPose(){
    for(int i=0;i<poseSize;i++){
        // right now using the raw.
        pose_[i] = static_cast<int>(dxl_.getPresentPosition(info_xels_sw_[i].id))<<BIOLOID_SHIFT;
        delay(10);   
    }
}
/* write pose out to servos using sync write. */

void BioloidInterpolate::writePose(){
    // simply copy data in and call off to library
    for(int i=0; i<poseSize; i++)
    {
        sw_data_[i].goal_position = pose_[i] >> BIOLOID_SHIFT;
    } 
    sw_infos_.is_info_changed = true;
    if (!dxl_.syncWrite(&sw_infos_)) {
#ifdef DBGSerial
      DBGSerial.print("*** syncwrite failed: ");
      DBGSerial.print("R:"); DBGSerial.print(sw_infos_.addr, DEC);
      DBGSerial.print(" "); DBGSerial.print(sw_infos_.addr_length, DEC);
      DBGSerial.print(" Cnt:"); DBGSerial.print(sw_infos_.xel_count, DEC);
      DBGSerial.print(" changed:"); DBGSerial.println(sw_infos_.is_info_changed, DEC);
#endif

    }
}

/* set up for an interpolation from pose to nextpose over TIME 
    milliseconds by setting servo speeds. */

void BioloidInterpolate::interpolateSetup(int time){
    int i;
  int frames = (time/frameLength) + 1;
  nextframe_ = millis() + frameLength;
    // set speed each servo...
    for(i=0;i<poseSize;i++){
        if(nextpose_[i] > pose_[i]){
            speed_[i] = (nextpose_[i] - pose_[i])/frames + 1;
    }
    else{
            speed_[i] = (pose_[i]-nextpose_[i])/frames + 1;
        }
    }
    interpolating = 1;
}
/* interpolate our pose, this should be called at about 30Hz. */
#define WAIT_SLOP_FACTOR 10  

int BioloidInterpolate::interpolateStep(boolean fWait){
  if(interpolating == 0) return 0x7fff;
    int i;int complete = poseSize;
  if (!fWait) {
    if (millis() < (nextframe_ - WAIT_SLOP_FACTOR)) {
      return (millis() - nextframe_);    // We still have some time to do something... 
    }
  }
  while(millis() < nextframe_) ;
  nextframe_ = millis() + frameLength;
    // update each servo
    for(i=0;i<poseSize;i++){
        int diff = nextpose_[i] - pose_[i];
        if(diff == 0){
            complete--;
    }
    else{
            if(diff > 0){
                if(diff < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
        }
        else
                    pose_[i] += speed_[i];
      }
      else{
                if((-diff) < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
        }
        else
                    pose_[i] -= speed_[i];                
            }       
        }
    }
    if(complete <= 0) interpolating = 0;
    writePose();      
  return 0;  
}

/* set a servo value in the next pose */

void BioloidInterpolate::setNextPose(int id, int pos){
    for(int i=0; i<poseSize; i++){
        if( info_xels_sw_[i].id == id ){
            nextpose_[i] = (pos << BIOLOID_SHIFT);
            return;
        }
    }
}

/* Added by Kurt */

void BioloidInterpolate::setNextPoseByIndex(int index, int pos) {  // set a servo value by index for next pose
  if (index < poseSize) {
    nextpose_[index] = (pos << BIOLOID_SHIFT);
  }
}
