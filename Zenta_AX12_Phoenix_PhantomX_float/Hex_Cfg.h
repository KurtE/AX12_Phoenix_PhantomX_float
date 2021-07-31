//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the 
//    Version 2 PhantomX robot.
//    Will first define to use their commander unit.
//
//Date: June 19, 2013
//Programmer: Kurt (aka KurtE)
//
//NEW IN V1.0
//   - First Release - Changes from Trossen member KingPin
//
//====================================================================
#ifndef HEX_CFG_H
#define HEX_CFG_H

#include "Controller_Cfg.h"
#include "Leg_Cfg.h"

#if defined __has_include
#  if __has_include ("Local_Hex_Cfg.h")
#    include "Local_Hex_Cfg.h"
#  endif
#endif

#endif // HEX_CFG_H
