/*******************************************************************************

  Copyright (C) by
  Honda Research Institute Europe GmbH,
  Carl-Legien Str. 30
  63073 Offenbach/Main
  Germany

  UNPUBLISHED PROPRIETARY MATERIAL.
  ALL RIGHTS RESERVED

  Authors: Michael Gienger

*******************************************************************************/

#include "HardwareComponent.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>



/*******************************************************************************
 * Hardware component base class
 ******************************************************************************/
Rcs::HardwareComponent::HardwareComponent(RcsGraph* graph_):
  callbackFunction(NULL),
  callbackParam(NULL),
  graph(graph_),
  isCallbackConnected(false)
{
}

/*******************************************************************************
 * Virtual destructor to allow overlaoding
 ******************************************************************************/
Rcs::HardwareComponent::~HardwareComponent()
{
}

/*******************************************************************************
 * Each component can maintain a callback function, which is triggered by
 * the component and calls the MotionControlLayer's callback() function.
 * It is not mandatory to do this. For one MotionControlLayer, only one
 * component should be the one having a callback function.
 ******************************************************************************/
void Rcs::HardwareComponent::registerCallback(CallbackType callback,
                                              void* param)
{
  this->callbackFunction = callback;
  this->callbackParam = param;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::HardwareComponent::callControlLayerCallbackIfConnected()
{
  if (this->callbackFunction != NULL)
  {
    if (this->isCallbackConnected==false)
    {
      RLOG(5, "Callback connected");
      this->isCallbackConnected=true;
    }
    callbackFunction(callbackParam);
  }
  else
  {
    if (this->isCallbackConnected==true)
    {
      RLOG(5, "Callback disconnected");
      this->isCallbackConnected=false;
    }
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::HardwareComponent::onEmergencyStop(const HardwareComponent* fault)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::HardwareComponent::onEmergencyRecovery()
{
  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::HardwareComponent::checkEmergencyCondition()
{
  return false;
}
