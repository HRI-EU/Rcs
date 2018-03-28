/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
