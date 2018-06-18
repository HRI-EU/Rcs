/*******************************************************************************

  Copyright (C) by
  Honda Research Institute Europe GmbH,
  Carl-Legien Str. 30
  63073 Offenbach/Main
  Germany

  UNPUBLISHED PROPRIETARY MATERIAL.
  ALL RIGHTS RESERVED

  Author: Michael Gienger

*******************************************************************************/

#include "MotionControlLayer.h"

#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::MotionControlLayer::MotionControlLayer():
  desiredGraph(NULL),
  currentGraph(NULL),
  callbackTriggerComponent(NULL),
  ownsDesiredGraph(false),
  stepMe(false),
  emergency(false),
  loopCount(0), overruns(0),
  dtMin(1.0), dtMax(0.0), dtDesired(0.0), sum(0.0)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::MotionControlLayer::MotionControlLayer(const char* xmlFile):
  desiredGraph(NULL),
  currentGraph(NULL),
  callbackTriggerComponent(NULL),
  ownsDesiredGraph(true),
  stepMe(false),
  emergency(false),
  loopCount(0), overruns(0),
  dtMin(1.0), dtMax(0.0), dtDesired(0.0), sum(0.0)
{
  this->desiredGraph = RcsGraph_create(xmlFile);
  RCHECK(this->desiredGraph);
  this->currentGraph = RcsGraph_clone(desiredGraph);
  RCHECK(this->currentGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::MotionControlLayer::~MotionControlLayer()
{
  // We remember the trigger component before disabling the callback for the
  // output.
  HardwareComponent* trigger = getCallbackTriggerComponent();

  // This class is deleted after the classes it inherits from. Therefore
  // the callback() function is invalid, while the trigger components thread
  // potentially accesses it. Therefore we need to stop the thread calls
  // to callback() in the parent destructor.
  if (this->callbackTriggerComponent!=NULL)
  {
    RWARNING(0, "Please make sure you disconnected the callback in the "
             "destructor of your class!");
    disconnectCallback();
  }

  // Delete all hardware components
  for (size_t i=0; i<componentVec.size(); ++i)
  {
    delete this->componentVec[i];
  }

  // Delete graph if it is owned by this class
  if (this->ownsDesiredGraph == true)
  {
    RcsGraph_destroy(this->desiredGraph);
  }

  RcsGraph_destroy(this->currentGraph);

  printf("[%s(%d)]: MotionControlLayer callback statistics:\n",
         __FUNCTION__, __LINE__);

  double mean = 0.0;

  if (getLoopCount()>0)
  {
    mean = (double) sum / getLoopCount();
  }

  if (trigger != NULL)
  {
    printf("  trigger component: %s\n", trigger->getName());
  }
  else
  {
    printf("  no trigger component\n");
  }

  printf("  target period = %7.3f msec\n", 1.0e3*dtDesired);
  printf("  min           = %7.3f msec\n", 1.0e3*dtMin);
  printf("  ave           = %7.3f msec\n", 1.0e3*mean);
  printf("  max           = %7.3f msec\n", 1.0e3*dtMax);
  printf("  total cycles  = %u\n", getLoopCount());
  printf("  overruns      = %u\n", overruns);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::MotionControlLayer::addHardwareComponent(HardwareComponent* component)
{
  if (component == NULL)
  {
    RLOG(1, "Component is NULL - not adding");
    return false;
  }

  for (size_t i=0; i<componentVec.size(); ++i)
  {
    if (component == this->componentVec[i])
    {
      RLOG(1, "Component \"%s\" has already been added - skipping",
           component->getName());
      return false;
    }
  }

  this->componentVec.push_back(component);

  // That's the earliest point in time where we can update the graph with the
  // component's real sensor values. We also update the desired graph with the
  // real sensor values so that it is consistent.
  component->updateGraph(this->desiredGraph);
  MotionControlLayer::updateGraph();

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MotionControlLayer::updateGraph()
{
  for (size_t i=0; i<componentVec.size(); ++i)
  {
    componentVec[i]->updateGraph(this->currentGraph);
  }

  RcsGraph_setState(this->desiredGraph, NULL, NULL);
  RcsGraph_setState(this->currentGraph, NULL, NULL);

  for (size_t i=0; i<componentVec.size(); ++i)
  {
    componentVec[i]->postUpdateGraph();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::MotionControlLayer::checkEmergencyConditions()
{
  for (size_t i=0; i<componentVec.size(); ++i)
  {
    if (componentVec[i]->checkEmergencyCondition())
    {
      // if any component reports an emergency condition, notify all components
      // that we need to stop.
      if (!this->emergency)
      {
        RLOG(0, "\n\n\t EMERGENCY STOP triggered by: %s",
             componentVec[i]->getName());
        notifyEmergencyStop(componentVec[i]);
      }
      this->emergency = true;
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MotionControlLayer::notifyEmergencyStop(const HardwareComponent* fault)
{
  for (size_t i=0; i<componentVec.size(); ++i)
  {
    componentVec[i]->onEmergencyStop(fault);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::MotionControlLayer::notifyEmergencyRecovery()
{
  this->emergency = false;

  for (size_t i=0; i<componentVec.size(); ++i)
  {
    this->emergency = this->emergency || !componentVec[i]->onEmergencyRecovery();
  }

  return !this->emergency;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MotionControlLayer::setEmergencyStop()
{
  if (this->emergency==false)
  {
    notifyEmergencyStop(NULL);
    this->emergency = true;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::MotionControlLayer::setCallbackTriggerComponent(HardwareComponent* component)
{
  if (component == this->callbackTriggerComponent)
  {
    RLOG(1, "Component %s is already trigger component",
         component ? component->getName() : "NULL");
    return false;
  }

  // In this case, we remove the old callback
  if (component == NULL)   // TriggerComponent is not NULL here, see above
  {
    this->callbackTriggerComponent->registerCallback(NULL, NULL);
    this->callbackTriggerComponent = NULL;
  }
  else if (component != NULL)
  {
    // Remove the old callback before adding the new one
    if (this->callbackTriggerComponent != NULL)
    {
      this->callbackTriggerComponent->registerCallback(NULL, NULL);
    }

    component->registerCallback(&staticCallback, this);
    this->callbackTriggerComponent = component;
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::HardwareComponent* Rcs::MotionControlLayer::getCallbackTriggerComponent()
{
  return this->callbackTriggerComponent;
}

/*******************************************************************************
 *
 ******************************************************************************/
const Rcs::HardwareComponent* Rcs::MotionControlLayer::getCallbackTriggerComponent() const
{
  return this->callbackTriggerComponent;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::MotionControlLayer::hasCallbackTriggerComponent() const
{
  return this->callbackTriggerComponent ? true : false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MotionControlLayer::disconnectCallback()
{
  // Here we determine the update period of the trigger component. At this
  // point, it is likely running and returning the correct value. If the
  // trigger component is stopped before disconnecting it, we might get a
  // value of zero. However, it's only for the statistics and does not have
  // any other side effects.
  if (callbackTriggerComponent != NULL)
  {
    this->dtDesired = callbackTriggerComponent->getCallbackUpdatePeriod();
  }

  this->callbackTriggerComponent = NULL;

  for (size_t i=0; i<componentVec.size(); ++i)
  {
    componentVec[i]->registerCallback(NULL, NULL);
  }

  // It may happen that the callback function is currently executed. Since it
  // usually takes only a few msec, we wait a little bit until it is surely
  // completed before continuing.
  Timer_waitDT(0.1);
}

/*******************************************************************************
 * This gets called after "setCallbackTriggerComponent()", and is only
 * stopped upon destruction of this class.
 ******************************************************************************/
void Rcs::MotionControlLayer::staticCallback(void* param)
{
  if (param==NULL)
  {
    return;
  }
  double dtCallback = Timer_getSystemTime();

  MotionControlLayer* bot = static_cast<MotionControlLayer*>(param);
  bot->updateGraph();

  if (!bot->checkEmergencyConditions())
  {
    // If we were in an emergency situation, notify components that it has
    // ended. This will allow them to perform any necessary recovery actions
    // before we resume control.
    if (bot->getEmergency())
    {
      bot->notifyEmergencyRecovery();
    }
    // if there is no emergency & all components are OK, send commands to the
    // robot
    else if (!bot->getEmergency())
    {
      bot->updateControl();
    }
  }

  bot->loopCount++;

  dtCallback = Timer_getSystemTime() - dtCallback;
  bot->sum += dtCallback;

  if (dtCallback < bot->dtMin)
  {
    bot->dtMin = dtCallback;
  }

  if (dtCallback > bot->dtMax)
  {
    bot->dtMax = dtCallback;
  }

  if (dtCallback > bot->getCallbackUpdatePeriod())
  {
    bot->overruns++;
  }

  if (bot->getStepMode() == true)
  {
    RPAUSE_MSG("Hit enter for next MotionControlLayer::staticCallback()");
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MotionControlLayer::explicitCallback()
{
  staticCallback((void*) this);
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int Rcs::MotionControlLayer::getLoopCount() const
{
  return this->loopCount;
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::HardwareComponent* Rcs::MotionControlLayer::getComponent(const char* name)
{
  for (size_t i=0; i<componentVec.size(); ++i)
  {
    if (STREQ(componentVec[i]->getName(), name))
    {
      return componentVec[i];
    }
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
const Rcs::HardwareComponent* Rcs::MotionControlLayer::getComponent(const char* name) const
{
  for (size_t i=0; i<componentVec.size(); ++i)
  {
    if (STREQ(componentVec[i]->getName(), name))
    {
      return componentVec[i];
    }
  }

  return NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::MotionControlLayer::hasComponent(const char* name) const
{
  if (getComponent(name) != NULL)
  {
    return true;
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
double Rcs::MotionControlLayer::getCallbackUpdatePeriod() const
{
  if (this->callbackTriggerComponent != NULL)
  {
    return callbackTriggerComponent->getCallbackUpdatePeriod();
  }
  else
  {
    return 0.0;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsGraph* Rcs::MotionControlLayer::getGraph() const
{
  return this->desiredGraph;
}

/*******************************************************************************
 *
 ******************************************************************************/
RcsGraph* Rcs::MotionControlLayer::getCurrentGraph() const
{
  return this->currentGraph;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MotionControlLayer::setMotorCommand(const MatNd* q_des,
                                              const MatNd* qDot_des,
                                              const MatNd* T_des)
{
  for (size_t i=0; i<componentVec.size(); ++i)
  {
    componentVec[i]->setCommand(q_des, qDot_des, T_des);
  }

  MatNd_copy(this->desiredGraph->q, q_des);

  if (qDot_des != NULL)
  {
    MatNd_copy(this->desiredGraph->q_dot, qDot_des);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::MotionControlLayer::getStepMode() const
{
  return this->stepMe;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::MotionControlLayer::setStepMode(bool pauseAfterStep)
{
  this->stepMe = pauseAfterStep;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::MotionControlLayer::getEmergency() const
{
  return this->emergency;
}
