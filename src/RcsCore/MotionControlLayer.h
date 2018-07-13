/******************************************************************************

  Copyright (C) by
  Honda Research Institute Europe GmbH,
  Carl-Legien Str. 30
  63073 Offenbach/Main
  Germany

  UNPUBLISHED PROPRIETARY MATERIAL.
  ALL RIGHTS RESERVED

  Author: Michael Gienger

******************************************************************************/

#ifndef RCS_MOTION_CONTROL_LAYER_H
#define RCS_MOTION_CONTROL_LAYER_H

#include "HardwareComponent.h"

#include <vector>

/*!
 *  \defgroup RcsMotionControlLayer Motion control components
 *
 *  A set of components that interface different hardware, along with a set
 *  of classes that define a generic interface to task-level control.
 */

namespace Rcs
{
/*! \ingroup RcsMotionControlLayer
 *  \brief Base class for task-level control components
 *
 *         This class can hold a vector of hardware components that interface
 *         all kinds of sensors and robots. It has a generic callback function
 *         that calls two functions:
 *         - updateGraph() traverses all of the classes HardwareComponents and
 *           calls their updateGraph() function. It's pretty generic and in most
 *           cases not needed to overwrite this function.
 *         - updateControl() is meant to compute a control input to the
 *           connected HardwareComponents, and to pass it through the routine
 *           setMotorCommand().
 *         The generic callback function is called by one of the
 *         HardwareComponents that has been made a "triggering" component (see
 *         setCallbackTriggerComponent()). This allows to assign the updating of
 *         states and controls to be synchronized with any HardwareComponent. If
 *         there is for instance a component with strict real-time updates, this
 *         classes callback can easily be synchronized with it.
 */
class MotionControlLayer
{
public:

  /*!  \brief Constructor with default arguments. This classes graph is assumed
   *          to not be owned, but to be a pointer from outside. It will not be
   *          destroyed in the destructor.
   */
  MotionControlLayer();

  /*!  \brief Constructor with xml file name containing the graph description.
   *          The graph will be constructed and destroyed in this class.
   *
   *   \param[in] xmlFile   Character array containing the xml file name with
   *                        the RcsGraph structure.
   */
  MotionControlLayer(const char* xmlFile);

  /*!  \brief The destructor disconnects all callbacks and destroys all
   *          HardwareComponents. If the graph is owned by this class, it will
   *          be destroyed.
   */
  virtual ~MotionControlLayer();

  /*!  \brief Assigns the job of calling this classes callback routine to the
   *          given HardwareComponent. The frequency is determined by the
   *          HardwareComponent.
   *
   *   \param[in] hwComponent   Hardware component that triggers this classes
   *                            callback function.
   */
  virtual bool addHardwareComponent(HardwareComponent* hwComponent);


  virtual bool setCallbackTriggerComponent(HardwareComponent* hwComponent);
  virtual HardwareComponent* getCallbackTriggerComponent();
  virtual const HardwareComponent* getCallbackTriggerComponent() const;
  virtual bool hasCallbackTriggerComponent() const;

  /*!  \brief If any of the classes Hardware components is triggering this
   *          classes callback function, it will be disconnected. This function
   *          must be called before anything of this or inherited classes is
   *          destroyed, since the triggering HardwareComponent is calling the
   *          updateGraph() and updateControl() routines. Please call it latest
   *          as the first call inside each derieved classes destructor.
   */
  virtual void disconnectCallback();


  /*!  \brief Returns a pointer to the HardwareComponent with the given name.
   *
   *   \return Pointer to the HardwareComponent with the given name or NULL if
   *           no HardwareComponent with the name exists.
   */
  virtual HardwareComponent* getComponent(const char* name);


  /*!  \brief Checks if a HardwareComponent with the given name exists.
   *
   *   \return True if the component has been found, false otherwise.
   */
  virtual bool hasComponent(const char* name) const;


  /*!  \brief Returns a const pointer to the HardwareComponent with the given
   *          name.
   *
   *   \return Const pointer to the HardwareComponent with the given name or
   *           NULL if no HardwareComponent with the name exists.
   */
  virtual const HardwareComponent* getComponent(const char* name) const;


  /*!  \brief Returns the update period of the classes callback (which calls
   *          updateGraph() and updateControl()). It is essentially the update
   *          period of the trigger component.
   *
   *   \return Update period in [sec]. If no component is triggering the
   *           callback, the function returns 0.
   */
  virtual double getCallbackUpdatePeriod() const;


  /*!  \brief Accessor function for the internal RcsGraph.
   *
   *   \return Pointer to the classes RcsGraph structure, or NULL if it doesn't
   *           exist (e.g. after calling the default constructor).
   */
  virtual RcsGraph* getGraph() const;
  virtual RcsGraph* getCurrentGraph() const;


  virtual void setMotorCommand(const MatNd* q_des, const MatNd* qp_des,
                               const MatNd* T_des);
  virtual bool getStepMode() const;
  virtual void setStepMode(bool pauseAfterStep);
  virtual void explicitCallback();

  virtual unsigned int getLoopCount() const;

  bool getEmergency() const;

  virtual void startThreads();
  virtual void stopThreads();

protected:
  virtual void updateControl() = 0;
  virtual void updateGraph();
  virtual bool checkEmergencyConditions();
  virtual void notifyEmergencyStop(const HardwareComponent* fault);
  virtual bool notifyEmergencyRecovery();
  virtual void setEmergencyStop();

  RcsGraph* desiredGraph;
  RcsGraph* currentGraph;
  std::vector<HardwareComponent*> componentVec;

private:
  HardwareComponent* callbackTriggerComponent;
  static void staticCallback(void* param);
  bool ownsDesiredGraph;
  bool stepMe;
  bool emergency;
  unsigned int loopCount, overruns;
  double dtMin, dtMax, dtDesired, sum;
};

}

#endif   // RCS_MOTION_CONTROL_LAYER_H
