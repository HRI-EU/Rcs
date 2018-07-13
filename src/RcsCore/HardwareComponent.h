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

#ifndef RCS_HARDWARE_COMPONENT_H
#define RCS_HARDWARE_COMPONENT_H


#include <Rcs_graph.h>

namespace Rcs
{
/*! \ingroup RcsMotionControlLayer
 *  \brief Base class for hardware components
 *
 *         This base class is a generic interface between hardware and control
 *         layers. Derived classes have to implement a updateGraph() method
 *         that reads out the sensory information and writes it into the graph.
 *
 *         The updateGraph() method doesn't need to compute the forward
 *         kinematics, that is done in the class MotionControlLayer that
 *         organizes a vector of HardwareComponents.
 *
 *         The setCommand() method gets three input arrays that are of
 *         the same dimensionality as the the graph. If a
 *         HardwareComponent is driving a robot, the corresponding
 *         entries of the q, qp and / or T vector are to be read out
 *         at the corresponding indices, and the values to be sent to
 *         the motors / joints. If a HardwareComponent does not drive
 *         hardware with actuators, this methods implementation is
 *         just empty.
 *
 *         Usually HardwareComponents have one or several threads that
 *         talk to some sensor / robot. If you want to make a
 *         HardwareComponent responsible to periodically call the
 *         MotionControlLayers updateControl() routine, this can be
 *         done with the registerCallback() function. The component
 *         then simply needs to call callControlLayerCallbackIfConnected()
 *         from the thread. A number of code example can be found in
 *         different derived classes (e.g. LwaComponent).
 */
class HardwareComponent
{
public:
  typedef void (*CallbackType)(void*);

  HardwareComponent();

  virtual ~HardwareComponent();

  /*! \brief Writes the compoents state into the overall graph state vector.
   *         This is called from the MotionControlLayer for each component.
   *         After that, the forward kinematics is computed.
   */
  virtual void updateGraph(RcsGraph* graph) = 0;

  /*! \brief Function that is called directly after computing the forward
   *         kinematics.
   */
  virtual void postUpdateGraph();

  /*! \brief Apply the given control inputs to the HardwareComponent. Each
   *         input array must be of size [RcsGraph::dof x 1]. The
   *         implementation must take care to take the control input from the
   *         correct array (e.g. a position-controlled component should take
   *         its input from q_des etc.)
   *
   *  \param[in] q_des      Vector of desired degrees of freedom.
   *  \param[in] q_dot_des  Vector of desired degrees of freedom velocities.
   *  \param[in] T_des      Vector of desired degrees of freedom torques.
   */
  virtual void setCommand(const MatNd* q_des, const MatNd* q_dot_des,
                          const MatNd* T_des) = 0;

  /*!  \brief Must return a pointer to a valid character array (terminated).
   *          It is assumed that this is not NULL.
   */
  virtual const char* getName() const = 0;

  /*!  \brief Return the time between two update steps of the component.
   *
   *   \return Update time in seconds.
   */
  virtual double getCallbackUpdatePeriod() const = 0;

  /*!  \brief Starts the component's thread with its specific update period.
   *
   *   \return True for success, false otherwise.
   */
  virtual bool startThread() = 0;

  /*!  \brief Stops the component's thread.
   *
   *   \return True for success, false otherwise.
   */
  virtual bool stopThread() = 0;

  /*! \brief Called periodically to check if the component is in a critical
   *         state and the robot should be stopped.
   *
   *  \return TRUE, if a critical condition is present, FALSE otherwise.
   */
  virtual bool checkEmergencyCondition();

  /*! \brief If any active component raises an emergency condition, this
   *         function will be called for all active HardwareComponents.
   */
  virtual void onEmergencyStop(const HardwareComponent* fault);

  /*! \brief If an emergency condition was encountered but has since been
   *          resolved, this function will be called to allow all other
   *          components to resume.
   *  \returns TRUE if the component has completed any post-emergency actions,
   *           FALSE if it still needs more time to safely recover.
   */
  virtual bool onEmergencyRecovery();

  virtual void registerCallback(CallbackType callback, void* param);

protected:

  virtual void callControlLayerCallbackIfConnected();
  CallbackType callbackFunction;
  void* callbackParam;
  bool isCallbackConnected;
};

}

#endif   // RCS_HARDWARE_COMPONENT_H
