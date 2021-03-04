/*******************************************************************************

  Copyright (c) 2017, Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_BODYPOINTDRAGGER_H
#define RCS_BODYPOINTDRAGGER_H

#include "MouseDragger.h"


namespace Rcs
{

/*!
* \ingroup BodyPointDraggerFunctions
*/
class BodyPointDragger: public MouseDragger
{
public:

  /*! \brief Default constructor.
   */
  BodyPointDragger();

  /*! \brief Returns the drag force. It points from the picked body anchor
   *         to the mouse tip and lies in the plane perpendicular to the
   *         camera view direction. A distance of 1m corresponds to 1N.
   */
  virtual void getDragForce(double F[3]) const;

  /*! \brief Scales the drag force by the given factor.
   */
  virtual void scaleDragForce(double scaleFactor);

  /*! \brief Returns the force scaling factor.
   */
  virtual double getForceScaling() const;

  /*! \brief Computes the static joint torque according to the drag force.
   *
   *  \param[out] draggerTorque   Static joint torque, reshaped to
   *                              RcsGraph::nJ x 1 dimensions. If no body
   *                              is dragged, the result is set to zero.
   *
   *  \param[in] graph            Graph structure to compute the required
   *                              Jacobian. It is not modified.
   */
  virtual void getJointTorque(MatNd* draggerTorque, const RcsGraph* graph) const;

  /*! \brief Computes the static joint torque according to the drag
   *         force and adds it to argument draggerTorque. The function checks
   *         for the correct size of the draggerTorque, but does not care
   *         about its shape (e.g. transposed or not). In case of a size
   *         mismatch, the function exits fatally.
   */
  virtual void addJointTorque(MatNd* draggerTorque, const RcsGraph* graph) const;

  /*! \brief Computes the static joint torque according to the drag force
   *         and substracts it from argument draggerTorque. The function
   *         checksfor the correct size of the draggerTorque, but does not
   *         care about its shape (e.g. transposed or not). In case of a size
   *         mismatch, the function exits fatally.
   */
  virtual void subJointTorque(MatNd* draggerTorque, const RcsGraph* graph) const;


private:

  double _forceScaleFactor;
  mutable OpenThreads::Mutex _forceMtx;
};

}   // namespace Rcs

#endif // RCS_BODYPOINTDRAGGER_H
