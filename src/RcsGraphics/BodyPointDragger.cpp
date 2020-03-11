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

#include "BodyPointDragger.h"

#include <Rcs_typedef.h>
#include <Rcs_kinematics.h>
#include <Rcs_Vec3d.h>
#include <Rcs_VecNd.h>
#include <Rcs_macros.h>



/******************************************************************************
 *
 *****************************************************************************/
Rcs::BodyPointDragger::BodyPointDragger() : MouseDragger(),
  _forceScaleFactor(1.0)
{
  setName("BodyPointDragger");
}

/******************************************************************************
 * Compute the force: I_force = c*(mouseTip - anchor)
 * Without scaling, 1 N corresponds to a tip-anchor displacement of 1m.
 *****************************************************************************/
void Rcs::BodyPointDragger::getDragForce(double f[3]) const
{
  double I_mouseTip[3], I_anchor[3];
  bool LMBpressed, lCtrlPressed;
  Vec3d_setZero(f);

  const RcsBody* bdy = getDragData(I_mouseTip, I_anchor, NULL,
                                   &LMBpressed, NULL, NULL, &lCtrlPressed);

  if ((bdy!=NULL) && (LMBpressed==true))
  {
    Vec3d_sub(f, I_mouseTip, I_anchor);
    Vec3d_constMulSelf(f, getForceScaling());

    if (lCtrlPressed)
    {
      Vec3d_constMulSelf(f, 10.0);
    }
  }

}

/******************************************************************************
 *
 *****************************************************************************/
void Rcs::BodyPointDragger::scaleDragForce(double scaleFactor)
{
  _forceMtx.lock();
  _forceScaleFactor = scaleFactor;
  _forceMtx.unlock();
}

/******************************************************************************
 *
 *****************************************************************************/
double Rcs::BodyPointDragger::getForceScaling() const
{
  double sf;
  _forceMtx.lock();
  sf = _forceScaleFactor;
  _forceMtx.unlock();

  return sf;
}

/******************************************************************************
 *
 *****************************************************************************/
void Rcs::BodyPointDragger::addJointTorque(MatNd* draggerTorque,
                                           const RcsGraph* graph) const
{
  MatNd* tmp = MatNd_create(graph->nJ, 1);
  getJointTorque(tmp, graph);
  RCHECK(draggerTorque->m*draggerTorque->n==graph->nJ);
  VecNd_addSelf(draggerTorque->ele, tmp->ele, graph->nJ);
  MatNd_destroy(tmp);
}

/******************************************************************************
 *
 *****************************************************************************/
void Rcs::BodyPointDragger::subJointTorque(MatNd* draggerTorque,
                                           const RcsGraph* graph) const
{
  MatNd* tmp = MatNd_create(graph->nJ, 1);
  getJointTorque(tmp, graph);
  RCHECK(draggerTorque->m*draggerTorque->n==graph->nJ);
  VecNd_subSelf(draggerTorque->ele, tmp->ele, graph->nJ);
  MatNd_destroy(tmp);
}

/******************************************************************************
 *
 *****************************************************************************/
void Rcs::BodyPointDragger::getJointTorque(MatNd* draggerTorque,
                                           const RcsGraph* graph) const
{
  MatNd_reshapeAndSetZero(draggerTorque, graph->nJ, 1);

  double I_mouseTip[3], I_anchor[3], k_anchor[3];
  bool leftCtrlPressed;
  const RcsBody* bdy = getDragData(I_mouseTip, I_anchor, k_anchor,
                                   NULL, NULL, NULL, &leftCtrlPressed);

  // Map external force to joints: M = J^T F
  if (bdy == NULL)
  {
    return;
  }

  double f[3];
  Vec3d_sub(f, I_mouseTip, I_anchor);
  Vec3d_constMulSelf(f, getForceScaling()*(leftCtrlPressed ? 10.0 : 1.0));

  // Compute transpose Jacobian of force contact point
  MatNd* JT = MatNd_create(3, graph->nJ);
  RcsGraph_bodyPointJacobian(graph, bdy, k_anchor, NULL, JT);
  MatNd_transposeSelf(JT);

  // Project drag force on joints
  MatNd* dH = MatNd_create(graph->nJ, 1);
  MatNd Fx = MatNd_fromPtr(3, 1, f);
  MatNd_mul(dH, JT, &Fx);
  MatNd_subSelf(draggerTorque, dH);

  MatNd_destroy(JT);
  MatNd_destroy(dH);
}
