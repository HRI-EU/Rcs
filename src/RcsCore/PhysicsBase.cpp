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

#include "PhysicsBase.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"


/*******************************************************************************
 * Constructor.
 ******************************************************************************/
Rcs::PhysicsBase::PhysicsBase(const RcsGraph* graph_) :
  T_des(NULL),
  q_des(NULL),
  q_dot_des(NULL),
  simTime(0.0),
  enablePPS(false),
  internalDesiredGraph(NULL)
{
  RCHECK(graph_);

  // create arrays for joint positions, velocities and torque
  this->T_des = MatNd_create(graph_->dof, 1);
  this->q_des = MatNd_clone(graph_->q);
  this->q_dot_des = MatNd_create(graph_->dof, 1);

  this->internalDesiredGraph = RcsGraph_clone(graph_);
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
Rcs::PhysicsBase::PhysicsBase(const PhysicsBase& copyFromMe) :
  T_des(NULL),
  q_des(NULL),
  q_dot_des(NULL),
  simTime(copyFromMe.simTime),
  enablePPS(false),
  internalDesiredGraph(NULL)
{
  // Create arrays for joint positions, velocities and torques
  this->T_des = MatNd_clone(copyFromMe.T_des);
  this->q_des = MatNd_clone(copyFromMe.q_des);
  this->q_dot_des = MatNd_clone(copyFromMe.q_dot_des);

  this->internalDesiredGraph = RcsGraph_clone(copyFromMe.getGraph());
}

/*******************************************************************************
 * Copy constructor.
 ******************************************************************************/
Rcs::PhysicsBase::PhysicsBase(const PhysicsBase& copyFromMe,
                              const RcsGraph* newGraph) :
  T_des(NULL),
  q_des(NULL),
  q_dot_des(NULL),
  simTime(copyFromMe.simTime),
  enablePPS(false),
  internalDesiredGraph(NULL)
{
  // Create arrays for joint positions, velocities and torques
  this->T_des = MatNd_clone(copyFromMe.T_des);
  this->q_des = MatNd_clone(copyFromMe.q_des);
  this->q_dot_des = MatNd_clone(copyFromMe.q_dot_des);

  this->internalDesiredGraph = RcsGraph_clone(newGraph);
}

/*******************************************************************************
 * Assignment operator.
 ******************************************************************************/
Rcs::PhysicsBase& Rcs::PhysicsBase::operator= (const Rcs::PhysicsBase& copyFromMe)
{
  // check for self-assignment by comparing the address of the
  // implicit object and the parameter
  if (this == &copyFromMe)
  {
    return *this;
  }

  MatNd_resizeCopy(&this->T_des, copyFromMe.T_des);
  MatNd_resizeCopy(&this->q_des, copyFromMe.q_des);
  MatNd_resizeCopy(&this->q_dot_des, copyFromMe.q_dot_des);

  this->simTime = copyFromMe.time();
  this->enablePPS = copyFromMe.getEnablePPS();

  RcsGraph_destroy(this->internalDesiredGraph);
  this->internalDesiredGraph = RcsGraph_clone(copyFromMe.getGraph());

  return *this;
}

/*******************************************************************************
 * Destructor.
 ******************************************************************************/
Rcs::PhysicsBase::~PhysicsBase()
{
  MatNd_destroy(this->T_des);
  MatNd_destroy(this->q_des);
  MatNd_destroy(this->q_dot_des);

  RcsGraph_destroy(this->internalDesiredGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PhysicsBase::simulate(double dt, RcsGraph* graph, MatNd* q_ddot,
                                MatNd* T, bool control)
{
  simulate(dt, graph->q, graph->q_dot, q_ddot, T, control);

  // Copy sensors
  RcsSensor* dstSensorPtr = graph->sensor;
  const RcsSensor* srcSensorPtr = internalDesiredGraph->sensor;

  while (dstSensorPtr != NULL)
  {
    MatNd_copy(dstSensorPtr->rawData, srcSensorPtr->rawData);
    dstSensorPtr = dstSensorPtr->next;
    srcSensorPtr = srcSensorPtr->next;
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::disableCollisionsWithinGroup(const char* suffix)
{
  // if suffix == NULL, disable collisions within all groups
  RCSGRAPH_TRAVERSE_BODIES(this->internalDesiredGraph)
  {
    //if ((suffix && STREQ(suffix, BODY->suffix)) || (!suffix))
    if ((suffix && STREQ(suffix, BODY->suffix)))
    {
      RcsBody* b1 = BODY;
      RCSGRAPH_TRAVERSE_BODIES(this->internalDesiredGraph)
      {
        if (STREQ(b1->suffix, BODY->suffix) && BODY != b1)
        {
          disableCollision(b1, BODY);
        }
      }
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::disableMeshCollisions()
{
  RCSGRAPH_TRAVERSE_BODIES(this->internalDesiredGraph)
  {
    RcsBody* b1 = BODY;

    RCSBODY_TRAVERSE_SHAPES(b1)
    {
      if (SHAPE->type == RCSSHAPE_MESH)
      {
        RCSGRAPH_TRAVERSE_BODIES(this->internalDesiredGraph)
        {
          RCSBODY_TRAVERSE_SHAPES(BODY)
          {
            if (SHAPE->type == RCSSHAPE_MESH)
            {
              if (b1 != BODY)
              {
                disableCollision(b1, BODY);
              }
            }
          }
        }
      }
    }
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::disableCollisions()
{
  RCSGRAPH_TRAVERSE_BODIES(this->internalDesiredGraph)
  {
    RcsBody* b1 = BODY;

    RCSGRAPH_TRAVERSE_BODIES(this->internalDesiredGraph)
    {
      if (b1 != BODY)
      {
        disableCollision(b1, BODY);
      }
    }
  }

}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::disableJointLimits()
{
  setJointLimits(false);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::enableJointLimits()
{
  setJointLimits(true);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
const RcsGraph* Rcs::PhysicsBase::getGraph() const
{
  return this->internalDesiredGraph;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
RcsGraph* Rcs::PhysicsBase::getGraph()
{
  return this->internalDesiredGraph;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
double Rcs::PhysicsBase::time() const
{
  return this->simTime;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::resetTime()
{
  this->simTime = 0.0;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::setTime(double t)
{
  this->simTime = t;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::incrementTime(double dt)
{
  this->simTime += dt;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::setControlInput(const MatNd* q_des_,
                                       const MatNd* q_dot_des_,
                                       const MatNd* T_des_)
{
  // Desired joint angles
  if (q_des_ != NULL)
  {
    RCHECK_EQ(q_des_->n, 1);

    if (q_des_->m==this->internalDesiredGraph->dof)
    {
      MatNd_copy(this->q_des, q_des_);
    }
    else if (q_des_->m==this->internalDesiredGraph->nJ)
    {
      RcsGraph_stateVectorFromIK(this->internalDesiredGraph, q_des_, this->q_des);
    }
    else
    {
      RFATAL("Dimension mismatch: q_des_->m=%d, but dof=%d and nJ=%d",
             q_des_->m, this->internalDesiredGraph->dof, this->internalDesiredGraph->nJ);
    }

    // Forward kinematics based on the desired state. This is needed in order
    // to set the transformations of kinematically controlled bodies.
    RcsGraph_setState(internalDesiredGraph, this->q_des, NULL);
  }

  // Desired joint velocities
  if (q_dot_des_ != NULL)
  {
    RCHECK_EQ(q_dot_des_->n, 1);

    if (q_dot_des_->m==this->internalDesiredGraph->dof)
    {
      MatNd_copy(this->q_dot_des, q_dot_des_);
    }
    else if (q_dot_des_->m==this->internalDesiredGraph->nJ)
    {
      RcsGraph_stateVectorFromIK(this->internalDesiredGraph, q_dot_des_, this->q_dot_des);
    }
    else
    {
      RFATAL("Dimension mismatch: q_dot_des_->m=%d, but dof=%d and nJ=%d",
             q_dot_des_->m, this->internalDesiredGraph->dof, this->internalDesiredGraph->nJ);
    }
  }

  // Desired joint torques
  if (T_des_ != NULL)
  {
    RCHECK_EQ(T_des_->n, 1);

    if (T_des_->m==this->internalDesiredGraph->dof)
    {
      MatNd_copy(this->T_des, T_des_);
    }
    else if (T_des_->m==this->internalDesiredGraph->nJ)
    {
      RcsGraph_stateVectorFromIK(this->internalDesiredGraph, T_des_, this->T_des);
    }
    else
    {
      RFATAL("Dimension mismatch: T_des_->m=%d, but dof=%d and nJ=%d",
             T_des_->m, this->internalDesiredGraph->dof, this->internalDesiredGraph->nJ);
    }
  }
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::getLastPositionCommand(MatNd* q_des_) const
{
  MatNd_reshapeCopy(q_des_, this->q_des);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::PhysicsBase::print() const
{
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::PhysicsBase::setParameter(ParameterCategory category,
                                    const char* name, const char* type,
                                    double value)
{
  RLOG(0, "Implement or overwrite me");
  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::PhysicsBase::setEnablePPS(bool enable)
{
  this->enablePPS = enable;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::PhysicsBase::getEnablePPS() const
{
  return this->enablePPS;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::PhysicsBase::removeBody(const char* name)
{
  RLOG(0, "Implement or overwrite me");
  return false;
}

/*******************************************************************************
*
******************************************************************************/
bool Rcs::PhysicsBase::addBody(const RcsBody* body)
{
  RLOG(0, "Implement or overwrite me");
  return false;
}

/*******************************************************************************
*
******************************************************************************/
bool Rcs::PhysicsBase::deactivateBody(const char* name)
{
  RLOG(0, "Implement or overwrite me");
  return false;
}

/*******************************************************************************
*
******************************************************************************/
bool Rcs::PhysicsBase::activateBody(const char* name, const HTr* A_BI)
{
  RLOG(0, "Implement or overwrite me");
  return false;
}

/*******************************************************************************
*
******************************************************************************/
bool Rcs::PhysicsBase::check() const
{
  return true;
}
