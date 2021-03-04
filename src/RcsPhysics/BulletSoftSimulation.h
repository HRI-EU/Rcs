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

#ifndef RCS_BULLETSOFTSIMULATION_H
#define RCS_BULLETSOFTSIMULATION_H

#include "BulletSimulation.h"

#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>


namespace Rcs
{
/*! \ingroup RcsPhysics
 * \brief Construct with:
 *        BulletSoftSimulation* sim = new BulletSoftSimulation();
 *        sim->PhysicsBase::initialize(graph, cfg);
 *
 *        That's needed due to some polymorphism going on during intialisation.
 *        This is not considered in the constructors.
 */
class BulletSoftSimulation : public BulletSimulation
{
public:

  BulletSoftSimulation();
  BulletSoftSimulation(const BulletSoftSimulation& copyFromMe);
  BulletSoftSimulation(const BulletSoftSimulation& copyFromMe,
                       const RcsGraph* newGraph);
  virtual ~BulletSoftSimulation();
  virtual const char* getClassName() const;
  void transformVerticesToWorld();
  void transformVerticesToShape();
  bool initialize(const RcsGraph* g, const char* physicsConfigFile);

protected:

  bool initialize(const RcsGraph* g, const PhysicsConfig* config);
  void createWorld(xmlNodePtr bulletParams);
  void convertShapesToMesh();
  void updateSoftMeshes();
  void createSoftBodies();
  int connectSoftToRigidBody(btSoftBody* softBdy, BulletRigidBody* rigidBdy);
  btSoftBodyWorldInfo* softBodyWorldInfo;
  btSoftRigidDynamicsWorld* softWorld;
  bool transformVerticesToShapeFrame;
};

}   // namespace Rcs

#endif   // RCS_BULLETSOFTSIMULATION_H
