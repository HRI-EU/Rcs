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

#ifndef RCS_BULLETSOFTSIMULATION_H
#define RCS_BULLETSOFTSIMULATION_H

#include "BulletSimulation.h"

#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>


namespace Rcs
{
class BulletSoftSimulation : public BulletSimulation
{
public:

  BulletSoftSimulation();
  BulletSoftSimulation(const RcsGraph* graph, const char* configFile=NULL);
  BulletSoftSimulation(const RcsGraph* graph, const PhysicsConfig* config);
  BulletSoftSimulation(const BulletSoftSimulation& copyFromMe);
  BulletSoftSimulation(const BulletSoftSimulation& copyFromMe,
                       const RcsGraph* newGraph);
  virtual ~BulletSoftSimulation();
  virtual const char* getClassName() const;

protected:

  virtual bool initialize(const RcsGraph* g, const PhysicsConfig* config);
  virtual void createWorld(xmlNodePtr bulletParams);
  virtual void updateSoftMeshes();
  void createSoftBodies();
  btSoftBodyWorldInfo* softBodyWorldInfo;
};

}   // namespace Rcs

#endif   // RCS_BULLETSOFTSIMULATION_H
