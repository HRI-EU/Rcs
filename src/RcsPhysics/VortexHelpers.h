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

#ifndef RCS_VORTEXHELPERS_H
#define RCS_VORTEXHELPERS_H

#include <Rcs_graph.h>

#include <Vx/VxPart.h>
#include <Vx/VxTransform.h>
#include <Vx/VxConstraint.h>

#include <fstream>

namespace Rcs
{

class VortexBody : public Vx::VxPart
{
public:
  VortexBody(const RcsBody* body);
  virtual ~VortexBody();
  const RcsBody* body;
  HTr A_PI;
};

//Rcs::VortexBody* getPartPtr2(const RcsBody* body);

Vx::VxTransform VxTransform_fromHTr(const HTr* A_KI);

void HTr_fromVxTransform(HTr* A_KI, const Vx::VxTransform tm);

Vx::VxCollisionGeometry* createSphere(const RcsShape* sh, const HTr* A_KI,
                                      Vx::VxMaterial* material);

Vx::VxCollisionGeometry* createCapsule(const RcsShape* sh, const HTr* A_KI,
                                       Vx::VxMaterial* material);

Vx::VxCollisionGeometry* createCylinder(const RcsShape* sh, const HTr* A_KI,
                                        Vx::VxMaterial* material);

Vx::VxCollisionGeometry* createSSR(const RcsShape* sh, const HTr* A_KI,
                                   Vx::VxMaterial* material);

Vx::VxCollisionGeometry* createBox(const RcsShape* sh, const HTr* A_KI,
                                   Vx::VxMaterial* material);

Vx::VxCollisionGeometry* createCone(const RcsShape* sh, const HTr* A_KI,
                                    Vx::VxMaterial* material);

Vx::VxCollisionGeometry* createMesh(const RcsShape* sh, const HTr* A_KI,
                                    Vx::VxMaterial* material);

Vx::VxCollisionGeometry* createTorus(const RcsShape* sh, const HTr* A_KI,
                                     Vx::VxMaterial* material);

void addTorus(const RcsShape* sh, const HTr* A_KI, Vx::VxPart** p,
              Vx::VxMaterial* material);

// If graph is NULL, no sensor offsets are considered
Vx::VxConstraint* createFixedJoint(Vx::VxPart* parent,
                                   Vx::VxPart* child,
                                   const RcsGraph* graph=NULL);

Vx::VxConstraint* createJoint1D(Vx::VxPart* parent,
                                Vx::VxPart* child,
                                const double jointLockStiffness,
                                const double jointLockDamping,
                                const double jointMotorLoss,
                                const double q0);

void printMaterial(const Vx::VxMaterial* material, std::ostream& out);
}

#endif // RCS_VORTEXHELPERS_H
