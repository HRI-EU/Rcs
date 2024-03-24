/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
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

#ifndef RCS_URDFGENERATOR_H
#define RCS_URDFGENERATOR_H

#include "URDFElement.h"

#include <Rcs_graph.h>


namespace Rcs
{
class URDFGenerator final
{
public:
  explicit URDFGenerator(RcsGraph* graph);

  // no copy allowed
  URDFGenerator(const URDFGenerator& rhs) = delete;
  URDFGenerator& operator=(const URDFGenerator& rhs) = delete;

  // set true, if you want to export the rigid body joints as floating joints.
  // default is false.
  void setFloatingJoints(bool flag);

  std::string toString();

private:
  std::unique_ptr<URDFElement> graphToURDFElements(const RcsGraph* graph);
  void addLinkAccordingToBody(URDFElement* robot, const RcsGraph* graph, const RcsBody* body);
  void addJointAccordingToLink(URDFElement* robot, const RcsGraph* graph, const RcsBody* body);
  void setJointAttribute(const RcsJoint* joint, URDFElement* urdfJoint);
  void handlingInertial(const RcsBody* body, const RcsGraph* graph, URDFElement* link);
  void handlingRigidBodyJoint(URDFElement* robot, const RcsGraph* graph, const RcsBody* body);
  void handlingCollision(const RcsBody* body, URDFElement* link);
  void handlingVisual(const RcsBody* body, URDFElement* link);
  void addSphere(const RcsShape* shape, URDFElement* link, const std::string& tag);
  void addCylinder(const RcsShape* shape, URDFElement* link, const std::string& tag);
  void addBox(const RcsShape* shape, URDFElement* link, const std::string& tag);
  void addMesh(const RcsShape* shape, URDFElement* link, const std::string& tag);
  void handlingMaterial(const RcsShape* shape, URDFElement* visual);

  RcsGraph* m_graph;
  std::unique_ptr<URDFElement> m_robot;
  bool m_withFloatingJoint;
  bool m_withDummyBase;
};

}

#endif // RCS_URDFGENERATOR_H
