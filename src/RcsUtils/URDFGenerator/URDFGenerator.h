/**************************************************************************
**  Copyright (c) 2017, Honda Research Institute Europe GmbH
**
**  Redistribution and use in source and binary forms, with or without
**  modification, are permitted provided that the following conditions are
**  met:
**
**  1. Redistributions of source code must retain the above copyright notice,
**  this list of conditions and the following disclaimer.
**
**  2. Redistributions in binary form must reproduce the above copyright
**  notice, this list of conditions and the following disclaimer in the
**  documentation and/or other materials provided with the distribution.
**
**  3. Neither the name of the copyright holder nor the names of its
**  contributors may be used to endorse or promote products derived from
**  this software without specific prior written permission.
**
**  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
**  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
**  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
**  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
**  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
**  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
**  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
**  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
**  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
**  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**  Author: weima
**  Date: 19 Oct 2022
**
**************************************************************************/


#ifndef URDFGENERATOR_H
#define URDFGENERATOR_H

#include <Rcs_body.h>
#include <Rcs_graph.h>
#include <Rcs_macros.h>
#include <Rcs_resourcePath.h>
#include <Rcs_typedef.h>
#include <Rcs_Mat3d.h>
#include <EulerAngles.h>

#include <memory>
#include <string>
#include "URDFElement.h"


namespace Rcs {
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
    void addLinkAccordingToBody(URDFElement* robot, RcsBody* body);
    void addJointAccordingToLink(URDFElement* robot, RcsBody* body);
    void setJointAttribute(RcsJoint* joint, Rcs::URDFElement* urdfJoint);
    void handlingInertial(RcsBody* body, Rcs::URDFElement* link);
    void handlingRigidBodyJoint(Rcs::URDFElement* robot, RcsBody* body);
    void handlingCollision(RcsBody* body, Rcs::URDFElement* link);
    void handlingVisual(RcsBody* body, Rcs::URDFElement* link);
    void addSphere(RcsShape* shape, Rcs::URDFElement* link, const std::string& tag);
    void addCylinder(RcsShape* shape, Rcs::URDFElement* link, const std::string& tag);
    void addBox(RcsShape* shape, Rcs::URDFElement* link, const std::string& tag);
    void addMesh(RcsShape* shape, Rcs::URDFElement* link, const std::string& tag);
    void handlingMaterial(RcsShape* shape, Rcs::URDFElement* visual);

  private:
    RcsGraph* m_graph;
    std::unique_ptr<URDFElement> m_robot;
    bool m_withFloatingJoint;
    bool m_withDummyBase;
}

#endif // URDFGENERATOR_H
