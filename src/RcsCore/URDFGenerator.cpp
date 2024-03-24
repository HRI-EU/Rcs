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

#include "URDFGenerator.h"

#include <Rcs_body.h>
#include <Rcs_Mat3d.h>
#include <EulerAngles.h>
#include <Rcs_material.h>
#include <Rcs_VecNd.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>

#include <sstream>


namespace Rcs
{

//------------------------------------------------------------------------

URDFGenerator::URDFGenerator(RcsGraph* graph) : m_graph(nullptr), m_withFloatingJoint(false), m_withDummyBase(false)
{
  if (graph)
  {
    m_graph = graph;
  }
  else
  {
    RLOG(5, "graph is invalid.");
  }
}

//------------------------------------------------------------------------

void URDFGenerator::setFloatingJoints(bool flag)
{
  m_withFloatingJoint = flag;
}

//------------------------------------------------------------------------

std::string URDFGenerator::toString()
{
  m_robot = graphToURDFElements(m_graph);

  if (m_robot)
  {
    return m_robot->toString();
  }
  return "NULL";
}

//------------------------------------------------------------------------

std::unique_ptr<URDFElement> URDFGenerator::graphToURDFElements(const RcsGraph* graph)
{
  // top-level
  if (graph == nullptr)
  {
    return nullptr;
  }

  auto robot = std::unique_ptr<URDFElement>(new URDFElement("robot"));
  robot->addAttribute("name", "RcsGeneratedRobot");

  // dummy root node. Every root node in the graph should be connected to this dummy root in exported urdf.
  int rootBdyNum = 0;
  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    const RcsBody* parent = RCSBODY_BY_ID(graph, BODY->parentId);
    if (parent == NULL)
    {
      ++rootBdyNum;
    }
  }

  const RcsBody* root = RCSBODY_BY_ID(graph, graph->rootId);
  if (std::string(root->name) != "dummy_base" && rootBdyNum > 1)
  {
    auto link = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("link"));
    link->addAttribute("name", "dummy_base");
    robot->addSubElement(std::move(link));
    m_withDummyBase = true;
  }

  // links
  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    addLinkAccordingToBody(robot.get(), graph, BODY);
  }

  // joints according to link
  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    if (std::string(BODY->name) != "dummy_base")
    {
      addJointAccordingToLink(robot.get(), graph, BODY);
    }
  }

  return robot;
}

//------------------------------------------------------------------------

void URDFGenerator::addLinkAccordingToBody(URDFElement* robot, const RcsGraph* graph, const RcsBody* body)
{
  auto link = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("link"));

  link->addAttribute("name", body->name);

  auto num = RcsBody_numJoints(graph, body);

  RLOG(5, "handling Rcs body, current body=%s, parent=%s, jointNumber=%d",
       body->name, RCSBODY_NAME_BY_ID(graph, body->parentId), num);

  // intertial
  handlingInertial(body, graph, link.get());

  // visual
  handlingVisual(body, link.get());

  // collision
  handlingCollision(body, link.get());

  robot->addSubElement(std::move(link));

  // if exist multiple joints, add dummy links.
  // if body->rigid_body_joints, it should be a floating joint after urdf.
  if (num > 1)
  {
    int dummyNum = 0;
    RCSBODY_FOREACH_JOINT(graph, body)
    {
      if (dummyNum != 0)
      {
        auto dummyLink = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("link"));
        const RcsJoint* prevJnt = RCSJOINT_BY_ID(graph, JNT->prevId);
        std::string lastJointName = prevJnt->name;
        dummyLink->addAttribute("name", lastJointName + "_to_" + JNT->name + "_dummy");
        robot->addSubElement(std::move(dummyLink));
      }
      ++dummyNum;
    }
  }

  const RcsJoint* bdyJnt = RCSJOINT_BY_ID(graph, body->jntId);
  if (bdyJnt && !HTr_isIdentity(&body->A_BP))  // -->A_JP-->A_BP. In this situation, we need an additional dummy link for relative transformation A_BP
  {
    RLOG(5, "add additional dummy link because of A_BP.");

    auto dummyLink = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("link"));
    std::string jointName = bdyJnt->name;
    dummyLink->addAttribute("name", jointName + "_to_A_BP_dummy");
    robot->addSubElement(std::move(dummyLink));
  }
}

//------------------------------------------------------------------------

void URDFGenerator::addJointAccordingToLink(URDFElement* robot, const RcsGraph* graph, const RcsBody* body)
{
  if (!body)
  {
    return;
  }

  // every body can only have one righid body joint or other joints
  // rigid_body_joints should be mapped to floating type in urdf. But there is not supported
  // for the urdf parser. See Rcs_URDFParser.c, line 370
  if (m_withFloatingJoint && body->rigid_body_joints)
  {
    handlingRigidBodyJoint(robot, graph, body);
    return;
  }

  unsigned int jointNumber = RcsBody_numJoints(graph, body);
  RLOG(5, "handling joint for body=%s, numJoints=%u, A_BP=%s",
       body->name, jointNumber, (HTr_isIdentity(&body->A_BP) ? "Identity" : "Not Identity"));
  const RcsBody* parentBdy = RCSBODY_BY_ID(graph, body->parentId);

  if (jointNumber == 0)  // for fixed joint.
  {
    auto uniqueJoint = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("joint"));

    // parent link
    auto parent = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("parent"));
    if (parentBdy)
    {
      uniqueJoint->addAttribute("name", std::string(body->name) + "_to_" + parentBdy->name +  "_dummy_joint");
      uniqueJoint->addAttribute("type","fixed");
      parent->addAttribute("link", parentBdy->name);
    }
    else if (!parentBdy && m_withDummyBase)
    {
      uniqueJoint->addAttribute("name", std::string(body->name) + "_to_dummy_base_dummy_joint");
      uniqueJoint->addAttribute("type","fixed");
      parent->addAttribute("link", "dummy_base");
    }
    uniqueJoint->addSubElement(std::move(parent));

    // child link
    auto child = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
    child->addAttribute("link",  body->name);
    uniqueJoint->addSubElement(std::move(child));

    // origin
    if (!HTr_isIdentity(&body->A_BP))
    {
      auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

      std::stringstream translation;
      std::stringstream rotation;
      translation <<  body->A_BP.org[0] << " " <<  body->A_BP.org[1] << " " << body->A_BP.org[2];

      double rpy[3];
      computeEulerAngles(rpy, MAT3D_CAST body->A_BP.rot, EulOrdXYZs);
      rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

      origin->addAttribute("xyz", translation.str());
      origin->addAttribute("rpy", rotation.str());
      uniqueJoint->addSubElement(std::move(origin));
    }

    if (m_withDummyBase || parentBdy)  // if body has no parent and has no dummy base in the graph. Then this body is root body. we ignore the joints on it.
    {
      robot->addSubElement(std::move(uniqueJoint));
    }
  }
  else if (jointNumber == 1)  // corner case for -->A_JP-->A_BP situation. create an additional dummy joint
  {
    auto uniqueJoint = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("joint"));
    const RcsJoint* bdyJnt = RCSJOINT_BY_ID(graph, body->jntId);
    const std::string dummyLinkName = std::string(bdyJnt->name) + "_to_A_BP_dummy";

    // joint name, type attribute and axis element
    setJointAttribute(bdyJnt, uniqueJoint.get());

    // parent link
    auto parent = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("parent"));
    if (parentBdy)
    {
      parent->addAttribute("link", parentBdy->name);
    }
    else
    {
      parent->addAttribute("link", "dummy_base");
    }
    uniqueJoint->addSubElement(std::move(parent));

    // child link
    if (!HTr_isIdentity(&body->A_BP))
    {
      auto child = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
      child->addAttribute("link",  dummyLinkName);  // the child body is the dummy link for the A_BP.
      uniqueJoint->addSubElement(std::move(child));
    }
    else
    {
      auto child = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
      child->addAttribute("link",  body->name);
      uniqueJoint->addSubElement(std::move(child));
    }

    // origin
    if (!HTr_isIdentity(&bdyJnt->A_JP))
    {
      auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

      std::stringstream translation;
      std::stringstream rotation;
      translation << bdyJnt->A_JP.org[0] << " " << bdyJnt->A_JP.org[1] << " " << bdyJnt->A_JP.org[2];

      double rpy[3];
      computeEulerAngles(rpy, MAT3D_CAST bdyJnt->A_JP.rot, EulOrdXYZs);
      rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

      origin->addAttribute("xyz", translation.str());
      origin->addAttribute("rpy", rotation.str());
      uniqueJoint->addSubElement(std::move(origin));
    }

    // limit
    auto limit = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("limit"));
    limit->addAttribute("effort", std::to_string(bdyJnt->maxTorque > 1000 ? 1000 : bdyJnt->maxTorque));
    limit->addAttribute("velocity", std::to_string(bdyJnt->speedLimit > 1000 ? 1000 : bdyJnt->speedLimit));
    limit->addAttribute("lower", std::to_string(bdyJnt->q_min));
    limit->addAttribute("upper", std::to_string(bdyJnt->q_max));
    uniqueJoint->addSubElement(std::move(limit));

    // mimic
    if (strlen(bdyJnt->coupledJntName)>0)
    {
      auto mimic = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("mimic"));
      mimic->addAttribute("joint", bdyJnt->coupledJntName);

      std::stringstream multiplier;
      multiplier << bdyJnt->couplingPoly[0];
      mimic->addAttribute("multiplier", multiplier.str());

      std::stringstream offset;
      RcsJoint* master = RCSJOINT_BY_ID(graph, bdyJnt->coupledToId);
      offset << bdyJnt->q_init - bdyJnt->couplingPoly[0] * master->q_init;
      mimic->addAttribute("offset", offset.str());
      uniqueJoint->addSubElement(std::move(mimic));
    }

    robot->addSubElement(std::move(uniqueJoint));

    // handling A_BP dummy joint
    if (!HTr_isIdentity(&body->A_BP))
    {
      auto dummyJoint = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("joint"));
      dummyJoint->addAttribute("name", std::string("A_BP_to_") + body->name + "_dummy_joint");
      dummyJoint->addAttribute("type","fixed");
      auto dummyParent = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("parent"));
      dummyParent->addAttribute("link", dummyLinkName);
      dummyJoint->addSubElement(std::move(dummyParent));
      auto dummyChild = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
      dummyChild->addAttribute("link",  body->name);
      dummyJoint->addSubElement(std::move(dummyChild));

      auto dummyOrigin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

      std::stringstream translation;
      std::stringstream rotation;
      translation <<  body->A_BP.org[0] << " " <<  body->A_BP.org[1] << " " << body->A_BP.org[2];

      double rpy[3];
      computeEulerAngles(rpy, MAT3D_CAST body->A_BP.rot, EulOrdXYZs);
      rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

      dummyOrigin->addAttribute("xyz", translation.str());
      dummyOrigin->addAttribute("rpy", rotation.str());
      dummyJoint->addSubElement(std::move(dummyOrigin));

      robot->addSubElement(std::move(dummyJoint));
    }
  }
  else if (jointNumber > 1)  // create dummy joint after last joint.
  {
    std::string dummyLinkName;
    unsigned int num = 0;
    RCSBODY_FOREACH_JOINT(graph, body)
    {
      if (num == jointNumber - 1)
      {
        dummyLinkName = std::string(RCSBODY_NAME_BY_ID(graph, body->jntId)) + "_to_A_BP_dummy";
      }

      auto subJoint = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("joint"));

      // joint name, type attribute and axis element
      setJointAttribute(JNT, subJoint.get());

      // parent link
      auto parent = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("parent"));
      const RcsBody* parentBdy = RCSBODY_BY_ID(graph, body->parentId);
      if (parentBdy && num == 0)
      {
        parent->addAttribute("link", parentBdy->name);
      }
      else if (!parentBdy && num == 0)
      {
        parent->addAttribute("link", "dummy_base");
      }
      else
      {
        const RcsJoint* prevJnt = RCSJOINT_BY_ID(graph, JNT->prevId);
        RCHECK(prevJnt);
        parent->addAttribute("link", std::string(prevJnt->name) + "_to_" + JNT->name + "_dummy");
      }
      subJoint->addSubElement(std::move(parent));

      // child link
      auto child = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
      const RcsJoint* nextJnt = RCSJOINT_BY_ID(graph, JNT->nextId);
      if (nextJnt)
      {
        child->addAttribute("link", std::string(JNT->name) + "_to_" + nextJnt->name + "_dummy");
      }
      else if (!nextJnt && !HTr_isIdentity(&body->A_BP))
      {
        child->addAttribute("link",  dummyLinkName);
      }
      else
      {
        child->addAttribute("link",  body->name);
      }
      subJoint->addSubElement(std::move(child));

      // origin      // todo: Check whether needs to apply joint chain transformation.
      // Every joint has its own pose relative to last joint. If all joint at the same point, just use the relative pose.
      auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

      // todo: this needs transform from first to current jnt.
      // for (int i = 0, currJNT = body->jnt; i < num; i++) {
      //     calculate the transformation.
      //     currJNT = body->jnt->next;
      // }
      std::stringstream translation;
      std::stringstream rotation;
      translation <<  JNT->A_JP.org[0] << " " <<  JNT->A_JP.org[1] << " " << JNT->A_JP.org[2];

      double rpy[3];
      computeEulerAngles(rpy, JNT->A_JP.rot, EulOrdXYZs);
      rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

      origin->addAttribute("xyz", translation.str());
      origin->addAttribute("rpy", rotation.str());
      subJoint->addSubElement(std::move(origin));

      // limit
      auto limit = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("limit"));
      limit->addAttribute("effort", std::to_string(JNT->maxTorque > 1000 ? 1000 : JNT->maxTorque));
      limit->addAttribute("velocity", std::to_string(JNT->speedLimit > 1000 ? 1000 : JNT->speedLimit));
      limit->addAttribute("lower", std::to_string(JNT->q_min));
      limit->addAttribute("upper", std::to_string(JNT->q_max));
      subJoint->addSubElement(std::move(limit));

      // mimic
      if (strlen(JNT->coupledJntName)!=0)
      {
        auto mimic = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("mimic"));
        mimic->addAttribute("joint", JNT->coupledJntName);

        std::stringstream multiplier;
        multiplier << JNT->couplingPoly[0];
        mimic->addAttribute("multiplier", multiplier.str());

        std::stringstream offset;
        RcsJoint* master = RCSJOINT_BY_ID(graph, JNT->coupledToId);
        offset << JNT->q_init - JNT->couplingPoly[0] * master->q_init;
        mimic->addAttribute("offset", offset.str());
        subJoint->addSubElement(std::move(mimic));
      }

      robot->addSubElement(std::move(subJoint));
      ++num;
    }

    if (!HTr_isIdentity(&body->A_BP))
    {
      auto dummyJoint = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("joint"));
      dummyJoint->addAttribute("name", std::string("A_BP_to_") + body->name + "_dummy_joint");
      dummyJoint->addAttribute("type","fixed");
      auto dummyParent = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("parent"));
      dummyParent->addAttribute("link", dummyLinkName);
      dummyJoint->addSubElement(std::move(dummyParent));
      auto dummyChild = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
      dummyChild->addAttribute("link",  body->name);
      dummyJoint->addSubElement(std::move(dummyChild));

      //if (body->A_BP)
      {
        auto dummyOrigin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

        std::stringstream translation;
        std::stringstream rotation;
        translation <<  body->A_BP.org[0] << " " <<  body->A_BP.org[1] << " " << body->A_BP.org[2];

        double rpy[3];
        computeEulerAngles(rpy, MAT3D_CAST body->A_BP.rot, EulOrdXYZs);
        rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

        dummyOrigin->addAttribute("xyz", translation.str());
        dummyOrigin->addAttribute("rpy", rotation.str());
        dummyJoint->addSubElement(std::move(dummyOrigin));
      }
      robot->addSubElement(std::move(dummyJoint));
    }
  }
}

//------------------------------------------------------------------------

void URDFGenerator::setJointAttribute(const RcsJoint* joint, URDFElement* urdfJoint)
{
  // joint name
  urdfJoint->addAttribute("name", joint->name);

  auto axis = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("axis"));

  // joint axis and type
  switch (joint->type)
  {
    case 0:
    {
      urdfJoint->addAttribute("type", "revolute");
      axis->addAttribute("xyz", "1 0 0");
      break;
    }
    case 1:
    {
      urdfJoint->addAttribute("type", "revolute");
      axis->addAttribute("xyz", "0 1 0");
      break;
    }
    case 2:
    {
      urdfJoint->addAttribute("type", "revolute");
      axis->addAttribute("xyz", "0 0 1");
      break;
    }
    case 3:
    {
      urdfJoint->addAttribute("type", "prismatic");
      axis->addAttribute("xyz", "1 0 0");
      break;
    }
    case 4:
    {
      urdfJoint->addAttribute("type", "prismatic");
      axis->addAttribute("xyz", "0 1 0");
      break;
    }
    case 5:
    {
      urdfJoint->addAttribute("type", "prismatic");
      axis->addAttribute("xyz", "0 0 1");
      break;
    }
    default:
    {
      RFATAL("joint type=%i, can not be handled.", joint->type);
    }
  }
  urdfJoint->addSubElement(std::move(axis));
}

//------------------------------------------------------------------------

void URDFGenerator::handlingInertial(const RcsBody* body, const RcsGraph* graph, URDFElement* link)
{
  auto inertial = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("inertial"));

  // mass
  auto mass = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("mass"));
  mass->addAttribute("value", std::to_string(body->m));
  inertial->addSubElement(std::move(mass));

  if (!Mat3d_isZero(MAT3D_CAST body->Inertia.rot) && !VecNd_isZero(body->Inertia.org, 3))
  {
    // origin
    auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));
    std::string cogVector = std::to_string(body->Inertia.org[0])
                            + " " + std::to_string(body->Inertia.org[1])
                            + " " + std::to_string(body->Inertia.org[2]);
    origin->addAttribute("xyz", cogVector);
    inertial->addSubElement(std::move(origin));

    // inertia
    auto inertia = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("inertia"));
    inertia->addAttribute("ixx", std::to_string(body->Inertia.rot[0][0]));
    inertia->addAttribute("iyy", std::to_string(body->Inertia.rot[1][1]));
    inertia->addAttribute("izz", std::to_string(body->Inertia.rot[2][2]));
    inertia->addAttribute("ixy", std::to_string(body->Inertia.rot[0][1]));
    inertia->addAttribute("ixz", std::to_string(body->Inertia.rot[0][2]));
    inertia->addAttribute("iyz", std::to_string(body->Inertia.rot[1][2]));
    inertial->addSubElement(std::move(inertia));
  }
  link->addSubElement(std::move(inertial));
}

//------------------------------------------------------------------------

void Rcs::URDFGenerator::handlingRigidBodyJoint(URDFElement* robot, const RcsGraph* graph, const RcsBody* body)
{
  if (body->rigid_body_joints)
  {
    RLOG(5, "body=%s has rigid body joints", body->name);
    auto rigidBodyJoint = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("joint"));
    const RcsBody* parentBdy = RCSBODY_BY_ID(graph, body->parentId);

    // joint name
    if (!parentBdy)    // if no parent, add a dummy joint to dummy_base
    {
      rigidBodyJoint->addAttribute("name", "dummy_joint");
    }
    else
    {
      rigidBodyJoint->addAttribute("name", std::string(body->name) + "_to_" + parentBdy->name + "_dummy_joint");
    }

    // joint type
    rigidBodyJoint->addAttribute("type","floating");

    // parent link
    auto parent = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("parent"));
    if (!parentBdy)
    {
      parent->addAttribute("link", "dummy_base");
    }
    else
    {
      parent->addAttribute("link", parentBdy->name);
    }
    rigidBodyJoint->addSubElement(std::move(parent));

    // child link
    auto child = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
    child->addAttribute("link",  body->name);
    rigidBodyJoint->addSubElement(std::move(child));

    // origin
    if (parentBdy && !HTr_isIdentity(&body->A_BP))
    {
      auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

      std::stringstream translation;
      std::stringstream rotation;
      translation <<  body->A_BP.org[0] << " " <<  body->A_BP.org[1] << " " << body->A_BP.org[2];

      double rpy[3];
      computeEulerAngles(rpy, MAT3D_CAST body->A_BP.rot, EulOrdXYZs);
      rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

      origin->addAttribute("xyz", translation.str());
      origin->addAttribute("rpy", rotation.str());
      rigidBodyJoint->addSubElement(std::move(origin));
    }

    robot->addSubElement(std::move(rigidBodyJoint));
    return;
  }
}

//------------------------------------------------------------------------

void URDFGenerator::handlingCollision(const RcsBody* body, URDFElement* link)
{

  if (body->nShapes==0)
  {
    RLOG(5, "body has no shape, bodyName: %s", body->name);
    return;
  }

  const std::string tag = "collision";
  RCSBODY_TRAVERSE_SHAPES(body)
  {
    if ((SHAPE->computeType & RCSSHAPE_COMPUTE_DISTANCE) == false)
    {
      continue;
    }

    switch (SHAPE->type)
    {
      case RCSSHAPE_SSL:   // to cylinder
      {
        addSphere(SHAPE, link, tag);
        addCylinder(SHAPE, link, tag);
        break;
      }
      case RCSSHAPE_CYLINDER:
      {
        addCylinder(SHAPE, link, tag);
        break;
      }
      case RCSSHAPE_SSR:  // to box
        RLOG(5, "SSR type is mapped to box.");
      case RCSSHAPE_BOX:
      {
        addBox(SHAPE, link, tag);
        break;
      }
      case RCSSHAPE_SPHERE:
      {
        addSphere(SHAPE, link, tag);
        break;
      }
      case RCSSHAPE_MESH:
      {
        addMesh(SHAPE, link, tag);
        break;
      }
      default:
      {
        RLOG(5, "ShapeType=%d, can not be exported.", SHAPE->type);
      }
    }

  }
}
//------------------------------------------------------------------------

void URDFGenerator::handlingVisual(const RcsBody* body, URDFElement* link)
{
  if (body->nShapes==0)
  {
    RLOG(5, "body has no shape, bodyName: %s", body->name);
    return;
  }

  const std::string tag = "visual";
  RCSBODY_TRAVERSE_SHAPES(body)
  {
    if ((SHAPE->computeType & RCSSHAPE_COMPUTE_GRAPHICS) == false)
    {
      continue;
    }

    switch (SHAPE->type)
    {
      case RCSSHAPE_SSL:   // to cylinder
      {
        addSphere(SHAPE, link, tag);
        addCylinder(SHAPE, link, tag);
        break;
      }
      case RCSSHAPE_CYLINDER:
      {
        addCylinder(SHAPE, link, tag);
        break;
      }
      case RCSSHAPE_SSR:  // to box
        RLOG(5, "SSR type is mapped to box.");
      case RCSSHAPE_BOX:
      {
        addBox(SHAPE, link, tag);
        break;
      }
      case RCSSHAPE_SPHERE:
      {
        addSphere(SHAPE, link, tag);
        break;
      }
      case RCSSHAPE_MESH:
      {
        addMesh(SHAPE, link, tag);
        break;
      }
      default:
      {
        RLOG(5, "ShapeType=%d, can not be exported.", SHAPE->type);
      }
    }
  }
}

//------------------------------------------------------------------------

void URDFGenerator::addSphere(const RcsShape* shape, URDFElement* link, const std::string& tag)
{
  if (shape->type != RCSSHAPE_SSL && shape->type != RCSSHAPE_SPHERE)
  {
    return;
  }

  int count = 0;
  do
  {
    ++count;
    auto sphere = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement(tag));

    auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

    std::stringstream translation;
    std::stringstream rotation;
    if (shape->type == RCSSHAPE_SSL)
    {
      HTr ssl;  // SSL cordinate center is at round side.
      HTr ssl2COB;
      HTr_setIdentity(&ssl);
      HTr_setIdentity(&ssl2COB);

      if (count % 2 == 0)
      {
        ssl.org[2] = shape->extents[2];  // other side of the SSL
      }

      HTr_transform(&ssl2COB, &shape->A_CB, &ssl);

      translation << ssl2COB.org[0] << " " << ssl2COB.org[1] << " " << ssl2COB.org[2];

      double rpy[3];
      computeEulerAngles(rpy, ssl2COB.rot, EulOrdXYZs);
      rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];
    }
    else
    {
      translation <<  shape->A_CB.org[0] << " " <<  shape->A_CB.org[1] << " " << shape->A_CB.org[2];
      double rpy[3];
      computeEulerAngles(rpy, MAT3D_CAST shape->A_CB.rot, EulOrdXYZs);
      rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];
    }

    origin->addAttribute("xyz", translation.str());
    origin->addAttribute("rpy", rotation.str());
    sphere->addSubElement(std::move(origin));

    // geometry
    auto geometry = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("geometry"));

    auto geometrySphere = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("sphere"));
    geometrySphere->addAttribute("radius", std::to_string(shape->extents[0]));
    geometry->addSubElement(std::move(geometrySphere));
    sphere->addSubElement(std::move(geometry));

    // material
    if (tag == "visual")
    {
      handlingMaterial(shape, sphere.get());
    }

    link->addSubElement(std::move(sphere));

  }
  while (shape->type == RCSSHAPE_SSL && count < 2);
}

//------------------------------------------------------------------------

void URDFGenerator::addCylinder(const RcsShape* shape, URDFElement* link, const std::string& tag)
{
  if (shape->type != RCSSHAPE_SSL && shape->type != RCSSHAPE_CYLINDER)
  {
    return;
  }

  auto cylinder = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement(tag));

  // origin
  auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

  std::stringstream translation;
  std::stringstream rotation;

  if (shape->type == RCSSHAPE_SSL)
  {
    HTr ssl;  // SSL cordinate center is at round side.
    HTr_setIdentity(&ssl);
    ssl.org[2] = shape->extents[2] * 0.5;

    HTr ssl2COB;
    HTr_setIdentity(&ssl2COB);
    HTr_transform(&ssl2COB, &shape->A_CB, &ssl);

    translation << ssl2COB.org[0]
                << " " << ssl2COB.org[1]
                << " " << ssl2COB.org[2];

    double rpy[3];
    computeEulerAngles(rpy, ssl2COB.rot, EulOrdXYZs);
    rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];
  }
  else
  {
    translation <<  shape->A_CB.org[0]
                << " " <<  shape->A_CB.org[1]
                << " " << shape->A_CB.org[2];
    double rpy[3];
    computeEulerAngles(rpy, MAT3D_CAST shape->A_CB.rot, EulOrdXYZs);
    rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];
  }

  origin->addAttribute("xyz", translation.str());
  origin->addAttribute("rpy", rotation.str());
  cylinder->addSubElement(std::move(origin));

  // geometry
  auto geometry = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("geometry"));

  auto geometryCylinder = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("cylinder"));
  geometryCylinder->addAttribute("radius", std::to_string(shape->extents[0]));
  geometryCylinder->addAttribute("length", std::to_string(shape->extents[2]));
  geometry->addSubElement(std::move(geometryCylinder));
  cylinder->addSubElement(std::move(geometry));

  // material
  if (tag == "visual")
  {
    handlingMaterial(shape, cylinder.get());
  }

  link->addSubElement(std::move(cylinder));
}

//------------------------------------------------------------------------

void URDFGenerator::addBox(const RcsShape* shape, Rcs::URDFElement* link, const std::string& tag)
{
  if (shape->type != RCSSHAPE_BOX)
  {
    return;
  }

  auto box = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement(tag));

  // origin
  auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

  std::stringstream translation;
  std::stringstream rotation;
  translation <<  shape->A_CB.org[0]
              << " " <<  shape->A_CB.org[1]
              << " " << shape->A_CB.org[2];

  double rpy[3];
  computeEulerAngles(rpy, MAT3D_CAST shape->A_CB.rot, EulOrdXYZs);
  rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

  origin->addAttribute("xyz", translation.str());
  origin->addAttribute("rpy", rotation.str());
  box->addSubElement(std::move(origin));

  // geometry
  auto geometry = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("geometry"));

  auto geometryBox = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("box"));
  std::string size = std::to_string(shape->extents[0]) + " " + std::to_string(shape->extents[1]) + " " + std::to_string(shape->extents[2]);
  geometryBox->addAttribute("size", size);
  geometry->addSubElement(std::move(geometryBox));
  box->addSubElement(std::move(geometry));

  // material
  if (tag == "visual")
  {
    handlingMaterial(shape, box.get());
  }

  link->addSubElement(std::move(box));
}

//------------------------------------------------------------------------

void URDFGenerator::addMesh(const RcsShape* shape, Rcs::URDFElement* link, const std::string& tag)
{
  if (shape->type != RCSSHAPE_MESH)
  {
    return;
  }

  auto mesh = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement(tag));

  // origin
  auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

  std::stringstream translation;
  std::stringstream rotation;
  translation <<  shape->A_CB.org[0]
              << " " <<  shape->A_CB.org[1]
              << " " << shape->A_CB.org[2];

  double rpy[3];
  computeEulerAngles(rpy, MAT3D_CAST shape->A_CB.rot, EulOrdXYZs);
  rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

  origin->addAttribute("xyz", translation.str());
  origin->addAttribute("rpy", rotation.str());
  mesh->addSubElement(std::move(origin));

  // geometry
  auto geometry = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("geometry"));
  auto geometryMesh = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("mesh"));
  std::string fileName = "file://";
  fileName += shape->meshFile;
  geometryMesh->addAttribute("filename", fileName);
  geometryMesh->addAttribute("scale", std::to_string(shape->scale3d[0]) + " " + std::to_string(shape->scale3d[1]) + " " + std::to_string(shape->scale3d[2]));
  geometry->addSubElement(std::move(geometryMesh));
  mesh->addSubElement(std::move(geometry));

  // material
  if (tag == "visual")
  {
    handlingMaterial(shape, mesh.get());
  }

  link->addSubElement(std::move(mesh));
}

//------------------------------------------------------------------------

void URDFGenerator::handlingMaterial(const RcsShape* shape, Rcs::URDFElement* visual)
{
  auto material = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("material"));
  if (shape->material)
  {
    material->addAttribute("name", shape->material);
  }

  // todo: how to map string to rgba? shape->color is a string
  const RcsMaterial* matData = Rcs_getMaterial(shape->color);

  auto color = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("color"));
  std::string colorStr;
  if (matData)
  {
    colorStr = std::to_string(matData->diff[0])
               + " " + std::to_string(matData->diff[1])
               + " " + std::to_string(matData->diff[2])
               + " " + std::to_string(matData->diff[3]);
  }
  else
  {
    colorStr = "0.7 0.7 0.7 1.0";  // default color.
  }
  color->addAttribute("rgba", colorStr);
  material->addSubElement(std::move(color));

  //texture
  if (shape->textureFile)
  {
    auto texture = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("texture"));
    texture->addAttribute("filename", shape->textureFile);
    material->addSubElement(std::move(texture));
  }
  visual->addSubElement(std::move(material));
}

}  // namespace Rcs
