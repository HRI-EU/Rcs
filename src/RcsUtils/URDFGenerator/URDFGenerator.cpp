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

#include <sstream>
#include <Rcs_graphicsUtils.h>
#include <Rcs_macros.h>
#include "URDFGenerator.h"

namespace Rcs {

//------------------------------------------------------------------------

URDFGenerator::URDFGenerator(RcsGraph* graph) : m_graph(nullptr), m_withFloatingJoint(false)
{
    if (graph) {
        m_graph = graph;
    } else {
        RLOG(0, "graph is invalid.");
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
        return m_robot->toString();
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

    auto robot = std::unique_ptr<URDFElement>( new URDFElement("robot"));
    robot->addAttribute("name", "RcsGeneratedRobot");

    // dummy root node. Every root node in the graph should be connected to this dummy root in exported urdf.
    if (std::string(graph->root->name) != "dummy_base") {
        auto link = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("link"));
        link->addAttribute("name", "dummy_base");
        robot->addSubElement(std::move(link));
    }

    // links
    RCSGRAPH_TRAVERSE_BODIES(graph)
    {
        addLinkAccordingToBody(robot.get(), BODY);
    }

    // joints according to link
    RCSGRAPH_TRAVERSE_BODIES(graph)
    {
        if (std::string(BODY->name) != "dummy_base") {
            addJointAccordingToLink(robot.get(), BODY);
        }
    }

    return robot;
}

//------------------------------------------------------------------------

void URDFGenerator::addLinkAccordingToBody(URDFElement* robot, RcsBody* body)
{
    auto link = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("link"));

    link->addAttribute("name", body->name);

    RLOG(0, "handling Rcs body, current body=%s, parent=%s", body->name, body->parent ? body->parent->name : "NULL");

    // intertial
    handlingInertial(body, link.get());

    // visual
    handlingVisual(body, link.get());

    // collision
    handlingCollision(body, link.get());

    robot->addSubElement(std::move(link));

    // if exist multiple joints, add dummy links.
    // if body->rigid_body_joints, it should be a floating joint after urdf.
    if (body->jnt && body->jnt->next) {
        int dummyNum = 0;
        RCSBODY_TRAVERSE_JOINTS(body) {
            if (dummyNum != 0) {
                auto dummyLink = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("link"));
                std::string lastJointName = JNT->prev->name;
                dummyLink->addAttribute("name", lastJointName + "_to_" + JNT->name + "_dummy");
                robot->addSubElement(std::move(dummyLink));
            }
            ++dummyNum;
        }
    }
}

//------------------------------------------------------------------------

void URDFGenerator::addJointAccordingToLink(URDFElement* robot, RcsBody* body)
{
    if (!body) {
        return;
    }

    // every body can only have one righid body joint or other joints
    // rigid_body_joints should be mapped to floating type in urdf. But there is not supported
    // for the urdf parser. See Rcs_URDFParser.c, line 370
    if (m_withFloatingJoint && body->rigid_body_joints) {
        handlingRigidBodyJoint(robot, body);
        return;
    }

    unsigned int jointNumber = RcsBody_numJoints(body);
    RLOG(0, "handling joint for body=%s, numJoints=%u", body->name, jointNumber);

    if (jointNumber <= 1) {  // if the joint is the unique joint connects two body, then it is easy.
        auto uniqueJoint = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("joint"));

        // parent link
        auto parent = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("parent"));
        if (body->parent) {
            if (body->jnt) {
                // joint name, type attribute and axis element
                setJointAttribute(body->jnt, uniqueJoint.get());
            }
            else
            {
                uniqueJoint->addAttribute("name", std::string(body->name) + "_to_dummy_base_joint");
                uniqueJoint->addAttribute("type","fixed");
            }
            parent->addAttribute("link", body->parent->name);
        }
        else
        {
            if (body->jnt) {
                // joint name, type attribute and axis element
                setJointAttribute(body->jnt, uniqueJoint.get());
            }
            else
            {
                uniqueJoint->addAttribute("name", std::string(body->name) + "_to_dummy_base_joint");
                uniqueJoint->addAttribute("type","fixed");
            }
            parent->addAttribute("link", "dummy_base");
        }
        uniqueJoint->addSubElement(std::move(parent));

        // child link
        auto child = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
        child->addAttribute("link",  body->name);
        uniqueJoint->addSubElement(std::move(child));

        // origin
        if (body->jnt && body->jnt->A_JP) {
            auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

            std::stringstream translation;
            std::stringstream rotation;
            translation <<  body->jnt->A_JP->org[0] << " " <<  body->jnt->A_JP->org[1] << " " << body->jnt->A_JP->org[2];

            double rpy[3];
            computeEulerAngles(rpy, body->jnt->A_JP->rot, EulOrdXYZs);
            rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

            origin->addAttribute("xyz", translation.str());
            origin->addAttribute("rpy", rotation.str());
            uniqueJoint->addSubElement(std::move(origin));
        }

        // limit
        if (body->jnt) {
            auto limit = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("limit"));
            limit->addAttribute("effort", std::to_string(body->jnt->maxTorque > 1000 ? 1000 : body->jnt->maxTorque));
            limit->addAttribute("velocity", std::to_string(body->jnt->speedLimit > 1000 ? 1000 : body->jnt->speedLimit));
            limit->addAttribute("lower", std::to_string(body->jnt->q_min));
            limit->addAttribute("upper", std::to_string(body->jnt->q_max));
            uniqueJoint->addSubElement(std::move(limit));
        }

        robot->addSubElement(std::move(uniqueJoint));
    }
    else  // multiple joints
    {  // otherweise, we must add dummy subjoints.
        int num = 0;
        RCSBODY_TRAVERSE_JOINTS(body) {
            auto subJoint = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("joint"));

            // joint name, type attribute and axis element
            setJointAttribute(JNT, subJoint.get());

            // parent link
            auto parent = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("parent"));
            if (body->parent && num == 0) {
                parent->addAttribute("link", body->parent->name);
            } else if (!body->parent && num == 0) {
                parent->addAttribute("link", "dummy_base");
            } else {
                parent->addAttribute("link", std::string(JNT->prev->name) + "_to_" + JNT->name + "_dummy");
            }
            subJoint->addSubElement(std::move(parent));

            // child link
            auto child = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
            if (JNT->next) {
                child->addAttribute("link", std::string(JNT->name) + "_to_" + JNT->next->name + "_dummy");
            } else {
                child->addAttribute("link",  body->name);
            }
            subJoint->addSubElement(std::move(child));

            // origin      // todo: Check whether needs to apply joint chain transformation.
            if (JNT->A_JP) {  // Every joint has its own pose relative to last joint. If all joint at the same point, just use the relative pose.
                auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

                // todo: this needs transform from first to current jnt.
                // for (int i = 0, currJNT = body->jnt; i < num; i++) {
                //     calculate the transformation.
                //     currJNT = body->jnt->next;
                // }
                std::stringstream translation;
                std::stringstream rotation;
                translation <<  JNT->A_JP->org[0] << " " <<  JNT->A_JP->org[1] << " " << JNT->A_JP->org[2];

                double rpy[3];
                computeEulerAngles(rpy, JNT->A_JP->rot, EulOrdXYZs);
                rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

                origin->addAttribute("xyz", translation.str());
                origin->addAttribute("rpy", rotation.str());
                subJoint->addSubElement(std::move(origin));
            }

            // limit
            auto limit = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("limit"));
            limit->addAttribute("effort", std::to_string(JNT->maxTorque > 1000 ? 1000 : JNT->maxTorque));
            limit->addAttribute("velocity", std::to_string(JNT->speedLimit > 1000 ? 1000 : JNT->speedLimit));
            limit->addAttribute("lower", std::to_string(JNT->q_min));
            limit->addAttribute("upper", std::to_string(JNT->q_max));
            subJoint->addSubElement(std::move(limit));

            robot->addSubElement(std::move(subJoint));
            ++num;
        }
    }
}

//------------------------------------------------------------------------

void URDFGenerator::setJointAttribute(RcsJoint* joint, URDFElement* urdfJoint)
{
    // joint name
    urdfJoint->addAttribute("name", joint->name);

    auto axis = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("axis"));

    // joint axis and type
    switch(joint->type) {
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

void URDFGenerator::handlingInertial(RcsBody* body, URDFElement* link)
{
    auto inertial = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("inertial"));

    // mass
    auto mass = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("mass"));
    mass->addAttribute("value", std::to_string(body->m));
    inertial->addSubElement(std::move(mass));

    if (body->Inertia) {
        // origin
        auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));
        std::string cogVector = std::to_string(body->Inertia->org[0])
                                + " " + std::to_string(body->Inertia->org[1])
                                + " " + std::to_string(body->Inertia->org[2]);
        origin->addAttribute("xyz", cogVector);
        inertial->addSubElement(std::move(origin));

        // inertia
        auto inertia = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("inertia"));
        inertia->addAttribute("ixx", std::to_string(body->Inertia->rot[0][0]));
        inertia->addAttribute("iyy", std::to_string(body->Inertia->rot[1][1]));
        inertia->addAttribute("izz", std::to_string(body->Inertia->rot[2][2]));
        inertia->addAttribute("ixy", std::to_string(body->Inertia->rot[0][1]));
        inertia->addAttribute("ixz", std::to_string(body->Inertia->rot[0][2]));
        inertia->addAttribute("iyz", std::to_string(body->Inertia->rot[1][2]));
        inertial->addSubElement(std::move(inertia));
    }
    link->addSubElement(std::move(inertial));
}

//------------------------------------------------------------------------

void Rcs::URDFGenerator::handlingRigidBodyJoint(URDFElement* robot, RcsBody* body)
{
    if (body->rigid_body_joints) {
        RLOG(0, "body=%s has rigid body joints", body->name);
        auto rigidBodyJoint = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("joint"));

        // joint name
        if (!body->parent) {  // if no parent, add a dummy joint to dummy_base
            rigidBodyJoint->addAttribute("name", "dummy_joint");
        } else {
            rigidBodyJoint->addAttribute("name", std::string(body->name) + "_to_" + body->parent->name + "_dummy_joint");
        }

        // joint type
        rigidBodyJoint->addAttribute("type","floating");

        // parent link
        auto parent = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("parent"));
        if (!body->parent) {
            parent->addAttribute("link", "dummy_base");
        } else {
            parent->addAttribute("link", body->parent->name);
        }
        rigidBodyJoint->addSubElement(std::move(parent));

        // child link
        auto child = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("child"));
        child->addAttribute("link",  body->name);
        rigidBodyJoint->addSubElement(std::move(child));

        // origin
        if (body->parent && body->A_BP) {
            auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

            std::stringstream translation;
            std::stringstream rotation;
            translation <<  body->A_BP->org[0] << " " <<  body->A_BP->org[1] << " " << body->A_BP->org[2];

            double rpy[3];
            computeEulerAngles(rpy, body->A_BP->rot, EulOrdXYZs);
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

void URDFGenerator::handlingCollision(RcsBody* body, URDFElement* link)
{
    if (!body->shape)
    {
        RLOG(0, "body has no shape, bodyName: %s", body->name);
        return;
    }

    const std::string tag = "collision";
    RCSBODY_TRAVERSE_SHAPES(body)
    {
        if ((SHAPE->computeType & RCSSHAPE_COMPUTE_DISTANCE) == false)
        {
            continue;
        }

        switch (SHAPE->type) {
            case RCSSHAPE_SSL: { // to cylinder
                addSphere(SHAPE, link, tag);
                addCylinder(SHAPE, link, tag);
                break;
            }
            case RCSSHAPE_CYLINDER: {
                addCylinder(SHAPE, link, tag);
                break;
            }
            case RCSSHAPE_SSR:  // to box
                RLOG(0, "SSR type is mapped to box.");
            case RCSSHAPE_BOX: {
                addBox(SHAPE, link, tag);
                break;
            }
            case RCSSHAPE_SPHERE: {
                addSphere(SHAPE, link, tag);
                break;
            }
            case RCSSHAPE_MESH: {
                addMesh(SHAPE, link, tag);
                break;
            }
            default: {
                RLOG(0, "ShapeType=%d, can not be exported.", SHAPE->type);
            }
        }

    }
}
//------------------------------------------------------------------------

void URDFGenerator::handlingVisual(RcsBody* body, URDFElement* link)
{
    if (!body->shape)
    {
        RLOG(0, "body has no shape, bodyName: %s", body->name);
        return;
    }

    const std::string tag = "visual";
    RCSBODY_TRAVERSE_SHAPES(body)
    {
        if ((SHAPE->computeType & RCSSHAPE_COMPUTE_GRAPHICS) == false)
        {
            continue;
        }

        switch (SHAPE->type) {
            case RCSSHAPE_SSL: { // to cylinder
                addSphere(SHAPE, link, tag);
                addCylinder(SHAPE, link, tag);
                break;
            }
            case RCSSHAPE_CYLINDER: {
                addCylinder(SHAPE, link, tag);
                break;
            }
            case RCSSHAPE_SSR:  // to box
                RLOG(0, "SSR type is mapped to box.");
            case RCSSHAPE_BOX: {
                addBox(SHAPE, link, tag);
                break;
            }
            case RCSSHAPE_SPHERE: {
                addSphere(SHAPE, link, tag);
                break;
            }
            case RCSSHAPE_MESH: {
                addMesh(SHAPE, link, tag);
                break;
            }
            default: {
                RLOG(0, "ShapeType=%d, can not be exported.", SHAPE->type);
            }
        }
    }
}

//------------------------------------------------------------------------

void URDFGenerator::addSphere(RcsShape* shape, URDFElement* link, const std::string& tag)
{
    if (shape->type != RCSSHAPE_SSL && shape->type != RCSSHAPE_SPHERE) {
        return;
    }

    int count = 0;
    do {
        ++count;
        auto sphere = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement(tag));

        auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

        std::stringstream translation;
        std::stringstream rotation;
        if (shape->type == RCSSHAPE_SSL) {
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
            computeEulerAngles(rpy, shape->A_CB.rot, EulOrdXYZs);
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
        if (tag == "visual") {
            handlingMaterial(shape, sphere.get());
        }

        link->addSubElement(std::move(sphere));

    } while (shape->type == RCSSHAPE_SSL && count < 2);
}

//------------------------------------------------------------------------

void URDFGenerator::addCylinder(RcsShape* shape, URDFElement* link, const std::string& tag)
{
    if (shape->type != RCSSHAPE_SSL && shape->type != RCSSHAPE_CYLINDER) {
        return;
    }

    auto cylinder = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement(tag));

    // origin
    auto origin = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("origin"));

    std::stringstream translation;
    std::stringstream rotation;

    if (shape->type == RCSSHAPE_SSL) {
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
        computeEulerAngles(rpy, shape->A_CB.rot, EulOrdXYZs);
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
    if (tag == "visual") {
        handlingMaterial(shape, cylinder.get());
    }

    link->addSubElement(std::move(cylinder));
}

//------------------------------------------------------------------------

void URDFGenerator::addBox(RcsShape* shape, Rcs::URDFElement* link, const std::string& tag)
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
    computeEulerAngles(rpy, shape->A_CB.rot, EulOrdXYZs);
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
    if (tag == "visual") {
        handlingMaterial(shape, box.get());
    }

    link->addSubElement(std::move(box));
}

//------------------------------------------------------------------------

void URDFGenerator::addMesh(RcsShape* shape, Rcs::URDFElement* link, const std::string& tag)
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
    computeEulerAngles(rpy, shape->A_CB.rot, EulOrdXYZs);
    rotation << rpy[0] << " " << rpy[1] << " " << rpy[2];

    origin->addAttribute("xyz", translation.str());
    origin->addAttribute("rpy", rotation.str());
    mesh->addSubElement(std::move(origin));

    // geometry
    auto geometry = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("geometry"));
    auto geometryMesh = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("mesh"));
    geometryMesh->addAttribute("filename", shape->meshFile);
    geometryMesh->addAttribute("scale", std::to_string(shape->scale) + " " + std::to_string(shape->scale) + " " + std::to_string(shape->scale));
    geometry->addSubElement(std::move(geometryMesh));
    mesh->addSubElement(std::move(geometry));

    // material
    if (tag == "visual") {
        handlingMaterial(shape, mesh.get());
    }

    link->addSubElement(std::move(mesh));
}

//------------------------------------------------------------------------

void URDFGenerator::handlingMaterial(RcsShape* shape, Rcs::URDFElement* visual)
{
    auto material = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("material"));
    if (shape->material) {
        material->addAttribute("name", shape->material);
    }

    // todo: how to map string to rgba? shape->color is a string
    RcsMaterialData* matData = nullptr;
    matData = Rcs::getMaterial(shape->color);

    auto color = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("color"));
    std::string colorStr;
    if (matData) {
        colorStr = std::to_string(matData->diff[0])
                + " " + std::to_string(matData->diff[1])
                + " " + std::to_string(matData->diff[2])
                + " " + std::to_string(matData->diff[3]);
    }
    else {
        colorStr = "0.7 0.7 0.7 1.0";  // default color.
    }
    color->addAttribute("rgba", colorStr);
    material->addSubElement(std::move(color));

    //texture
    if (shape->textureFile) {
        auto texture = std::unique_ptr<Rcs::URDFElement>(new (std::nothrow) Rcs::URDFElement("texture"));
        texture->addAttribute("filename", shape->textureFile);
        material->addSubElement(std::move(texture));
    }
    visual->addSubElement(std::move(material));
}

}  // namespace Rcs
