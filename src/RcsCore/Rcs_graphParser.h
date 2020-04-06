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

#ifndef RCS_GRAPHPARSER_H
#define RCS_GRAPHPARSER_H


#include "Rcs_graph.h"

#include <Rcs_parser.h>


#ifdef __cplusplus
extern "C" {
#endif


#define RCSGRAPH_MAX_GROUPDEPTH (32)


/*!
 *  \page XMLParsing Graph description in XML
 *
 *  <h1> XML graph description </h1>
 *
 *  Values in the xml files are represented in human-readable units. For
 *  angles, it is degrees, for positions, it is meters.
 *
 *  <h2> Joints </h2>
 *
 * A RcsJoint holds a relative transform A_JP (Index P: Prevoius, index J:
 * Joint) that is followed by a transformation A_q resulting from the joints
 * degree of freedom. The relative transformation A_JP can be defined in the
 * joint's "transform" tag. The transformation A_q is an elementary rotation or
 * translation, depending on the joint's tag type. A joint can be preceeded
 * by another joint, a RcsBody, or by nothing (NULL). The joint's transformation
 * A_JI (Index I for Inertial or world frame) will be computed by propagating
 * the transformation of the joint's predecessor through A_JP and A_q. It
 * will be updated in each call to the forward kinematics.
 *
 * <img src="../images/RcsJoint.png" >
 *
 * \code
 *    <Joint name="WristZ"
 *           type="RotZ"
 *           transform="0 0 0 90 0 0"
 *           range="-60 10 60"
 *           torqueLimit="10"
 *           speedLimit="180"
 *           coupledTo="WristX"
 *           couplingFactor="0.4" />
 * \endcode
 *
 * - name: Joint name. If no name is given, the name will be set to
 *         "unnamed joint". It is recommended to give a name in order to avoid
 *         having several joints with the same name.
 *
 * - type: TransX, TransY, TransZ, RotX, RotY, RotZ for translation or
 *         rotation in the respective direction.
 *
 * - range: Lower limit, center position and upper limit values of the joint.
 *          If range contains one value, it will be set to "-value 0 +value".
 *          If range contains two values, it will be set to 
 *          "value_1 0.5*(value_1+value_2) value_2", essentially setting the
 *          center position to the middle between lower and upper range. If
 *          range contains three values, it will be set to 
 *          "value_1 value_2 value_3"
 *
 * - weightJL: Pre-scaler for the joint's joint limit cost and gradient
 *             contribution. Default is 1.
 *
 * - weightCA: Pre-scaler for the joint's collision cost and gradient
 *             contribution. Default is 1
 *
 * - weightMetric: Pre-scaler for the joint space weighting matrix. It is
 *                 reflected in the function \ref RcsGraph_getInvWq().
 *                 Default is 1
 *
 * - torqueLimit: Default is no limit.
 *
 * - speedLimit: Default is no limit. For rotational joints, the limits are
 *               interpreted in degrees per seconds, for translational joints,
 *               they are interpreted in meters per second.
 *
 * - gearRatio: Default is 1
 *
 * - coupledTo: Name of the joint the current one is kinematically coupled to.
 *              Default is none
 *
 * - couplingFactor: Default is none
 *
 * - transform: Relative transformation of the joint with respect to its
 *              predecessor. The predecessor can be another joint, or a body
 *              (if the joint is the first one linked to a body). The order
 *              of transformations is x-y-z-a-b-c with a-b-c being the
 *              rotations about the rotated frame (Rotating Euler angles). The
 *              default is an identity transformation.
 *
 * - ctrlType: Control type of the joint inside the physics simulation. Options
 *             are "Position" or "pos" for position-controlled joints,
 *             "Velocity" or "vel" for velocity-controlled joints or "Torque"
 *             or "tor" for torque-controlled joints. The default is position-
 *             controlled.
 *
 * - constraint: True if the joint is kinematically constrained, false
 *               otherwise. Default is false.
 *
 *
 *
 * <br>
 * <h2> Shapes </h2>
 *
 * Any number of RcsShapes can be associated with a RcsBody. Their relative
 * transformation with respect to the boie's frame can be defined in the
 * RcsShape's tag "transform". If none is given, the identity transform is
 * the default.
 *
 * <img src="../images/RcsShape.png" >
 *
 * \code
 *    <Shape type="MESH"
 *           physics="true"
 *           graphics="false"
 *           meshFile="myMesh.tri"
 *           color="ORANGE" />
 * \endcode
 *
 * - type:
 *   - SSL
 *   - SSR
 *   - BOX
 *   - CYLINDER
 *   - MESH
 *   - FRAME
 *   - SPHERE
 *   - CONE
 *   - GPISF
 *   - TORUS
 *   - OCTREE
 *   - POINT
 *   - MARKER
 *
 * - transform: Relative transformation of the shape with respect to the body
 *              it belongs. The order of transformations is x-y-z-a-b-c with
 *              a-b-c being the rotations about the rotated frame (Rotating
 *              Euler angles). The default is an identity transformation.
 *
 * - scale: Scaling factor, has influence on mesh files and frames only.
 *
 * - distance: Shape will be considered in distance calculations when true
 *
 * - physics: Shape will be considered in physics simulation when true
 *
 * - graphics: Shape will be visualized in graphics model when true
 *
 * - extents: x, y and z dimensions of shape. For some shapes, the index 0 is
 *            considered as a radius, and the index 2 as a length (e.g. SSL,
 *            CYLINDER, CONE ...). For these, the tags radius and length could
 *            also be used.
 *
 * - radius: Will be written to extents[0]
 *
 * - length: Will be written to extents[2]
 *
 * - meshFile: Absolute file name of mesh file if the shape's type is MESH
 *
 * - textureFile: Absolute file name of texture file to be applied in
 *                graphics visulaization
 *
 * - color: Name of the color, as defined in the colors.xml config file.
 *
 * - material: Material name of the shape for physics simulation. Its parameters
 *             are assumed to be defined in the physics configuration file.
 *
 *
 *
 * <br>
 * <h2> Bodies </h2>
 *
 * A RcsBody can contain no, one or any number of RcsJoints. Their ordering is
 * depicted in the image below: The absolute body transformation is computed by
 * propagating the transformation of the previous body through the joints, and
 * then through the relative body transformation A_BP. This relative
 * transformation A_BP can be specified in the tag "transform" of the body.
 * Please note that the order of transformations is different to how they
 * are written in the xml description. In the xml description, the relative
 * body transformation is put first, followed by the joint descriptions. It is
 * a bit counter intuitive. Any sugestion how to improve it is welcome.
 *
 * <img src="../images/RcsBody.png" >
 *
 * \code
 * <Body name="Hand"
 *       prev="Wrist"
 *       transform="0.1 0 0 30 0 90"
 *       mass="1.0"
 *       cogVector="0.1 0 0"
 *       physics="fixed"
 *       color="GREEN"
 *       inertia="1 0 0 0 1 0 0 0 1" >
 *
 *       <Shape ... />
 *       <Sensor ... />
 *       <Joint ... />
 * </Body>
 * \endcode
 *
 * Bodies can contain an arbitrary number of shapes and joints, which are
 * explained below. As for the joints, each body description contains the
 * joints that attach it to its predecessor.
 *
 * - name: Body name. If no name is given, the name will be set to
 *         "unnamed body <num>" where num is a unique number that is incremented
 *         for each body. It is required to assign a unique name to the
 *         body in order to avoid having several body with the same name.
 *         The function \ref RcsGraph_check() checks for this.
 *
 * - prev: Body name of the parent body. If it is not given, the bodie's
 *         parent is the world coordinate frame.
 *
 * - transform: Relative transformation of the body with respect to its
 *              predecessor. The order of transformations is x-y-z-a-b-c with
 *              a-b-c being the rotations about the rotated frame (Rotating
 *              Euler angles). The default is an identity transformation. If
 *              this tag exists, there must be no tag with the name "quat".
 *
 * - quat: Relative quaternion (Convention w-x-y-z) of the body with respect
 *         to its predecessor. If this tag exists, there must be no tag with
 *         the name "transform".
 *
 * - physics: How the body is simulated in the rigid physics simulation. The
 *            following options exist:
 *            - kinematic: Collision response, but no dynamics
 *            - dynamic: Dynamic response
 *            - fixed: Dynamic response, but rigidly fixed to predecessor
 *
 * - color: String with a color defined in the file "colors.xml". If the
 *          color does not exist, a default color is used (gray). The color
 *          sets the default color for all shapes that belong to the body.
 *          They can individually be overwritten in each shape.
 *
 * - rigid_body_joints: This tag can either be given with <true/false>, or
 *                      with 6 values that represent a transformation. In the
 *                      first case, 6 constrained joints in the order
 *                      x-y-z-a-b-c will be created and linked to this body.
 *                      They correspond to the 6 rigid body degrees of motion.
 *                      In the second case, the same joints are created, and
 *                      their joint centers are set to the transformation that
 *                      correspond to these values. The default is "false".
 *
 * - mass: Mass of the body in [kg]. It must not be negative. The default is
 *         zero.
 *
 * - inertia: Inertia tensor around the COM, represented in the bodie's frame
 *            of reference. The default is a boolean addition of each of the
 *            bodie's shapes. For some shapes, there is no inertia calculation
 *            (e.g. meshes). See the shape's explanation for more details.
 *
 * - cogVector: Vector from body origin to body COM, represented in the
 *              bodie's frame of reference. The default is zero.
 */



/*! \ingroup RcsParserFunctions
 *  \brief Creates an instance of a graph from a given XML tree.
 */
RcsGraph* RcsGraph_createFromXmlNode(const xmlNodePtr node);

/*! \ingroup RcsParserFunctions
*  \brief Loads the graph's xml file, looks for the tag "model_state" with
*         the specified time stamp, and if found, sets the state to the
*         corresponding values. Currently, timeStamp is ignored.
*
*  \return true for success, false otherwise.
 */
bool RcsGraph_setModelStateFromXML(RcsGraph* self, const char* modelStateName,
                                   int timeStamp);

/*! \ingroup RcsParserFunctions
 *  \brief Reads the model state from a graph. The function opens the xml file
 *         corresponding to the graph's xmlFileName. It then reads all values
 *         with an xml tag "joint_state". All other values remain unchanged.
 *         The qrray q will be reshaped to the correct dimensions.
 */
bool RcsGraph_getModelStateFromXML(MatNd* q, const RcsGraph* self,
                                   const char* modelStateName, int timeStamp);

#ifdef __cplusplus
}
#endif

#endif   // RCS_GRAPHPARSER_H
