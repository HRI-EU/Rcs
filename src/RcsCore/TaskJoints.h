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

#ifndef RCS_TASKJOINTS_H
#define RCS_TASKJOINTS_H

#include "CompositeTask.h"


namespace Rcs
{
/*! \ingroup RcsTask
 * \brief This tasks allows to control a set of joints in a single task.
 *
 * First example: Specifying joint names
 *
 * \code
 *    <Task controlVariable="Joints"
 *          jnts="joint1 joint2 joint3"
 *          refJnts="basejoint1 basejoint2 basejoint3"
 *          refGains="1 2 3" />
 * \endcode
 *
 * Second example: Specifying body names. If a body is specified by the
 * effector or refBdy attribute, all joints that connect the body to its
 * predecessor are considered.
 *
 * \code
 *    <Task controlVariable="Joints"
 *          effector="body"
 *          refBdy="basebody"
 *          refGains="1" />
 * \endcode
 *
 * - jnts: Space-separated list of joint names that are to be controlled
 *
 * - refJnts: Optional space-separated list of joint names. The joints listed
 *            in the jnts attribute are described with respect to the refJnts.
 *            It means that the velocities of the jnts are added to the ones
 *            of the refJnts with a given scaling factor (see refGains).
 *            If this attribute is given, the number of refJnts must match the
 *            number of the attribute jnts.
 *
 * - refGains: Optional space-separated list of scaling factors. If this
 *             attribute is not given, the default is 1. If this attribute is
 *             given, the number of refJnts must either match the number of
 *             the attribute jnts, or must be 1. In the latter case, the same
 *             scaling factor is applied to all joints.
 *
 * - effector: All joints that connect the body to its predecessors are
 *             extracted. This attribute is exclusive to jnts
 *
 * - refBdy: All joints that connect the body to its predecessors are
 *           extracted as refJnts. The dimensions must match. This attribute
 *           is exclusive to refJnts.
 */
class TaskJoints: public CompositeTask
{
public:

  /*! Constructor based on xml parsing
   */
  TaskJoints(const std::string& className, xmlNode* node, RcsGraph* graph);

  /*! Constructor based on joints of a body
   */
  TaskJoints(const RcsBody* effector, RcsGraph* graph);

  /*! Constructor based on linking rigid bodies
   */
  TaskJoints(const RcsBody* effector, const RcsBody* refBdy, RcsGraph* graph);

  /*! \brief Polymorphic clone function.
   */
  TaskJoints* clone(RcsGraph* newGraph=NULL) const;

  void setJoints(std::vector<const RcsJoint*> jnt);
  void setRefJoints(std::vector<const RcsJoint*> jnt);
  void setRefJoint(size_t index, const RcsJoint* jnt);
  void setRefGains(std::vector<double> gains);
  void setRefGains(double gain);
  std::vector<const RcsJoint*> getJoints() const;
  std::vector<const RcsJoint*> getRefJoints() const;
  std::vector<double> getRefGains() const;

  /*! \brief Returns true if the task is specified correctly, false
   *         otherwise. The following checks are performed:
   *         - All joints in attribute "jnts" exist
   *         - If refJnts is given:
   *           - All joints in attribute "refJnts" exist
   *           - Number of refJnts matches number of joints
   *         - If refGains is given:
   *           - Number of refGains matches one or number of joints
   *         - If effector is given:
   *           - Attribute "jnts" is not specified
   *           - Corresponding body exists in graph
   *         - If refBdy is given:
   *           - Attribute "refFnts" is not specified
   *           - Corresponding body exists in graph
   *           - Number of body joints matches number of the joints specified
   *             in the task
   */
  static bool isValid(xmlNode* xml_node, const RcsGraph* graph);

protected:

  /*! \brief Writes the specific task's xml representation to a file
   *         desriptor. Here it is effector, refBdy, refFrame. To be
   *         overwritten in specialized classes.
   */
  virtual void toXMLBody(FILE* out) const;

};

}

#endif // RCS_TASKJOINTS_H
