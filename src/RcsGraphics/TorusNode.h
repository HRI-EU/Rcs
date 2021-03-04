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

#ifndef RCS_TORUSNODE_H
#define RCS_TORUSNODE_H

#include "NodeBase.h"

namespace Rcs
{
/*!
 * \ingroup RcsGraphics
 * \brief An OSG node that draws a torus
 */
class TorusNode : public NodeBase
{
public:
  TorusNode(double radius, double thickness, double startAng=0.0,
            double endAng=2.0*M_PI);

  virtual ~TorusNode();

  /*!
   * \brief Creates a osg::Geode that holds the vertices and normals of the
   *        torus
   */
  static osg::ref_ptr<osg::Geometry> createGeometry(double radius,
                                                    double thickness,
                                                    double startAng=0.0,
                                                    double endAng=2.0*M_PI);

  static void resize(double radius, double thickness, osg::Geometry* geometry);

private:
  static void createHelperArrays(double radius, double thickness,
                                 double startAngle, double endAngle,
                                 unsigned int nSides, unsigned int nRings,
                                 osg::Vec3Array* vertices,
                                 osg::Vec3Array* normals);
};

} // namespace Rcs

#endif // RCS_TORUSNODE_H
