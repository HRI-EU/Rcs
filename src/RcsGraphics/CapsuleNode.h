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

#ifndef RCS_CAPSULENODE_H
#define RCS_CAPSULENODE_H

#include "NodeBase.h"

#include <Rcs_mesh.h>

#include <osg/Geode>
#include <osg/Geometry>

namespace Rcs
{
/*!
 * \ingroup RcsGraphics
 */

class CapsuleGeometry : public osg::Group
{
public:
  CapsuleGeometry(double radius, double length, bool resizeable=false,
                  unsigned int nSegments=16);
  virtual ~CapsuleGeometry();
  void update(double radius, double length);
  double getRadius() const;
  double getLength() const;

private:

  void initMesh(unsigned int nSegments);
  void initPrimitive();

  osg::ref_ptr<osg::PositionAttitudeTransform> top, bottom, hull;
  double radius;
  double length;
};



class CapsuleNode: public NodeBase
{

public:

  CapsuleNode(const double ballPoint[3],
              double A_KI[3][3],
              double radius,
              double length,
              bool resizeable=false);

  /*! \brief Only works if the class has been instantiated with the resizeable
   *         flag.
   */
  void setLength(double length);

  /*! \brief Only works if the class has been instantiated with the resizeable
   *         flag.
   */
  void setRadius(double radius);

protected:

  osg::ref_ptr<Rcs::CapsuleGeometry> capsule;
};


}   // namespace Rcs

#endif // RCS_CAPSULENODE_H
