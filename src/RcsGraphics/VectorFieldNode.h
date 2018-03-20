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

#ifndef VECTORFIELDNODE_H
#define VECTORFIELDNODE_H

#include <Rcs_MatNd.h>
#include <Rcs_HTr.h>

#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>

namespace Rcs
{

/**
 * \ingroup RcsGraphics
 */
class VectorFieldNode : public osg::PositionAttitudeTransform
{
public:
  VectorFieldNode(const MatNd* xdx, const HTr* refFrame, double scale = 1.0, bool withPoints = true, const char* color = "BLACK_RUBBER");
  VectorFieldNode(const MatNd* x, const MatNd* dx, const HTr* refFrame, double scale = 1.0, bool withPoints = true, const char* color = "BLACK_RUBBER");

  ~VectorFieldNode();

  void setTransformation(const HTr* A_KI);
  const HTr* getTransform() const;

protected:
  void init(const char* color);

  osg::ref_ptr<osg::Vec3Array> lines;
  osg::ref_ptr<osg::Vec3Array> points;
  //  osg::Vec3Array* _color;

  osg::ref_ptr<osg::Geometry> pointsGeom;
  osg::ref_ptr<osg::Geometry> linesGeom;

  const HTr* refFrame;
};

} // namespace Rcs

#endif // VECTORFIELDNODE_H
