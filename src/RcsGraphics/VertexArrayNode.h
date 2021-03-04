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

#ifndef VERTEXARRAYNODE_H
#define VERTEXARRAYNODE_H

#include "NodeBase.h"

#include <Rcs_MatNd.h>

#include <osg/Geometry>
#include <osg/Geode>



namespace Rcs
{

/*!
 * \ingroup RcsGraphics
 *
 *           This class displays a set of vertices. The vertices are extracted
 *           from the double pointer given to the constructor. It is assumed to
 *           have  the vertices ordered consecutively in the order xyz in
 *           memory. The array is dynamically interpreted, this means that
 *           if the  array contains less or more points, this is reflected
 *           in the visualization. In case there is an index mismatch, the
 *           function will insult you as you deserve it on debug levels 4 or
 *           higher. It must be made sure that the pointer is not getting
 *           invalid. The supported modes are
 *           - POINTS
 *           - LINES
 *           - LINE_STRIP
 *           - LINE_LOOP
 *           - TRIANGLES
 *           - TRIANGLE_STRIP
 *           - TRIANGLE_FAN
 *           - QUADS
 *           - QUAD_STRIP
 *           - POLYGON
 *
 */

class VertexArrayNode : public NodeBase
{
public:

  VertexArrayNode(osg::PrimitiveSet::Mode mode = osg::PrimitiveSet::LINES,
                  const std::string& color = "RED");
  VertexArrayNode(const double* points, size_t nPoints,
                  osg::PrimitiveSet::Mode mode = osg::PrimitiveSet::LINES,
                  const std::string& color = "RED");
  VertexArrayNode(const MatNd* points,
                  osg::PrimitiveSet::Mode mode=osg::PrimitiveSet::LINES,
                  const std::string& color = "RED");
  virtual ~VertexArrayNode();
  bool setPoints(const double* points, size_t nPoints);

  /*! \brief Array mat must be shaped nPoints x 2 or nPoints x 3.
   */
  bool setPoints(const MatNd* mat);
  bool copyPoints(const MatNd* mat);
  bool setColor(const std::string& color);
  bool setPointSize(float pointSize);
  void setLighting(bool enabled);
  void setManualUpdate(bool enabled);
  void performUpdate();
  bool takePointsOwnership();

protected:

  void init(const std::string& color);
  bool dataValid() const;
  bool updatePoints();
  virtual bool eventCallback(const osgGA::GUIEventAdapter& ea,
                             osgGA::GUIActionAdapter& aa);
  osg::ref_ptr<osg::Geode> geode;
  osg::ref_ptr<osg::Vec3Array> pointsArray;
  osg::ref_ptr<osg::Vec4Array> colors;
  osg::ref_ptr<osg::Geometry> geometry;
  osg::PrimitiveSet::Mode mode;
  const double* points;
  size_t nPoints;
  const MatNd* mat;
  MatNd* myMat;
  bool manual_update;
  bool perform_update;
};

}   // namespace Rcs

#endif   // VERTEXARRAYNODE_H
