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

#include "VertexArrayNode.h"
#include "Rcs_graphicsUtils.h"
#include "RcsViewer.h"

#include <KeyCatcherBase.h>
#include <Rcs_macros.h>

#include <osg/Material>
#include <osg/LineWidth>
#include <osg/Point>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VertexArrayNode::VertexArrayNode(osg::PrimitiveSet::Mode mode_,
                                      const std::string& color) :
  NodeBase(), mode(mode_), points(NULL), nPoints(0), mat(NULL), myMat(NULL)
{
  init(color);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VertexArrayNode::VertexArrayNode(const double* points_, size_t nPoints_,
                                      osg::PrimitiveSet::Mode mode_,
                                      const std::string& color) :
  NodeBase(), mode(mode_), points(points_), nPoints(nPoints_),
  mat(NULL), myMat(NULL)
{
  init(color);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VertexArrayNode::VertexArrayNode(const MatNd* mat_,
                                      osg::PrimitiveSet::Mode mode_,
                                      const std::string& color)
  : NodeBase(), mode(mode_), points(mat_ ? mat_->ele : NULL),
    nPoints(mat_ ? mat_->m : 0), mat(mat_), myMat(NULL)
{
  init(color);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::VertexArrayNode::~VertexArrayNode()
{
  MatNd_destroy(this->myMat);   // Accepts NULL arg
}

/*******************************************************************************
 * Sets a point array.
 ******************************************************************************/
void Rcs::VertexArrayNode::init(const std::string& color)
{
  this->pointsArray = new osg::Vec3Array;
  this->geometry = new osg::Geometry;
  this->geometry->addPrimitiveSet(new osg::DrawArrays(this->mode));
  this->geode = new osg::Geode();
  this->geode->addDrawable(this->geometry.get());
  this->patPtr()->addChild(this->geode.get());
  this->colors = new osg::Vec4Array;
  this->manual_update = false;
  this->perform_update = false;

  setName("VertexArrayNode");
  setColor(color);
  setLighting(true);
  setPointSize(3.0);
  setPoints(this->points, this->nPoints);

  KeyCatcherBase::registerKey("v", "Toggle visualization VertexArrayNodes",
                              "Viewer");

  makeDynamic();
}

/*******************************************************************************
 * Sets a point array.
 ******************************************************************************/
bool Rcs::VertexArrayNode::eventCallback(const osgGA::GUIEventAdapter& ea,
                                         osgGA::GUIActionAdapter& aa)
{
  switch (ea.getEventType())
  {
    case (osgGA::GUIEventAdapter::KEYDOWN):
    {
      // Toggle visibility
      if (ea.getKey() == 'v')
      {
        toggle();
      }
      break;
    }

    case (osgGA::GUIEventAdapter::FRAME):
    {
      if (!manual_update || perform_update)
      {
        perform_update = false;
        bool success = updatePoints();

        if (success == false)
        {
          RLOG(4, "Failed to set points in VertexArrayNode");
        }
      }
      break;
    }

    default:
      break;

  }   // switch(...)

  return false;
}

/*******************************************************************************
 * Sets a point array.
 ******************************************************************************/
bool Rcs::VertexArrayNode::setPoints(const MatNd* mat_)
{
  if (mat_==NULL)
  {
    RLOG(4, "Matrix is NULL - skipping setPoints()");
    return false;
  }
  this->mat = mat_;
  return setPoints(mat_->ele, mat_->m);
}

/*******************************************************************************
 * Copies and sets a point array.
 ******************************************************************************/
bool Rcs::VertexArrayNode::copyPoints(const MatNd* mat_)
{
  if (mat_==NULL)
  {
    RLOG(4, "Matrix is NULL - skipping copyPoints()");
    return false;
  }

  this->mat = mat_;
  return takePointsOwnership();
}

/*******************************************************************************
 * Sets a point array.
 ******************************************************************************/
bool Rcs::VertexArrayNode::setPoints(const double* points_, size_t nPoints_)
{
  this->nPoints = nPoints_;
  this->points  = points_;

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VertexArrayNode::updatePoints()
{
  // update nPoints, in case of a given matrix (m might be dynamic)
  if (this->mat)
  {
    this->nPoints = this->mat->m;
  }

  bool success = true;

  if (!dataValid())
  {
    success = false;
  }

  this->pointsArray->clear();

  if (success == true)
  {
    // special case if 2D points are given --> Z = 0
    bool mat_is_2D = (this->mat && this->mat->n == 2);
    int stride = mat_is_2D ? 2 : 3;

    for (size_t i = 0; i < this->nPoints; i++)
    {
      const double* row = &this->points[i*stride];

      if (fabs(row[0])>=DBL_MAX || fabs(row[1])>=DBL_MAX)
      {
        continue;
      }

      if (!mat_is_2D && fabs(row[2])>=DBL_MAX)
      {
        continue;
      }

      this->pointsArray->push_back(osg::Vec3(row[0], row[1],
                                             mat_is_2D ? 0.0 : row[2]));
    }
  }

  osg::DrawArrays* ps =
    ((osg::DrawArrays*)(this->geometry)->getPrimitiveSet(0));
  ps->setCount(this->pointsArray->size());
  this->geometry->setVertexArray(this->pointsArray.get());
  this->geometry->setPrimitiveSet(0, ps);

  return success;
}

/*******************************************************************************
 * Sets the color to a point array.
 ******************************************************************************/
bool Rcs::VertexArrayNode::setColor(const std::string& color)
{
  RcsMaterialData* material = getMaterial(color);

  if (material == NULL)
  {
    RLOG(4, "Failed to set color \"%s\"", color.c_str());
    return false;
  }

  this->colors->clear();
  this->colors->push_back(material->diff);
  this->geometry->setColorArray(this->colors.get());
  this->geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
  //this->geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

  return true;
}

/*******************************************************************************
 * Sets the point size of a point array.
 ******************************************************************************/
bool Rcs::VertexArrayNode::setPointSize(float ptSize)
{
  if (ptSize < 1.0)
  {
    RLOG(4, "Point size smaller 1.0: is %f", ptSize);
    return false;
  }

  osg::StateSet* stateset = this->geode->getOrCreateStateSet();
  osg::LineWidth* line = new osg::LineWidth();
  osg::Point* point = new osg::Point();

  line->setWidth(ptSize);
  point->setSize(ptSize);
  stateset->setAttribute(point, osg::StateAttribute::ON);
  stateset->setAttribute(line, osg::StateAttribute::ON);
  this->geode->setStateSet(stateset);

  return true;
}

/*******************************************************************************
 * Returns true if the point data is vaild, false otherwise
 ******************************************************************************/
bool Rcs::VertexArrayNode::dataValid() const
{
  // Even number of points for lines
  if ((this->mode == osg::PrimitiveSet::LINES) && (this->nPoints % 2 != 0))
  {
    RLOG(4, "To display a line, this class expects an array with an even number"
         " of points. But it holds an uneven (%zu) number!", this->nPoints);
    return false;
  }

  // NULL pointers, or zero points in array
  if ((this->points==NULL) || (this->nPoints==0))
  {
    NLOG(6, "points is %s, nPoints is %zu", this->points ? "VALID" : "NULL",
         this->nPoints);
    return false;
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VertexArrayNode::setLighting(bool enabled)
{
  osg::StateSet* linestateset = this->geode->getOrCreateStateSet();

  if (enabled)
  {
    linestateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    this->geode->setStateSet(linestateset);

    // cast and receive shadows
    geode->setNodeMask(geode->getNodeMask() | CastsShadowTraversalMask);
    geode->setNodeMask(geode->getNodeMask() | ReceivesShadowTraversalMask);
  }
  else
  {
    linestateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    this->geode->setStateSet(linestateset);

    // neither cast nor receive shadows
    geode->setNodeMask(geode->getNodeMask() & ~CastsShadowTraversalMask);
    geode->setNodeMask(geode->getNodeMask() & ~ReceivesShadowTraversalMask);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VertexArrayNode::setManualUpdate(bool enabled)
{
  this->manual_update = enabled;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::VertexArrayNode::performUpdate()
{
  this->perform_update = true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::VertexArrayNode::takePointsOwnership()
{
  if (myMat == mat)
  {
    return true;   // already owner
  }

  myMat = MatNd_realloc(myMat, mat->m, mat->n);
  MatNd_copy(myMat, mat);

  return setPoints(myMat);
}
