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

#include "TransformTrace.h"

#include <Rcs_macros.h>
#include <Rcs_math.h>

#include <osgGA/GUIEventHandler>
#include <osg/Geometry>
#include <osg/Geode>



using namespace Rcs;



/*******************************************************************************

  \brief Update callback: Handle some keys etc.

*******************************************************************************/

class TransformTraceEventHandler : public osgGA::GUIEventHandler
{
public:

  TransformTraceEventHandler(const HTr* A, unsigned int nPts)
  {
    _A = A;
    RCHECK(_A);
    _points = new osg::Geometry;
    _nPts = nPts;
    _pos = _A->org;

    // Create an array with nPts points, all initialized with the origin of A
    _vertices = new osg::Vec3Array(_nPts);
    for (unsigned int i=0; i<_nPts; i++)
    {
      _vertices->push_back(osg::Vec3(A->org[0], A->org[1], A->org[2]));
    }

    // Set the line color to yellow
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
    _points->setColorArray(colors);
    _points->setColorBinding(osg::Geometry::BIND_OVERALL);
    _points->addPrimitiveSet
    (new osg::DrawArrays(osg::PrimitiveSet:: POINTS, 0, _nPts));

    // Set the vertex array to the geometry
    osg::DrawArrays* ps = ((osg::DrawArrays*)(_points)->getPrimitiveSet(0));
    ps->setCount(_nPts);
    _points->setVertexArray(_vertices.get());
    _points->setPrimitiveSet(0, ps);

    // Switch of light for the points
    _points->getOrCreateStateSet()->setMode(GL_LIGHTING,
                                            osg::StateAttribute::OFF);

    geode = new osg::Geode();
    geode->addDrawable(_points.get());
  }



  TransformTraceEventHandler(const double* pos, const double* rm,
                             unsigned int nPts)
  {
    _pos = pos;
    RCHECK(_pos);
    _rm = rm;
    _points = new osg::Geometry;
    _nPts = nPts;

    // Create an array with nPts points, all initialized with the origin of A
    _vertices = new osg::Vec3Array(_nPts);
    for (unsigned int i=0; i<_nPts; i++)
    {
      _vertices->push_back(osg::Vec3(_pos[0], _pos[1], _pos[2]));
    }

    // Set the line color to yellow
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
    _points->setColorArray(colors);
    _points->setColorBinding(osg::Geometry::BIND_OVERALL);
    _points->addPrimitiveSet
    (new osg::DrawArrays(osg::PrimitiveSet:: POINTS, 0, _nPts));

    // Set the vertex array to the geometry
    osg::DrawArrays* ps =
      ((osg::DrawArrays*)(_points)->getPrimitiveSet(0));
    ps->setCount(_nPts);
    _points->setVertexArray(_vertices.get());
    _points->setPrimitiveSet(0, ps);

    // Switch of light for the points
    _points->getOrCreateStateSet()->setMode(GL_LIGHTING,
                                            osg::StateAttribute::OFF);

    geode = new osg::Geode();
    geode->addDrawable(_points.get());
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter& aa)
  {

    switch (ea.getEventType())
    {

      case (osgGA::GUIEventAdapter::FRAME):
      {
        _vertices->insert(_vertices->begin(),
                          osg::Vec3(_pos[0], _pos[1], _pos[2]));
        _vertices->resize(_nPts);

        osg::DrawArrays* ps = ((osg::DrawArrays*)(_points)->getPrimitiveSet(0));
        ps->setCount(_nPts);
        _points->setVertexArray(_vertices.get());
        _points->setPrimitiveSet(0, ps);
        break;
      }

      default:
        break;

    }   // switch(...)

    return false;
  }

public:

  osg::ref_ptr<osg::Geode> geode;

protected:

  virtual ~TransformTraceEventHandler()
  {
  }
  osg::ref_ptr<osg::Geometry> _points;
  osg::ref_ptr<osg::Vec3Array> _vertices;
  const HTr* _A;
  const double* _pos;
  const double* _rm;
  unsigned int _nPts;
};



/*******************************************************************************

  \brief

*******************************************************************************/

TransformTrace::TransformTrace(const double* pos, unsigned int nPts) :
  osg::Switch()
{
  setName("TransformTrace");
  TransformTraceEventHandler* cHandler =
    new TransformTraceEventHandler(pos, NULL, nPts);
  addChild(cHandler->geode.get());
  setEventCallback(cHandler);
}



/*******************************************************************************

  \brief

*******************************************************************************/

void TransformTrace::show()
{
  setAllChildrenOn();
}



/*******************************************************************************

  \brief

*******************************************************************************/

void TransformTrace::hide()
{
  setAllChildrenOff();
}



/*******************************************************************************

  \brief

*******************************************************************************/

void TransformTrace::toggle()
{
  bool visible = getValue(0);

  if (visible)
  {
    setAllChildrenOff();
  }
  else
  {
    setAllChildrenOn();
  }
}
