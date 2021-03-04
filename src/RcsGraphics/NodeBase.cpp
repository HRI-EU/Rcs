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

#include "NodeBase.h"
#include "Rcs_graphicsUtils.h"

#include <Rcs_macros.h>
#include <Rcs_math.h>

#include <osg/PolygonMode>



/*******************************************************************************
 * Update callback: Handle some keys etc.
 ******************************************************************************/
namespace
{
class NodeBaseEventHandler : public osgGA::GUIEventHandler
{
public:
  NodeBaseEventHandler(Rcs::NodeBase* nd) : _nd(nd)
  {
  }
  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter& aa)
  {

    switch (ea.getEventType())
    {

      case (osgGA::GUIEventAdapter::FRAME):
      {
        if (_nd->getPosPtr())
        {
          _nd->setPosition(_nd->getPosPtr());
        }

        if (_nd->getRotMatPtr())
        {
          _nd->setRotation((double (*)[3]) _nd->getRotMatPtr());
        }

        _nd->frameCallback();

        break;
      }

      default:
        break;

    }   // switch(...)

    _nd->eventCallback(ea, aa);

    return false;
  }


protected:
  virtual ~NodeBaseEventHandler() {}
  Rcs::NodeBase* _nd;
};

}   // namespace Rcs



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::NodeBase::NodeBase() : showWireframe(false), posPtr(NULL),
  rmPtr(NULL), isDynamic(false)
{
  setName("NodeBase");
  this->pat = new osg::PositionAttitudeTransform();

  // We overwrite the addChild() method, therefore we explicitely call the
  // base classes method in the constructor.
  osg::Switch::addChild(this->pat.get());
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::NodeBase::frameCallback()
{
  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::NodeBase::eventCallback(const osgGA::GUIEventAdapter& ea,
                                  osgGA::GUIActionAdapter& aa)
{
  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::show()
{
  setAllChildrenOn();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::hide()
{
  setAllChildrenOff();
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::NodeBase::toggle()
{
  bool visible = isVisible();

  if (visible)
  {
    hide();
  }
  else
  {
    show();
  }

  return !visible;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setPosition(double x, double y, double z)
{
  setPosition(osg::Vec3(x, y, z));
}
/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::NodeBase::addChild(osg::Node* child)
{
  return pat->addChild(child);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setPosition(const double x[3])
{
  if (x != NULL)
  {
    setPosition(osg::Vec3(x[0], x[1], x[2]));
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setPosition(osg::Vec3 pos)
{
  if ((!Math_isFinite(pos[0])) || (!Math_isFinite(pos[1])) ||
      (!Math_isFinite(pos[2])))
  {
    RLOG(4, "Invalid position vector: %f %f %f", pos[0], pos[1], pos[2]);
    return;
  }

  this->pat->setPosition(pos);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setRotation(double A_KI[3][3])
{
  if (!A_KI)
  {
    return;
  }

  bool isValid = true;

  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)

      if (!Math_isFinite(A_KI[i][j]))
      {
        isValid = false;
      }

  if (!isValid)
  {
    REXEC(4)
    {
      RLOG(4, "[%s] Invalid rotation matrix:", getName().c_str());
      Mat3d_fprint(stderr, A_KI);
    }
    return;
  }

  osg::Quat qA;
  qA.set(osg::Matrix(A_KI[0][0], A_KI[0][1], A_KI[0][2], 0.0,
                     A_KI[1][0], A_KI[1][1], A_KI[1][2], 0.0,
                     A_KI[2][0], A_KI[2][1], A_KI[2][2], 0.0,
                     0.0,        0.0,        0.0,        1.0));

  this->pat->setAttitude(qA);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setRotation(double eulerABC[3])
{
  if (eulerABC)
  {
    double A[3][3];
    Mat3d_fromEulerAngles(A, eulerABC);
    setRotation(A);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setRotation(osg::Quat att)
{
  this->pat->setAttitude(att);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setTransformation(const HTr* A_BI)
{
  setTransformation(A_BI->org, (double (*)[3]) A_BI->rot);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setTransformation(const double x[3], double A_KI[3][3])
{
  setPosition(x);
  setRotation(A_KI);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setWireframe(bool visible)
{
  this->showWireframe = visible;

  osg::ref_ptr<osg::StateSet> pStateSet = getOrCreateStateSet();

  if (this->showWireframe == true)
  {
    pStateSet->setAttribute(new osg::PolygonMode
                            (osg::PolygonMode::FRONT_AND_BACK,
                             osg::PolygonMode::LINE));
  }
  else
  {
    pStateSet->setAttribute(new osg::PolygonMode
                            (osg::PolygonMode::FRONT_AND_BACK,
                             osg::PolygonMode::FILL));
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
osg::ref_ptr<osg::PositionAttitudeTransform> Rcs::NodeBase::patPtr() const
{
  return this->pat;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::toggleWireframe()
{
  setWireframe(!this->showWireframe);
}

/*******************************************************************************
 * Updates the position from the posPtr on each frame update.
 ******************************************************************************/
void Rcs::NodeBase::makeDynamic()
{
  if (this->isDynamic == true)
  {
    RLOG(4, "Node is already dynamic - skipping");
    return;
  }

  this->isDynamic = true;
  setEventCallback(new NodeBaseEventHandler(this));
}

/*******************************************************************************
 * Updates the position from the pointer to pos on each frame update.
 ******************************************************************************/
void Rcs::NodeBase::makeDynamic(const double pos[3])
{
  setPosPtr(pos);
  makeDynamic();
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::makeDynamic(double A_KI[3][3])
{
  setRotMatPtr((const double*) A_KI);
  makeDynamic();
}

/*******************************************************************************
 * Updates the position and rotation matrix from the posPtr and rmPtr on each
 * frame update.
 ******************************************************************************/
void Rcs::NodeBase::makeDynamic(const double pos[3], double A_KI[3][3])
{
  setPosPtr(pos);
  setRotMatPtr((const double*) A_KI);
  makeDynamic();
}

/*******************************************************************************
 * Returns the pointer to the position.
 ******************************************************************************/
const double* Rcs::NodeBase::getPosPtr() const
{
  return this->posPtr;
}

/*******************************************************************************
 * Returns the pointer to the rotation matrix.
 ******************************************************************************/
const double* Rcs::NodeBase::getRotMatPtr() const
{
  return this->rmPtr;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setPosPtr(const double* pos)
{
  this->posPtr = pos;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::NodeBase::setRotMatPtr(const double* rot)
{
  this->rmPtr = rot;
}

/*******************************************************************************
 * Sets the material properties
 ******************************************************************************/
void Rcs::NodeBase::setMaterial(const char* material, double alpha)
{
  if (material==NULL)
  {
    return;
  }

  setMaterial(std::string(material), alpha);
}

/*******************************************************************************
 * Sets the material properties.
 ******************************************************************************/
void Rcs::NodeBase::setMaterial(const std::string& material, double alpha)
{
  setNodeMaterial(material, pat.get(), alpha);
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::NodeBase::isVisible() const
{
  return getValue(0);
}
