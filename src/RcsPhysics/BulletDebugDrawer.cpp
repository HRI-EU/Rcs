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

#include "BulletDebugDrawer.h"

#include <Rcs_macros.h>

#include <LinearMath/btTransform.h>



/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletDebugDrawer::BulletDebugDrawer():
  VertexArrayNode(osg::PrimitiveSet::LINES, "WHITE"), debugMode(0)
{
  setManualUpdate(true);
  setPointSize(1.0);
  setLighting(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::BulletDebugDrawer::~BulletDebugDrawer()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletDebugDrawer::drawLine(const btVector3& from,
                                      const btVector3& to,
                                      const btVector3& color)
{
  pointsArray->push_back(osg::Vec3(from.x(), from.y(), from.z()));
  pointsArray->push_back(osg::Vec3(to.x(), to.y(), to.z()));
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletDebugDrawer::drawContactPoint(const btVector3& pointOnB,
                                              const btVector3& normalOnB,
                                              btScalar distance,
                                              int lifeTime,
                                              const btVector3& color)
{
  drawLine(pointOnB, pointOnB + normalOnB*distance, color);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletDebugDrawer::reportErrorWarning(const char* warningString)
{
  RMSG("Warning: %s", warningString ? warningString : "NULL");
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletDebugDrawer::draw3dText(const btVector3& location,
                                        const char* textString)
{
  RMSG("%s", textString ? textString : "NULL");
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletDebugDrawer::setDebugMode(int debugMode)
{
  this->debugMode = debugMode;
}

/*******************************************************************************
 *
 ******************************************************************************/
int Rcs::BulletDebugDrawer::getDebugMode() const
{
  return this->debugMode;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::BulletDebugDrawer::clear()
{
  pointsArray->clear();
}

/*******************************************************************************
 * This must happen outside the frame() call of the viewer, otherwise we'll
 * get into trouble with parallel access to the vertex data. From the simulator
 * side, the function is called within the simulate mutex already.
 ******************************************************************************/
void Rcs::BulletDebugDrawer::apply()
{
  osg::DrawArrays* ps = ((osg::DrawArrays*)(geometry)->getPrimitiveSet(0));
  ps->setCount(pointsArray->size());
  geometry->setVertexArray(pointsArray.get());
  geometry->setPrimitiveSet(0, ps);
}
