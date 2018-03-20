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

#ifndef RCS_BULLETDEBUGDRAWER_H
#define RCS_BULLETDEBUGDRAWER_H

#include <VertexArrayNode.h>

#include <LinearMath/btIDebugDraw.h>


namespace Rcs
{

class BulletDebugDrawer : public btIDebugDraw, public VertexArrayNode
{
public:

  BulletDebugDrawer();
  virtual ~BulletDebugDrawer();

  virtual void clear();
  virtual void apply();

  /**
   * @name VirtualInterface
   *
   * Virtual interface of the btIDebugDraw class, must be implemented here
   */

  ///@{

  virtual void drawLine(const btVector3& from,
                        const btVector3& to,
                        const btVector3& color);

  virtual void drawContactPoint(const btVector3& PointOnB,
                                const btVector3& normalOnB,
                                btScalar distance,
                                int lifeTime,
                                const btVector3& color);

  virtual void reportErrorWarning(const char* warningString);

  virtual void draw3dText(const btVector3& location,
                          const char* textString);

  virtual void setDebugMode(int debugMode);

  virtual int getDebugMode() const;

  ///@}


private:

  int debugMode;
};

}   // namespace Rcs

#endif   // RCS_BULLETDEBUGDRAWER_H
