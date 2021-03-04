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

#ifndef RCS_PHYSICSNODE_H
#define RCS_PHYSICSNODE_H

#include <PhysicsBase.h>
#include <GraphNode.h>
#include <NodeBase.h>


namespace Rcs
{

class PhysicsNode : public NodeBase
{

public:

  PhysicsNode(PhysicsBase* sim, bool resizeable=false);
  virtual ~PhysicsNode();
  void setPhysicsTransform(bool enable);
  void setModelTransform(bool enable);
  bool setGhostMode(const std::string& bodyName, const std::string& matname="");
  void setDisplayMode(int mode);
  int getDisplayMode() const;
  const char* getDisplayModeStr() const;
  void addBodyNode(const RcsBody* body);
  bool removeBodyNode(const char* body);
  bool setDebugDrawer(bool enable);

  /*! \brief Adds a wireframe box to show the considered bounding box for the
   *         simulation. This is specific for Bullet and will do nothing if
   *         the PhysicsBase is not a BullerSimulation.
   */
  void showWorldBoundingBox();

  /*! \brief Adds a small sphere to the COM location of each simulated rigid
   *         body. This is specific for Bullet and will do nothing if
   *         the PhysicsBase is not a BullerSimulation.
   */
  void showBodyCOMs();

protected:

  virtual bool eventCallback(const osgGA::GUIEventAdapter& ea,
                             osgGA::GUIActionAdapter& aa);

  void updateTransformPointers();

public:
  GraphNode* modelNd;
  GraphNode* physicsNd;
  PhysicsBase* sim; ///> Physics simulation
  int displayMode;
  bool resizeable;
};

}   // namespace Rcs

#endif // RCS_PHYSICSNODE_H
