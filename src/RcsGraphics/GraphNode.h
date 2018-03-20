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

#ifndef RCS_GRAPHNODE_H
#define RCS_GRAPHNODE_H

#include "BodyNode.h"

#include <osgGA/GUIEventHandler>

#include <pthread.h>

namespace Rcs
{

/*!
 * \ingroup RcsGraphics
 */
class GraphNode: public osg::PositionAttitudeTransform
{
  friend class GraphNodeEventHandler;

public:

  GraphNode(const RcsGraph* graph, bool resizeable=false,
            bool automaticallyAddTargetSetters=true);
  virtual ~GraphNode();
  void toggleGraphicsModel();
  void toggleCollisionModel();
  void togglePhysicsModel();
  void toggleReferenceFrames();
  void toggleDebugInformation();
  void toggleWireframe();
  void toggleGhostMode();
  void setGhostMode(bool enabled, const std::string& matname="");
  void showWireframe(bool enabled);
  void show();
  void hide();
  void toggle();
  const RcsGraph* getGraphPtr() const;

  /*! \brief Removes a BodyNode by pointer from the GraphNode.
   *
   *  \param[in] body  Pointer to the body to be removed. If it is NULL, the
   *                   function returns false. If no BodyNode for this pointer
   *                   is found, the function also returns false. If a node for
   *                   this body is found, it will be removed and the function
   *                   returns true.
   *  \param[in] mtx   Mutex around the viewer's frame() call. This must be
   *                   given in order to avoid crashes when the viewer's
   *                   frame() function is called from a different thread.
   * \return true for success, false otherwise.
   */
  bool removeBodyNode(const RcsBody* body, pthread_mutex_t* mtx=NULL);

  /*! \brief Removes a BodyNode by name from the GraphNode.
   *
   *  \param[in] body  Name of the body to be removed. If it is NULL, the
   *                   function returns false. If no BodyNode for this name
   *                   is found, the function also returns false. If a node for
   *                   this body is found, it will be removed and the function
   *                   returns true.
   *  \param[in] mtx   Mutex around the viewer's frame() call. This must be
   *                   given in order to avoid crashes when the viewer's
   *                   frame() function is called from a different thread.
   * \return true for success, false otherwise.
   */
  bool removeBodyNode(const char* body, pthread_mutex_t* mtx=NULL);

  bool hideBodyNode(const RcsBody* body);
  bool hideBodyNode(const char* body);
  bool hideSubGraph(const char* body);
  bool hideSubGraph(const RcsBody* body);

  /*! \brief Returns a pointer to the body node with the indicated name. If
   *         several bodies have the same name, the closest one to the root
   *         node will be taken. If no matching body is found, NULL will be
   *         returned. If name is NULL, also NULL is returned.
   *
   *  \param[in] name  Name of the body node.
   *  \return Pointer to desired body node or NULL.
   */
  BodyNode* getBodyNode(const char* name);

  /*! \brief Returns a pointer to the body node with the RcsBody pointer. If
   *         no matching body is found, NULL will be returned. If name is NULL,
   *         also NULL is returned.
   *
   *  \param[in] body  Pointer to desired body
   *  \return Pointer to desired body node or NULL.
   */
  BodyNode* getBodyNode(const RcsBody* body);

  /*! \brief Attaches the BodyNode to the transformation A_BI. It must be a
   *         valid transformation. If the body has no BodyNode, the function
   *         does nothing.
   *
   *  \param[in] body  Pointer to body
   *  \param[in] A_BI  Transformation from World (I) frame to body frame.
   *  \return Pointer to transformation to be tracked.
   */
  void setBodyTransformPtr(const RcsBody* body, const HTr* A_BI);



protected:
  virtual bool callback(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& aa);

  void addNode(osg::Node* child);

  const RcsGraph* _graph;
  bool _wireframe;
  bool _ghost_mode;
  osg::ref_ptr<osg::Switch> _switch;
  osg::ref_ptr<osgGA::GUIEventHandler> frameHandler;
};

}   // namespace Rcs

#endif // RCS_GRAPHNODE_H
