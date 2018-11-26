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
 * \brief Node to display an RcsGraph data structure.
 *
 *        A number of keys are associatde with a function if pressed in the
 *        viewer window:
 *        - Pressing key r will toggle the reference frames
 *        - Pressing key c will toggle the collision model
 *        - Pressing key g will toggle the graphics model
 *        - Pressing key P will toggle the physics model
 *        - Pressing key D will toggle debug information
 *        - Pressing key G will toggle the ghost mode
 *        - Pressing key i will print out the 3d coordinates under the mouse
 *          to the console
 *        - Pressing key I will print out the 3d coordinates and detailed
 *          information about the RcsBody under the mouse to the console
 */
class GraphNode: public osg::PositionAttitudeTransform
{
  friend class GraphNodeEventHandler;

public:

  /*! \brief Constructs an empty GraphNode. To complete, init() needs to be
   *         called.
   */
  GraphNode();

  /*! \brief Constructs a GraphNode from a given RcsGraph data structure.
   *
   *  \param[in] graph            RcsGraph the node will be built from
   *  \param[in] resizeable       If true, the nodes will reflect the changed
   *                              of the RcsShapes dynamically. This will lead
   *                              to a bit less efficiency, therefore the flag
   *                              is set to false by default.
   *  \param[in] addTargetSetters If true, all RcsBodies with rigid body degrees
   *                              of freedom will get a dragger node. It can be
   *                              visualized by pressing the TAB key.
   */
  GraphNode(const RcsGraph* graph, bool resizeable=false,
            bool addTargetSetters=true);

  /*! \brief Removes event handlers
   */
  virtual ~GraphNode();

  /*! \brief Adds all shapes and starts the event callback for updating.
   *
   *  \return true for success, false for failure:
   *          - graph is NULL
   *          - Class already initialized
   */
  bool init(const RcsGraph* graph, bool resizeable, bool addTargetSetters);

  /*! \brief Toggles the visibility of the graphics model of all BodyNodes
   */
  void toggleGraphicsModel();

  /*! \brief Toggles the visibility of the collision model of all BodyNodes
   */
  void toggleCollisionModel();

  /*! \brief Toggles the visibility of the physics model of all BodyNodes
   */
  void togglePhysicsModel();

  /*! \brief Toggles the visibility of the reference frames of all BodyNodes
   */
  void toggleReferenceFrames();

  /*! \brief Toggles the visibility of the debug information of all BodyNodes
   */
  void toggleDebugInformation();

  /*! \brief Toggles the wireframe display of all BodyNodes
   */
  void toggleWireframe();

  /*! \brief Toggles the transparency (ghost mode) of all BodyNodes
   */
  void toggleGhostMode();
  void displayCollisionModel(bool visibility=true);
  void displayGraphicsModel(bool visibility=true);
  void displayPhysicsModel(bool visibility=true);
  void displayReferenceFrames(bool visibility=true);

  /*! \brief Returns true if the overall node is visible, false otherwise.
   */
  bool isVisible() const;

  /*! \brief Returns true if the collision model is visible, false otherwise.
   */
  bool collisionModelVisible() const;

  /*! \brief Returns true if the graphics model is visible, false otherwise.
   */
  bool graphicsModelVisible() const;

  /*! \brief Returns true if the physics model is visible, false otherwise.
   */
  bool physicsModelVisible() const;

  /*! \brief Returns true if the debug information is visible, false otherwise.
   */
  bool debugInformationVisible() const;

  /*! \brief Returns true if the reference frames are visible, false otherwise.
   */
  bool referenceFramesVisible() const;

  /*! \brief Returns true if the models are displayed as wireframes,
   *         false otherwise.
   */
  bool getWireframe() const;

  /*! \brief Returns true if the models are displayed in the transparent ghost
   *         mode, false otherwise.
   */
  bool getGhostMode() const;
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

  /*! \brief Returns a vector with all BodyNodes that have been added.
   *
   *  \return Vector of pointers to all BodyNodes of the GraphNode.
   */
  std::vector<const BodyNode*> getBodyNodes() const;

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

  osg::ref_ptr<osg::Switch> switchNode;
  osg::ref_ptr<osgGA::GUIEventHandler> frameHandler;

private:

  const RcsGraph* graph;
  bool wireframe;
  bool ghostMode;
};

}   // namespace Rcs

#endif // RCS_GRAPHNODE_H
