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

#ifndef TARGETSETTER_H
#define TARGETSETTER_H

#include "RigidBodyTracker.h"

#include <osgText/Text>
#include <osg/Switch>
#include <osgGA/GUIEventHandler>
#include <osgManipulator/CommandManager>



namespace Rcs
{

/*!
 * \ingroup RcsGraphics
 * \brief OSG node for manipulating bodies
 *
 * Usage:
 * \code
 * TargetSetter *ts = new TargetSetter(x0);
 * viewer->addNode(ts);
 * viewer->addEventHandler(ts->getHandler());
 * \endcode
 */
class TargetSetter: public osg::Switch
{
  friend class TargetHandler;

public:

  /*!
   *  \brief Constructor for rotation matrix tracking.
   */
  TargetSetter(double posPtr[3], double rmPtr[3][3], double size=0.25,
               bool withSphericalTracker=true);

  /*!
   *  \brief Constructor for Euler angles tracking.
   */
  TargetSetter(double posPtr[3], double angPtr[3], double size=0.25,
               bool withSphericalTracker=true);

  /*!
   *  \brief Virtual destructor to allow inheriting from this class.
   */
  virtual ~TargetSetter();

  /*!
   *  \brief returns the GUIEventHandler, which is responsible for dragging
   *         the target node
   */
  osgGA::GUIEventHandler* getHandler() const;

  /*!
   * \brief Specifies a reference frame, for example for bodies with
   *        rigid_body_joints that have parents
   */
  void setReferenceFrame(double pos[3], double rot[3][3]);


protected:

  void updateValues(const osg::Matrixd& transform);
  void updateText();
  void initGraphics();
  osg::Matrixd getTransform() const;

  double* posPtr;
  double* angPtr;
  double* rmPtr;
  double scale;
  bool withSphericalTracker;
  char textBuf[256];
  osg::ref_ptr<osgGA::GUIEventHandler> pickHandler;
  osg::ref_ptr<RigidBodyTracker> dragger;
  osg::ref_ptr<osgManipulator::CommandManager> cmdMgr;
  osg::ref_ptr<osgText::Text> _coord_text;
  osg::ref_ptr<osg::PositionAttitudeTransform> refFrame;
};

}   // namespace Rcs

#endif /* TARGETSETTER_H */
