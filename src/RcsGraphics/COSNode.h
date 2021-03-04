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

#ifndef RCS_COSNODE_H
#define RCS_COSNODE_H

#include "NodeBase.h"

#include <Rcs_HTr.h>



namespace Rcs
{

/*!
 * \ingroup RcsGraphics
 */
class COSNode: public NodeBase
{

public:

  typedef enum
  {
    RotMat,
    Euler,
    None

  } AngularMode;

  /*! \brief Instantiates a coordinate system node. The axes are colored
   *         in red (x-axis), green (y-axis) and blue (z-axis). The
   *         default axis length is 1.0m. The scale factor scales the
   *         frame as a whole. The arguments lengthX, lengthY and lengthZ
   *         determine the ratio of the axis length to the length of the
   *         arrow tip.
   */
  COSNode(float scale = 1.0f, float lengthX = 0.9, float lengthY = 0.9,
          float lengthZ = 0.9);

  /*! \brief See above. Additionally the frame will be positioned at
   *         vector pos (must therefore point to 3 elements). The pointer
   *         will internally be memorized and the coordinate system
   *         position will be updated upon each frame callback. Please make
   *         sure that the pointer remains valid, otherwise undefined
   *         behavior will result.
   */
  COSNode(const double* pos, float scale = 1.0f, float lengthX = 0.9,
          float lengthY = 0.9, float lengthZ = 0.9);

  /*! \brief See above. Additionally the frame will be oriented according
   *         to rotation rot. If the AngularMode is set to RotMat, it must
   *         point to 9 elements: The rotation matrix is interpreted in
   *         row major form, which is a rotation from world into coordinate
   *         system frame. If the AngularMode is Euler, the pointer is
   *         interpreted as three Euler angles (xyz order). The pointers to
   *         pos and rot will internally be memorized and the coordinate
   *         system position will be updated upon each frame-callback.
   *         Please make sure that the pointer remains valid, otherwise
   *         undefined behavior will result.
   */
  COSNode(const double* pos, const double* rot, float scale = 1.0f,
          AngularMode md = RotMat, float lengthX = 0.9,
          float lengthY = 0.9, float lengthZ = 0.9);

  /*! \brief See above. Additionally the frame will be oriented according
   *         to the transform. The pointers to
   *         pos and rot will internally be memorized and the coordinate
   *         system position will be updated upon each frame-callback.
   *         Please make sure that the pointer remains valid, otherwise
   *         undefined behavior will result.
   */
  COSNode(const HTr* A_BI, float scale = 1.0f, float lengthX = 0.9,
          float lengthY = 0.9, float lengthZ = 0.9);

  /*! \brief Returns the angular description of the frame, see enum AngularMode.
   */
  AngularMode getAngularMode() const;

protected:

  virtual bool frameCallback();
  void init(float scale, float lengthX, float lengthY, float lengthZ);
  AngularMode angMode;
};

}   // namespace Rcs

#endif // RCS_COSNODE_H
