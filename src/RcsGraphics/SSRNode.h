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

#ifndef RCS_SSRNODE_H
#define RCS_SSRNODE_H

#include "NodeBase.h"

#include <Rcs_mesh.h>

#include <osg/Geometry>



namespace Rcs
{


class SSRGeometry : public osg::Geometry
{
public:
  SSRGeometry(const double extents[3], unsigned int nSegments=16);
  void update(const double extents[3]);

private:
  RcsMeshData* mesh;
  double extents[3];
  unsigned int nSeg;
  virtual ~SSRGeometry();
  SSRGeometry& operator=(const SSRGeometry&);
  SSRGeometry(const SSRGeometry&);
};

class SSRNode: public NodeBase
{

public:
  SSRNode(const double center[3],
          double A_KI[3][3],
          const double extent[2],
          const double radius,
          bool resizeable=false);

private:
  SSRNode& operator=(const SSRNode&);
  SSRNode(const SSRNode&);
};

}   // namespace Rcs

#endif // RCS_SSRNODE_H
