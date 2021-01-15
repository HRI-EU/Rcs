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

#include "FTSensorNode.h"

#include <Rcs_Vec3d.h>
#include <Rcs_Mat3d.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>


/*******************************************************************************
 * Constructor.
 ******************************************************************************/
Rcs::FTSensorNode::FTSensorNode(const RcsSensor* fts, const RcsGraph* graph) :
  ArrowNode(),
  loadCell(fts)
{
  RCHECK(fts->type==RCSSENSOR_LOAD_CELL);
  RCHECK(fts->bodyId!=-1);

  this->mountBdy = &graph->bodies[fts->bodyId];
  this->A_BI = &mountBdy->A_BI;

  setName("FTSensorNode");
  makeDynamic();
}

/*******************************************************************************
 * Destructor.
 ******************************************************************************/
Rcs::FTSensorNode::~FTSensorNode()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
bool Rcs::FTSensorNode::frameCallback()
{
  double pos[3], dir[3];

  Vec3d_transRotate(pos, (double (*)[3]) this->A_BI->rot,
                    this->loadCell->A_SB.org);
  Vec3d_addSelf(pos, this->A_BI->org);

  Vec3d_copy(dir, this->loadCell->rawData->ele);
  Vec3d_transRotateSelf(dir, (double(*)[3])this->loadCell->A_SB.rot);
  Vec3d_transRotateSelf(dir, (double(*)[3]) this->A_BI->rot);
  Vec3d_constMulSelf(dir, 0.01);   // 1kg = 10cm

  double A[3][3];
  Mat3d_fromVec(A, dir, 2);
  setTransformation(pos, A);
  setArrowLength(Vec3d_getLength(dir));

  return false;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
void Rcs::FTSensorNode::setTransformPtr(const HTr* A_BI_)
{
  RCHECK_MSG(A_BI_, "Can't set NULL transform pointer in sensor %s",
             loadCell->name);
  this->A_BI = A_BI_;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
const RcsBody* Rcs::FTSensorNode::getMountBody() const
{
  return this->mountBdy;
}
