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

#include "TaskSleeve.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_stlParser.h"
#include "Rcs_math.h"
#include "Rcs_kinematics.h"
#include "Rcs_geometry.h"
#include "Rcs_body.h"

#include <algorithm>



// register task at the task factory
static Rcs::TaskFactoryRegistrar<Rcs::TaskSleeve> registrar("Sleeve");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskSleeve::TaskSleeve(const std::string& className,
                            xmlNode* node,
                            const RcsGraph* _graph,
                            int dim) :
  TaskGenericIK(className, node, _graph, dim), slideBdyId(-1)
{
  if (getClassName()=="Sleeve")
  {
    std::string shldrName = getXMLNodePropertySTLString(node, "shoulder");
    std::string elbowName = getXMLNodePropertySTLString(node, "elbow");
    std::string wristName = getXMLNodePropertySTLString(node, "wrist");
    std::string slideName = getXMLNodePropertySTLString(node, "slider");

    const RcsBody* sh = RcsGraph_getBodyByName(getGraph(), shldrName.c_str());
    const RcsBody* el = RcsGraph_getBodyByName(getGraph(), elbowName.c_str());
    const RcsBody* wr = RcsGraph_getBodyByName(getGraph(), wristName.c_str());
    const RcsBody* sl = RcsGraph_getBodyByName(getGraph(), slideName.c_str());

    RCHECK(sh&&el&&wr&&sl);

    setEffectorId(wr->id);
    setRefBodyId(el->id);
    setRefFrameId(sh->id);
    slideBdyId = sl->id;

    double guiMax = 1.5, guiMin = -0.5;
    getXMLNodePropertyDouble(node, "guiMax", &guiMax);
    getXMLNodePropertyDouble(node, "guiMin", &guiMin);

    bool hide = false;
    getXMLNodePropertyBoolString(node, "hide", &hide);
    if (hide)
    {
      guiMin = 0.0;
      guiMax = 0.0;
    }

    resetParameter(Parameters(guiMin, guiMax, 1.0, "s [0...1]"));
    addParameter(Parameters(guiMin, guiMax, 1.0, "y [m]"));
    addParameter(Parameters(guiMin, guiMax, 1.0, "z [m]"));
  }

}

/*******************************************************************************
 * Clone function
 ******************************************************************************/
Rcs::TaskSleeve* Rcs::TaskSleeve::clone(const RcsGraph* newGraph) const
{
  TaskSleeve* task = new Rcs::TaskSleeve(*this);
  task->setGraph(newGraph);
  return task;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSleeve::computeX(double* x) const
{
  const RcsBody* sh = getShoulder();
  const RcsBody* sl = getSlider();
  const RcsBody* wr = getWrist();
  const RcsBody* el = getElbow();

  double A[3][3];
  computeFrame(A);

  // Component 0: Slider point projected on shoulder-wrist connection
  double* r_sw = A[0];
  double cp[3];
  Math_sqrDistPointLine(sl->A_BI.org, sh->A_BI.org, r_sw, cp);
  x[0] = Vec3d_distance(sh->A_BI.org, cp)/ Vec3d_distance(sh->A_BI.org, wr->A_BI.org);

  // If the s-coordinate gets slightly negative, we must flip the sign since
  // the distance will remain positive.
  double dir[3];
  Vec3d_sub(dir, cp, sh->A_BI.org);
  double dirSgn = Vec3d_innerProduct(dir, r_sw) < 0.0 ? -1.0 : 1.0;
  x[0] *= dirSgn;

  // Component 1: Slider offset to plane spanned by sh-el-wr
  double* elbowNormal = A[1];
  double r_cps[3];   // Vector from closest point to slider
  Vec3d_sub(r_cps, sl->A_BI.org, cp);
  x[1] = Vec3d_innerProduct(r_cps, elbowNormal);

  // Component 2: Distance to arm surface
  double d1 = RcsBody_distance(sh, sl, NULL, NULL, NULL);
  double d2 = RcsBody_distance(el, sl, NULL, NULL, NULL);
  x[2] = std::min(d1, d2);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSleeve::computeDX(double* dx,
                                const double* x_des,
                                const double* x_curr) const
{
  double x_des_clipped[3];
  Vec3d_copy(x_des_clipped, x_des);

  // We enforce the desired command to be within 0 and 1 to avoid divergence
  // when leaving the "meaningful" model range.
  x_des_clipped[0] = Math_clip(x_des_clipped[0], 0.0, 1.0);

  TaskGenericIK::computeDX(dx, x_des_clipped, x_curr);

  // The distances pointing outwards from the arm surface are always positive
  // regardless if they are up or down from the arm. Our Jacobian is directed
  // however. We mitigate this by scaling the error compensation with the
  // dot product of surface normal and Jacobian direction. This still does not
  // make it correct, however we don't get divergence any more.
  double n[3], A[3][3];
  computeContactNormal(n);
  computeFrame(A);
  dx[2] *= Math_dsign(Vec3d_innerProduct(n, A[2]));
  Vec3d_constMulSelf(dx, 0.1);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSleeve::computeJ(MatNd* jacobian) const
{
  double A[3][3];
  computeFrame(A);
  RcsGraph_worldPointJacobian(getGraph(), getSlider(), NULL, A, jacobian);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSleeve::computeFrame(double A[3][3]) const
{
  const RcsBody* sh = getShoulder();
  const RcsBody* el = getElbow();
  const RcsBody* wr = getWrist();

  // Row 0: Slider point projected on shoulder-wrist connection
  double* r_sw = A[0];
  Vec3d_sub(A[0], wr->A_BI.org, sh->A_BI.org);
  Vec3d_normalizeSelf(r_sw);

  // Row 1: Slider offset to plane spanned by sh-el-wr
  double* elbowNormal = A[1];
  double r_se[3], r_ew[3];
  Vec3d_sub(r_se, el->A_BI.org, sh->A_BI.org);
  Vec3d_sub(r_ew, wr->A_BI.org, el->A_BI.org);
  Vec3d_crossProduct(elbowNormal, r_se, r_ew);
  Vec3d_normalizeSelf(elbowNormal);

  // Row 2: right hand rule completion
  Vec3d_crossProduct(A[2], r_sw, elbowNormal);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSleeve::computeContactNormal(double n[3]) const
{
  double n1[3], n2[3];
  double d1 = RcsBody_distance(getShoulder(), getSlider(), NULL, NULL, n1);
  double d2 = RcsBody_distance(getElbow(), getSlider(), NULL, NULL, n2);
  Vec3d_copy(n, (d1<d2) ? n1 : n2);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSleeve::computeH(MatNd* hessian) const
{
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSleeve::toXMLBody(FILE* out) const
{
  Task::toXMLBody(out);

  fprintf(out, " shoulder=\"%s\"", getShoulder()->name);
  fprintf(out, " elbow=\"%s\"", getElbow()->name);
  fprintf(out, " wrist=\"%s\"", getWrist()->name);
  fprintf(out, " slider=\"%s\"", getSlider()->name);
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsBody* Rcs::TaskSleeve::getShoulder() const
{
  return getRefFrame();
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsBody* Rcs::TaskSleeve::getElbow() const
{
  return getRefBody();
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsBody* Rcs::TaskSleeve::getWrist() const
{
  return getEffector();
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsBody* Rcs::TaskSleeve::getSlider() const
{
  return RCSBODY_BY_ID(getGraph(), slideBdyId);
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskSleeve::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "Sleeve");

  // Check if all bodies exist
  std::string shldrName = getXMLNodePropertySTLString(node, "shoulder");
  std::string elbowName = getXMLNodePropertySTLString(node, "elbow");
  std::string wristName = getXMLNodePropertySTLString(node, "wrist");
  std::string slideName = getXMLNodePropertySTLString(node, "slider");

  const RcsBody* sh = RcsGraph_getBodyByName(graph, shldrName.c_str());
  const RcsBody* el = RcsGraph_getBodyByName(graph, elbowName.c_str());
  const RcsBody* wr = RcsGraph_getBodyByName(graph, wristName.c_str());
  const RcsBody* sl = RcsGraph_getBodyByName(graph, slideName.c_str());

  if ((!sh) || (!el) || (!wr) || (!sl))
  {
    success = false;

    REXEC(3)
    {
      std::string taskName = getXMLNodePropertySTLString(node, "name");
      RMSG("Task \"%s\": One of these bodies for attributes \"shoulder\", "
           "\"elbow\", \"wrist\" or \"slider\" was not found in the graph",
           taskName.c_str());
    }
  }

  return success;
}
