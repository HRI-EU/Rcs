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

#include "TaskSphericalForce1D.h"
#include "TaskFactory.h"
#include "Rcs_typedef.h"
#include "Rcs_macros.h"
#include "Rcs_parser.h"
#include "Rcs_utils.h"
#include "Rcs_VecNd.h"
#include "Rcs_kinematics.h"



static Rcs::TaskFactoryRegistrar<Rcs::TaskSphericalForce1D> registrar1("ForceSphR");
static Rcs::TaskFactoryRegistrar<Rcs::TaskSphericalForce1D> registrar2("ForceSphT");
static Rcs::TaskFactoryRegistrar<Rcs::TaskSphericalForce1D> registrar3("ForceSphP");



/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskSphericalForce1D::TaskSphericalForce1D(const std::string& className,
                                                xmlNode* node,
                                                RcsGraph* _graph,
                                                int dim):
  TaskSpherical1D(className, node, _graph, dim),
  ft_curr_temp(0.0), ft_des_temp(0.), force_feedback(true)
{
  if (className=="ForceSphR")
  {
    resetParameter(Parameters(-10.0, 10.0, 1.0, "Radius Force [N]"));
    this->index = 0;
  }
  else if (className=="ForceSphT")
  {
    resetParameter(Parameters(-4.0*M_PI, 4.0*M_PI, 180.0/M_PI, "Theta Force"));
    this->index = 1;
  }
  else if (className=="ForceSphP")
  {
    resetParameter(Parameters(-4.0*M_PI, 4.0*M_PI, 180.0/M_PI, "Phi Force"));
    this->index = 2;
  }

  getXMLNodePropertyBoolString(node, "forceFeedback", &this->force_feedback);
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskSphericalForce1D::TaskSphericalForce1D(const TaskSphericalForce1D& copyFromMe, RcsGraph* newGraph):
  TaskSpherical1D(copyFromMe, newGraph),
  ft_curr_temp(copyFromMe.ft_curr_temp),
  ft_des_temp(copyFromMe.ft_des_temp),
  force_feedback(copyFromMe.force_feedback)
{
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskSphericalForce1D::~TaskSphericalForce1D()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskSphericalForce1D* Rcs::TaskSphericalForce1D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskSphericalForce1D(*this, newGraph);
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSphericalForce1D::computeX(double* x_res) const
{
  x_res[0] = this->ft_curr_temp;
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSphericalForce1D::computeAX(double* a_res,
                                          double* integral_x,
                                          const double* x_des,
                                          const double* x_dot_des,
                                          const double* x_ddot_des,
                                          const double* S_des,
                                          const double a_des,
                                          const double kp,
                                          const double kd,
                                          const double ki) const
{
  a_res[0] = 0.;
  const_cast<TaskSphericalForce1D*>(this)->ft_des_temp = x_des[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSphericalForce1D::computeAF(double* ft_res,
                                          double* ft_int,
                                          const double* ft_des,
                                          const double* selection,
                                          const double* ft_task,
                                          const double a_des,
                                          const double kp,
                                          const double ki) const
{
  const_cast<double*>(ft_des)[0] = this->ft_des_temp;
  const_cast<double*>(ft_task)[0] = this->ft_curr_temp;
  const_cast<double*>(selection)[0] = 0.;

  if (this->force_feedback == true)
  {
    Rcs::TaskSpherical1D::computeAF(ft_res, ft_int, ft_des, selection, ft_task, a_des, kp, ki);
  }
  else
  {
    Rcs::TaskSpherical1D::computeAF(ft_res, ft_int, ft_des, selection, NULL, a_des, kp, ki);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSphericalForce1D::forceTrafo(double* ft_task) const
{
  Rcs::TaskSpherical1D::forceTrafo(ft_task);
  const_cast<TaskSphericalForce1D*>(this)->ft_curr_temp = ft_task[0];
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskSphericalForce1D::toXMLBody(FILE* out) const
{
  TaskSpherical1D::toXMLBody(out);
  fprintf(out, " forceFeedback=\"%s\"", this->force_feedback ? "true" : "false");
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskSphericalForce1D::isValid(xmlNode* node, const RcsGraph* graph)
{
  std::vector<std::string> classNameVec;
  classNameVec.push_back(std::string("ForceSphR"));
  classNameVec.push_back(std::string("ForceSphT"));
  classNameVec.push_back(std::string("ForceSphP"));

  bool success = Rcs::Task::isValid(node, graph, classNameVec);

  return success;
}
