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

#include "TaskGenericEuler3D.h"
#include "TaskFactory.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_parser.h>
#include <Rcs_utils.h>
#include <Rcs_math.h>
#include <Rcs_kinematics.h>
#include <EulerAngles.h>

#include <sstream>

/*
  These functions are used from the parent class TaskEuler3d:

  - computeJ
  - computeH
  - computeTaskCost
  - computeTaskGradient
  - computeAx
*/

static Rcs::TaskFactoryRegistrar<Rcs::TaskGenericEuler3D> registrar("genericEuler");



/*******************************************************************************
 * Convert Rcs RotMatrix to Shoemake \ref HMatrix
 * H is a 4x4 matrix, the last column and row are 1 on the diagonal and 0
 * elsewhere
 ******************************************************************************/
static void RotMatrix2HMatrix(HMatrix H, const double R[3][3])
{
  memset(H, 0, 16*sizeof(double));
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      H[i][j] = R[j][i];
    }
  }
  H[3][3] = 1.0;
}

/*******************************************************************************
 * Convert Shoemake \ref HMatrix to Rcs RotMatrix
 ******************************************************************************/
static void HMatrix2RotMatrix(double R[3][3], HMatrix H)
{
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      R[j][i] = H[i][j];
    }
  }
}

/*******************************************************************************
 * Convert generic Euler Angles to Rcs RotMatrix
 ******************************************************************************/
static void Eul2RotMatrix(double A_KI[3][3], const double ea[3], int eulerOrder)
{
  // Compute current rotation matrix from current euler angles
  EulerAngles inAngs;
  inAngs.x = ea[0];
  inAngs.y = ea[1];
  inAngs.z = ea[2];
  inAngs.order = eulerOrder;

  // Determine the current rotation matrix according to the angular ordering
  HMatrix R;
  Eul_ToHMatrix(inAngs, R);

  // ... and convert it to column major representation
  HMatrix2RotMatrix(A_KI, R);
}

/*******************************************************************************
 * Constructor based on xml parsing
 ******************************************************************************/
Rcs::TaskGenericEuler3D::TaskGenericEuler3D(const std::string& className_,
                                            xmlNode* node,
                                            RcsGraph* _graph,
                                            int dim):
  Rcs::Task(className_, node, _graph, dim)
{
  char eulerOrderChar[64] = "ABCr";

  if (getXMLNodePropertyStringN(node, "eulerOrder", eulerOrderChar, 64) == 0)
  {
    RLOG(1, "Task has no eulerOrder! Using \"%s\"", eulerOrderChar);
  }

  // we reconstruct the Euler order definition from the string
  int EulOrdPara[4] = {0, 0, 0, 0}; // used for EulOrd macro

  // S or s: static frame
  if (eulerOrderChar[3] == 's' || eulerOrderChar[3] == 'S')
  {
    this->eulerOrderVect[3] = EulOrdPara[3] = EulFrmS;
  }
  // R or r: rotating frame
  else if (eulerOrderChar[3] == 'r' || eulerOrderChar[3] == 'R')
  {
    this->eulerOrderVect[3] = EulOrdPara[3] = EulFrmR;
  }
  else
  {
    // error flag
    this->eulerOrderVect[3] = 2;
  }

  // these are "human readable" and will be used on the Rcs side
  for (size_t i = 0; i < 3; i++)
  {
    if (eulerOrderChar[i] == 'A' || eulerOrderChar[i] == 'X')
    {
      this->eulerOrderVect[i] = ShoemakeIdx_X;
    }
    else if (eulerOrderChar[i] == 'B' || eulerOrderChar[i] == 'Y')
    {
      this->eulerOrderVect[i] = ShoemakeIdx_Y;
    }
    else if (eulerOrderChar[i] == 'C' || eulerOrderChar[i] == 'Z')
    {
      this->eulerOrderVect[i] = ShoemakeIdx_Z;
    }
    else
    {
      // error flag
      this->eulerOrderVect[3] = 2;
    }
  }

  // cannot have identical neighboring coords
  if ((this->eulerOrderVect[0] == this->eulerOrderVect[1]) ||
      (this->eulerOrderVect[1] == this->eulerOrderVect[2]))
  {
    // error flag
    this->eulerOrderVect[3] = 2;
  }

  if (this->eulerOrderVect[3] == 2)
  {
    RLOG(1, "Invalid eulerOrder! Using \"ABCr\"");
    this->eulerOrderVect[0] = ShoemakeIdx_X;
    this->eulerOrderVect[1] = ShoemakeIdx_Y;
    this->eulerOrderVect[2] = ShoemakeIdx_Z;
    this->eulerOrderVect[3] = EulOrdPara[3] = EulFrmR;
  }

  // now we determine all the values for the Shoemake EulOrd macro
  // EulOrd(1st/3rd coord, ordered?, repeating?, static/rotating);

  // repeating coordinates (1st and 3rd are the same)?
  if (this->eulerOrderVect[0] == this->eulerOrderVect[2])
  {
    EulOrdPara[2] = EulRepYes;
  }
  else
  {
    EulOrdPara[2] = EulRepNo;
  }

  if (EulOrdPara[3] == EulFrmS) // static
  {
    EulOrdPara[0] = this->eulerOrderVect[0]; // X,Y,Z 1st coord

    // 3rd coord is directly after 2nd coord in the [X,Y,Z,X] order?
    if ((this->eulerOrderVect[1]+1) == this->eulerOrderVect[2] ||
        (this->eulerOrderVect[1]+1-3) == this->eulerOrderVect[2])
    {
      EulOrdPara[1] = EulParEven;
    }
    else
    {
      EulOrdPara[1] = EulParOdd;
    }
  }
  else if (EulOrdPara[3] == EulFrmR) //rotating
  {
    EulOrdPara[0] = this->eulerOrderVect[2]; // X,Y,Z 3rd coord

    // 1st coord is directly after 2nd coord in the [X,Y,Z,X] order?
    if ((this->eulerOrderVect[1]+1) == this->eulerOrderVect[0] ||
        (this->eulerOrderVect[1]+1-3) == this->eulerOrderVect[0])
    {
      EulOrdPara[1] = EulParEven;
    }
    else
    {
      EulOrdPara[1] = EulParOdd;
    }
  }

  // 1D int that serves as Shoemake eulerOrder definition
  this->eulerOrder = EulOrd(EulOrdPara[0], EulOrdPara[1], EulOrdPara[2], EulOrdPara[3]);

  // re-initialize parameters
  clearParameters();
  for (size_t i = 0; i < getDim(); i++)
  {
    std::ostringstream label;
    label << eulerOrderChar[i] << " ";
    if (EulOrdPara[3] == EulFrmR)
    {
      label << "rotating";
    }
    else if (EulOrdPara[3] == EulFrmS)
    {
      label << "static";
    }
    label << " [deg]";
    addParameter(Parameters(-M_PI, M_PI, (180.0/M_PI), label.str()));
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::TaskGenericEuler3D::TaskGenericEuler3D(RcsGraph* graph_,
                                            const char* eulerOrderChar,
                                            const RcsBody* effector,
                                            const RcsBody* refBdy,
                                            const RcsBody* refFrame)
{
  // Populate classname and taskname manually
  this->graph = graph_;
  setClassName("genericEuler");
  setDim(3);

  std::string taskName = std::string("genericEuler");

  if (effector != NULL)
  {
    taskName += std::string(" ");
    taskName += std::string(effector->name);
  }

  if (refBdy != NULL)
  {
    taskName += std::string("-");
    taskName += std::string(refBdy->name);
  }

  setName(taskName);

  setEffectorId(effector ? effector->id : -1);
  setRefBodyId(refBdy ? refBdy->id : -1);
  setRefFrameId(refFrame ? refFrame->id : getRefBodyId());

  // we reconstruct the Euler order definition from the string
  int EulOrdPara[4] = {0, 0, 0, 0}; // used for EulOrd macro

  // S or s: static frame
  if (eulerOrderChar[3] == 's' || eulerOrderChar[3] == 'S')
  {
    this->eulerOrderVect[3] = EulOrdPara[3] = EulFrmS;
  }
  // R or r: rotating frame
  else if (eulerOrderChar[3] == 'r' || eulerOrderChar[3] == 'R')
  {
    this->eulerOrderVect[3] = EulOrdPara[3] = EulFrmR;
  }
  else
  {
    // error flag
    this->eulerOrderVect[3] = 2;
  }

  // these are "human readable" and will be used on the Rcs side
  for (size_t i = 0; i < 3; i++)
  {
    if (eulerOrderChar[i] == 'A' || eulerOrderChar[i] == 'X')
    {
      this->eulerOrderVect[i] = ShoemakeIdx_X;
    }
    else if (eulerOrderChar[i] == 'B' || eulerOrderChar[i] == 'Y')
    {
      this->eulerOrderVect[i] = ShoemakeIdx_Y;
    }
    else if (eulerOrderChar[i] == 'C' || eulerOrderChar[i] == 'Z')
    {
      this->eulerOrderVect[i] = ShoemakeIdx_Z;
    }
    else
    {
      // error flag
      this->eulerOrderVect[3] = 2;
    }
  }

  // cannot have identical neighboring coords
  if ((this->eulerOrderVect[0] == this->eulerOrderVect[1]) ||
      (this->eulerOrderVect[1] == this->eulerOrderVect[2]))
  {
    // error flag
    this->eulerOrderVect[3] = 2;
  }

  if (this->eulerOrderVect[3] == 2)
  {
    RLOG(1, "Invalid eulerOrder! Using \"ABCr\"");
    this->eulerOrderVect[0] = ShoemakeIdx_X;
    this->eulerOrderVect[1] = ShoemakeIdx_Y;
    this->eulerOrderVect[2] = ShoemakeIdx_Z;
    this->eulerOrderVect[3] = EulOrdPara[3] = EulFrmR;
  }

  // now we determine all the values for the Shoemake EulOrd macro
  // EulOrd(1st/3rd coord, ordered?, repeating?, static/rotating);

  // repeating coordinates (1st and 3rd are the same)?
  if (this->eulerOrderVect[0] == this->eulerOrderVect[2])
  {
    EulOrdPara[2] = EulRepYes;
  }
  else
  {
    EulOrdPara[2] = EulRepNo;
  }

  if (EulOrdPara[3] == EulFrmS) // static
  {
    EulOrdPara[0] = this->eulerOrderVect[0]; // X,Y,Z 1st coord

    // 3rd coord is directly after 2nd coord in the [X,Y,Z,X] order?
    if ((this->eulerOrderVect[1]+1) == this->eulerOrderVect[2] ||
        (this->eulerOrderVect[1]+1-3) == this->eulerOrderVect[2])
    {
      EulOrdPara[1] = EulParEven;
    }
    else
    {
      EulOrdPara[1] = EulParOdd;
    }
  }
  else if (EulOrdPara[3] == EulFrmR) //rotating
  {
    EulOrdPara[0] = this->eulerOrderVect[2]; // X,Y,Z 3rd coord

    // 1st coord is directly after 2nd coord in the [X,Y,Z,X] order?
    if ((this->eulerOrderVect[1]+1) == this->eulerOrderVect[0] ||
        (this->eulerOrderVect[1]+1-3) == this->eulerOrderVect[0])
    {
      EulOrdPara[1] = EulParEven;
    }
    else
    {
      EulOrdPara[1] = EulParOdd;
    }
  }

  // 1D int that serves as Shoemake eulerOrder definition
  this->eulerOrder = EulOrd(EulOrdPara[0], EulOrdPara[1], EulOrdPara[2], EulOrdPara[3]);

  // re-initialize parameters
  clearParameters();
  for (size_t i = 0; i < getDim(); i++)
  {
    std::ostringstream label;
    label << eulerOrderChar[i] << " ";
    if (EulOrdPara[3] == EulFrmR)
    {
      label << "rotating";
    }
    else if (EulOrdPara[3] == EulFrmS)
    {
      label << "static";
    }
    label << " [deg]";

    addParameter(Parameters(-M_PI, M_PI, (180.0/M_PI), label.str()));
  }
}

/*******************************************************************************
 * Copy constructor doing deep copying
 ******************************************************************************/
Rcs::TaskGenericEuler3D::TaskGenericEuler3D(const TaskGenericEuler3D& src,
                                            RcsGraph* newGraph):
  Rcs::Task(src, newGraph)
{
  this->eulerOrder = src.eulerOrder;
  memmove(this->eulerOrderVect, src.eulerOrderVect, 4*sizeof(int));
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
Rcs::TaskGenericEuler3D::~TaskGenericEuler3D()
{
}

/*******************************************************************************
 * Returns a clone of the instance.
 ******************************************************************************/
Rcs::TaskGenericEuler3D* Rcs::TaskGenericEuler3D::clone(RcsGraph* newGraph) const
{
  return new Rcs::TaskGenericEuler3D(*this, newGraph);
}


/*******************************************************************************
 * Computes the current value of the task variable: Euler angles
 ******************************************************************************/

void Rcs::TaskGenericEuler3D::computeX(double* x_res) const
{
  computeEulerAngles(x_res, getEffector(), getRefBody(), this->eulerOrder);
}

/*******************************************************************************
 * Angular velocity Jacobian. This is not the geometric Jacobian, since we
 * compute differential gains for the IK in order to get around singularity
 * issues. For this reason, this Jacobian can be considered as "safe" (It is
 * always non-singular), and such differs from the standard formulations (for
 * instance used in the element-wise Kardan Angles Jacobian as implemented
 * here).
 *
 * ref_JR = A_ref-I * (I_JR,ef - I_JR,ref)
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::computeJ(MatNd* jacobian) const
{
  RcsGraph_3dOmegaJacobian(this->graph, getEffector(), getRefBody(),
                           getRefFrame(), jacobian);
}

/*******************************************************************************
 * see RcsGraph_3dOmegaHessian();
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::computeH(MatNd* hessian) const
{
  RcsGraph_3dOmegaHessian(this->graph, getEffector(), getRefBody(),
                          getRefFrame(), hessian);
}

/*******************************************************************************
 * Computes the current value of the task variable: Euler angles
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::computeEulerAngles(double* ea,
                                                 const RcsBody* effector,
                                                 const RcsBody* referenceBody,
                                                 const int eulerOrder)
{
  // Determine the rotation matrix
  double A_curr[3][3];
  computeRelativeRotationMatrix(A_curr, effector, referenceBody);

  // use Shoemake code to convert RotMatrix to EulerAngles
  HMatrix H;
  RotMatrix2HMatrix(H, A_curr);
  EulerAngles outAngs;
  outAngs = Eul_FromHMatrix(H, eulerOrder);

  // convert Shoemake EulerAngles to states
  ea[0] = outAngs.x;
  ea[1] = outAngs.y;
  ea[2] = outAngs.z;
}

/*******************************************************************************
 * Computes the current velocity in task space: Euler angles
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::computeXp(double* eap) const
{
  double omega[3], ea[3], Binv[3][3];
  computeOmega(omega);
  computeX(ea);

  // the velocity now is in omegas, but we want it in Euler differences
  OmegasToEulerRateMat(Binv, ea, this->eulerOrderVect);

  Vec3d_rotate(eap, Binv, omega);
}

/*******************************************************************************
 * Computes the current velocity in task space: Euler velocities
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::computeXpp(double* eapp, const MatNd* qpp) const
{
  Rcs::Task::computeXpp_ik(eapp, qpp);

  double ea[3], eap[3];
  this->computeX(ea);
  this->computeXp(eap);

  double dBeap[3]; // dB/dt ap
  EulerRateToOmegasDerivativeTimesEulerVel(dBeap, ea, eap, this->eulerOrderVect);

  double Binv[3][3];
  OmegasToEulerRateMat(Binv, ea, this->eulerOrderVect);

  Vec3d_subSelf(eapp, dBeap); // (wp - dB/dt ap)
  Vec3d_rotateSelf(eapp, Binv); // H (wp - d(H^-1)/dt ap)
}

/*******************************************************************************
 * Computes the delta in task space for the differential kinematics
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::computeDX(double* dx,
                                        const double* x_des,
                                        const double* x_curr) const
{
  double A_curr[3][3], A_des[3][3];
  Eul2RotMatrix(A_curr, x_curr, this->eulerOrder);
  Eul2RotMatrix(A_des, x_des, this->eulerOrder);

  double angle = Mat3d_diffAngle(A_des, A_curr);

  if (angle<1.0*(M_PI/180.0))   // Less than 1 deg error: Use Euler error
  {
    Mat3d_getEulerError(dx, A_curr, A_des);
  }
  else   // Larger error: Use axis angle error
  {
    Mat3d_getAxisAngle(dx, A_des, A_curr);
    Vec3d_constMulSelf(dx, angle);
  }
}

/*******************************************************************************
 * Computes the velocity error
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::computeDXp(double* delta_om,
                                         const double* eap_des) const
{
  double om_curr[3], ea[3], B[3][3];
  computeOmega(om_curr);
  computeX(ea);
  EulerRateToOmegasMat(B, ea, this->eulerOrderVect);

  double om_des[3];
  Vec3d_rotate(om_des, B, eap_des);
  Vec3d_sub(delta_om, om_des, om_curr);

  //  // HACK
  //  Vec3d_setZero(delta_om);

  //   //eap_des in omega_p
  //   double om_curr[3];
  //
  //   computeOmega(om_curr);
  //   Vec3d_sub(delta_om, eap_des, om_curr);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::computeAF(double* ft_res,
                                        double* ft_int,
                                        const double* ft_des,
                                        const double* selection,
                                        const double* ft_task,
                                        const double a_des,
                                        const double kp,
                                        const double ki) const
{
  Rcs::Task::computeAF(ft_res, ft_int, ft_des, selection, ft_task,
                       a_des, kp, ki);

  double ea[3], Binv[3][3];
  computeX(ea);
  OmegasToEulerRateMat(Binv, ea, this->eulerOrderVect);  //B = H^-1

  Vec3d_transRotateSelf(ft_res, Binv);
}


/*******************************************************************************
 * Computes the feed forward acceleration
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::computeFfXpp(double* omegap_des,
                                           const double* eapp_des) const
{
  //wp = dB/dt ap + B app
  double ea[3], eap[3];
  this->computeX(ea);
  this->computeXp(eap);

  double B[3][3];
  EulerRateToOmegasMat(B, ea, this->eulerOrderVect);

  // wp = B app
  Vec3d_rotate(omegap_des, B, eapp_des);

  double dBeap[3]; // dB/dt ap
  EulerRateToOmegasDerivativeTimesEulerVel(dBeap, ea, eap, this->eulerOrderVect);

  Vec3d_addSelf(omegap_des, dBeap);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::forceTrafo(double* ft_task) const
{
  double ea[3], B[3][3];
  computeX(ea);
  EulerRateToOmegasMat(B, ea, this->eulerOrderVect);   //B = H^-1

  // transform the values obtained by the controller from the complete
  // Jacobian from omegas to Euler
  Vec3d_transRotateSelf(ft_task, B);
}

/*******************************************************************************
 * Transforms the selection into the Jacobian coordinates
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::selectionTrafo(double* S_des_trafo,
                                             const double* S_des) const
{
  double sqrLengthS = Vec3d_sqrLength(S_des);

  if (sqrLengthS == 3. || sqrLengthS == 0.) // fully selected or de-selected
  {
    VecNd_copy(S_des_trafo, S_des, getDim());
  }
  else
  {
    double ea[3], B[3][3], S_full[3];
    computeX(ea);
    EulerRateToOmegasMat(B, ea, this->eulerOrderVect);

    Vec3d_rotate(S_des_trafo, B, S_des);

    Vec3d_setElementsTo(S_full, 1.);
    Vec3d_rotateSelf(S_full, B);

    for (size_t i=0; i<3; i++)
    {
      S_des_trafo[i] /= S_full[i];
    }
  }
}

/*******************************************************************************
 * see Rcs/1.4/RcsController/1.4/doc/latex Sect. 1.7
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::EulerRateToOmegasMat(double B[3][3],
                                                   const double ea[3],
                                                   const int eulerOrderVect[4])
{
  //basis vectors for the individual coordinates
  double e0[3];
  Vec3d_setZero(e0);
  e0[eulerOrderVect[0]] = 1;

  double e1[3];
  Vec3d_setZero(e1);
  e1[eulerOrderVect[1]] = 1;

  double e2[3];
  Vec3d_setZero(e2);
  e2[eulerOrderVect[2]] = 1;

  // the rotation matrix for the second coordinate is needed for both the
  // static and rotating frame
  double A1[3][3];
  Mat3d_setElementaryRotation(A1, eulerOrderVect[1], ea[1]);

  if (eulerOrderVect[3] == EulFrmR) // rotating frame
  {
    // rotation matrix for the first coordinate
    double A0[3][3];
    Mat3d_setElementaryRotation(A0, eulerOrderVect[0], ea[0]);

    // rotate the individual basis vectors
    // e0 is not rotated

    // e1 by the first coordinate
    Vec3d_transRotateSelf(e1, A0);

    // e2 by the first & second coordinates
    Vec3d_transRotateSelf(e2, A1);
    Vec3d_transRotateSelf(e2, A0);
  }
  else // static frame
  {
    // rotation matrix for the third coordinate
    double A2[3][3];
    Mat3d_setElementaryRotation(A2, eulerOrderVect[2], ea[2]);

    // rotate the individual basis vectors
    // e1 by the second & third coordinates
    Vec3d_transRotateSelf(e0, A1);
    Vec3d_transRotateSelf(e0, A2);

    // e1 by the third coordinate
    Vec3d_transRotateSelf(e1, A2);

    // e2 is not rotated
  }

  // build matrix from the individual basis vectors
  for (int i=0; i<3; i++)
  {
    B[i][0] = e0[i];
    B[i][1] = e1[i];
    B[i][2] = e2[i];
  }
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::OmegasToEulerRateMat(double Binv[3][3],
                                                   const double ea[3],
                                                   const int eulerOrderVect[4])
{
  double B[3][3];
  EulerRateToOmegasMat(B, ea, eulerOrderVect);

  // pseudo inverse to make it a bit more robust
  double lambda[3] = {1.e-6, 1.e-6, 1.e-6};
  Mat3d_rwPinv(Binv, B, NULL, lambda);
}

/*******************************************************************************
 * See header
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::EulerRateToOmegasDerivativeTimesEulerVel(double dBeap[3], const double ea[3], const double eap[3], const int eulerOrderVect[4])
{
  Vec3d_setZero(dBeap);

  // dBeap = dB/dt eap
  // here we do not calculate the complete matrix dB/dt at once but do it
  // column-wise see Rcs/1.4/RcsController/1.4/doc/latex Sect. 1.7

  // stuff for middle column needed by both cases

  // directions of the coord
  double e1[3];
  Vec3d_setZero(e1);
  e1[eulerOrderVect[1]] = 1;

  double A1[3][3];
  double A1p[3][3];
  // rotation matrix
  Mat3d_setElementaryRotation(A1, eulerOrderVect[1], ea[1]);
  // derivative of rotation matrix
  Mat3d_dAdq(A1p, eulerOrderVect[1], ea[1]);

  double temp[3];

  if (eulerOrderVect[3] == EulFrmR)
  {
    // directions of the last coord
    double e2[3];
    Vec3d_setZero(e2);
    e2[eulerOrderVect[2]] = 1;

    double A0[3][3];
    double A0p[3][3];
    // rotation matrix for 1st coord
    Mat3d_setElementaryRotation(A0, eulerOrderVect[0], ea[0]);
    // derivative of rotation matrix for 1st coord
    Mat3d_dAdq(A0p, eulerOrderVect[0], ea[0]);

    // 2nd column: A0p^T * eap[0] * e1  * eap[1]
    Vec3d_transRotate(temp, A0p, e1);
    Vec3d_constMulSelf(temp, eap[0]);
    Vec3d_constMulSelf(temp, eap[1]);
    Vec3d_addSelf(dBeap, temp);

    // 3d column part 1: A0p^T * eap[0] * A1^T * e2  * eap[2]
    Vec3d_transRotate(temp, A1, e2);
    Vec3d_transRotateSelf(temp, A0p);
    Vec3d_constMulSelf(temp, eap[0]);
    Vec3d_constMulSelf(temp, eap[2]);
    Vec3d_addSelf(dBeap, temp);

    // 3d column part 2: A0^T * A1p^T * eap[1] * e2  * eap[2]
    Vec3d_transRotate(temp, A1p, e2);
    Vec3d_constMulSelf(temp, eap[1]);
    Vec3d_transRotateSelf(temp, A0);
    Vec3d_constMulSelf(temp, eap[2]);
    Vec3d_addSelf(dBeap, temp);
  }
  else
  {
    // directions of the first coord
    double e0[3];
    Vec3d_setZero(e0);
    e0[eulerOrderVect[0]] = 1;

    double A2[3][3];
    double A2p[3][3];
    // rotation matrix for last coord
    Mat3d_setElementaryRotation(A2, eulerOrderVect[2], ea[2]);
    // derivative of rotation matrix for last coord
    Mat3d_dAdq(A2p, eulerOrderVect[2], ea[2]);

    // 1st column part 1: A2p^T * eap[2] * A1^T * e0  * eap[0]
    Vec3d_transRotate(temp, A1, e0);
    Vec3d_transRotateSelf(temp, A2p);
    Vec3d_constMulSelf(temp, eap[2]);
    Vec3d_constMulSelf(temp, eap[0]);
    Vec3d_addSelf(dBeap, temp);

    // 1st column part 2: A2^T * A1p^T * eap[1] * e0  * eap[0]
    Vec3d_transRotate(temp, A1p, e0);
    Vec3d_constMulSelf(temp, eap[1]);
    Vec3d_transRotateSelf(temp, A2);
    Vec3d_constMulSelf(temp, eap[0]);
    Vec3d_addSelf(dBeap, temp);

    // 2nd column: A2p^T * eap[0] * e1  * eap[1]
    Vec3d_transRotate(temp, A2p, e1);
    Vec3d_constMulSelf(temp, eap[2]);
    Vec3d_constMulSelf(temp, eap[1]);
    Vec3d_addSelf(dBeap, temp);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void Rcs::TaskGenericEuler3D::integrateXp_ik(double* x_res,
                                             const double* ea_curr,
                                             const double* omega,
                                             double dt) const
{
  // Determine the rotation matrix A_CI for ea_curr
  double A_CI[3][3];
  Eul2RotMatrix(A_CI, ea_curr, this->eulerOrder);

  // Rotate current rotation matrix about omega's displacement
  double delta[3];
  Vec3d_constMul(delta, omega, dt);
  Mat3d_rotateOmegaSelf(A_CI, delta, true);

  // use Shoemake code to convert RotMatrix to EulerAngles
  HMatrix H;
  RotMatrix2HMatrix(H, A_CI);
  EulerAngles outAngs;
  outAngs = Eul_FromHMatrix(H, this->eulerOrder);

  x_res[0] = outAngs.x;
  x_res[1] = outAngs.y;
  x_res[2] = outAngs.z;
}

/*******************************************************************************
 * See header.
 ******************************************************************************/
bool Rcs::TaskGenericEuler3D::isValid(xmlNode* node, const RcsGraph* graph)
{
  bool success = Rcs::Task::isValid(node, graph, "genericEuler");

  // Check if frame is static or relative
  char eulerOrderChar[64];

  if (getXMLNodePropertyStringN(node, "eulerOrder", eulerOrderChar, 64) == 0)
  {
    RLOG(3, "Task has no eulerOrder!");
    success = false;
  }

  // S or s: static frame
  if ((eulerOrderChar[3]!='s') && (eulerOrderChar[3]!='S') &&
      (eulerOrderChar[3]!='r') && (eulerOrderChar[3]!='R'))
  {
    RLOG(3, "Wrong character for static or rotating frame: \"%c\" "
         "(Should be s, S, r or R)", eulerOrderChar[3]);
    success = false;
  }

  char eulerOrderSpecifiers[16] = "aAxXbByYcCzZ";

  for (int i=0; i<3; i++)
  {
    bool hasValidSpecifier = false;

    for (int j=0; j<12; j++)
    {
      if (eulerOrderChar[i]==eulerOrderSpecifiers[j])
      {
        hasValidSpecifier = true;
      }
    }

    if (hasValidSpecifier == false)
    {
      RLOG(3, "Wrong character for order element %d: \"%c\" (Should be"
           " one of aAxXbByYcCzZ)", i, eulerOrderChar[i]);
      success = false;
    }

  }


  return success;
}
