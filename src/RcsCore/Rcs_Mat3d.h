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

#ifndef RCS_MAT3D_H
#define RCS_MAT3D_H

#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * \defgroup RcsMat3dFunctions Mat3d: 3-dimensional matrix functions
 */


/*! \ingroup RcsMat3dFunctions
 *  \brief Sets the matrix to the identity matrix.
 */
void Mat3d_setIdentity(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Copies random values within the range between lower and upper
 *         into the matrix.
 */
void Mat3d_setRandom(double A[3][3], double lower, double upper);

/*! \ingroup RcsMat3dFunctions
 *  \brief Sets the matrix to a uniformly distributed random rotation
 *         matrices.
 */
void Mat3d_setRandomRotation(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Adds random values within the range between lower and upper
 *         to the matrix.
 */
void Mat3d_addRandom(double A[3][3], double lower, double upper);

/*! \ingroup RcsMat3dFunctions
*  \brief Adds the values to the diagonal.
*/
void Mat3d_addDiag(double A[3][3], double diag[3]);

/*! \ingroup RcsMat3dFunctions
*  \brief Adds the value to the main diagonal.
*/
void Mat3d_addConstToDiag(double A[3][3], double value);

/*! \ingroup RcsMat3dFunctions
 *  \brief Sets the matrix to zero.
 */
void Mat3d_setZero(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Elementary x-rotation matrix:
 *         <br>
 *         \f$
 *         \mathbf{A_x} =
 *         \mbox{$\left( \begin{array}{ccc}
 *                       {1} & {0} & {0} \\ [0.2em]
 *                       {0} & {\cos(\varphi)} & {\sin(\varphi)} \\ [0.2em]
 *                       {0} & {-\sin(\varphi)} & {cos(\varphi)}
 *                       \end{array} \right)$}
 *         \f$
 */
void Mat3d_setRotMatX(double A[3][3], double angle);

/*! \ingroup RcsMat3dFunctions
 *  \brief Elementary y-rotation matrix:
 *         <br>
 *         \f$
 *         \mathbf{A_y} =
 *         \mbox{$\left( \begin{array}{ccc}
 *                       {\cos(\varphi)} & {0} & {-\sin(\varphi)} \\ [0.2em]
 *                       {0} & {1} & {0} \\ [0.2em]
 *                       {\sin(\varphi)} & {0} & {\cos(\varphi)}
 *                       \end{array} \right)$}
 *         \f$
 */
void Mat3d_setRotMatY(double A[3][3], double angle);

/*! \ingroup RcsMat3dFunctions
 *  \brief Elementary z-rotation matrix:
 *         <br>
 *         \f$
 *         \mathbf{A_z} =
 *         \mbox{$\left( \begin{array}{ccc}
 *                       {\cos(\varphi)} & {\sin(\varphi)} & {0} \\ [0.2em]
 *                       {-\sin(\varphi)} & {\cos(\varphi)} & {0} \\ [0.2em]
 *                       {0} & {0} & {1}
 *                       \end{array} \right)$}
 *         \f$
 */
void Mat3d_setRotMatZ(double A[3][3], double angle);

/*! \ingroup RcsMat3dFunctions
 *  \brief Sets the matrix to the elementary rotation matrix according to
 *         index dir. It must be [0...2], otherwise the function exits
 *         fatally. See Mat3d_setRotMatX(), Mat3d_setRotMatY() and
 *         Mat3d_setRotMatZ() for details.
 */
void Mat3d_setElementaryRotation(double A[3][3], unsigned int dir,
                                 double angle);

/*! \ingroup RcsMat3dFunctions
 *  \brief Copies src to dst. Memory areas may overlap.
 */
void Mat3d_copy(double dst[3][3], double src[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Copies the nine elements from src to dst, row-wise.
 */
void Mat3d_fromArray(double dst[3][3], double src[9]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Copies the matrix elements row-wise to a contiguous array.
 */
void Mat3d_toArray(double dst[9], double src[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief C = A - B.
 */
void Mat3d_sub(double C[3][3], double A[3][3], double B[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief dst -= src.
 */
void Mat3d_subSelf(double dst[3][3], double src[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief dst += src.
 */
void Mat3d_addSelf(double dst[3][3], double src[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief dst *= value.
 */
void Mat3d_constMulSelf(double dst[3][3], double value);

/*! \ingroup RcsMat3dFunctions
 *  \brief Performs a similarity transform on a matrix:
 *
 *         I_I = A_IB * B_I * A_IB^T = A_BI^T * B_I * A_BI.
 *
 *         This does for instance transform the inertia tensor given in
 *         coordinate system B into coordinate system I.
 */
void Mat3d_similarityTransform(double I_I[3][3], double A_BI[3][3],
                               double B_I[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief C = A * B
 */
void Mat3d_mul(double C[3][3], double A[3][3], double B[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief A = B * A
 */
void Mat3d_preMulSelf(double A[3][3], double B[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief A = A * B
 */
void Mat3d_postMulSelf(double A[3][3], double B[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief A_BI = (A_PB)^T * A_PI
 */
void Mat3d_transposeMul(double A_BI[3][3], double A_PB[3][3],
                        double A_PI[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes the relative rotation matrix A_21 = A_2I * (A_1I)^T.
 */
void Mat3d_mulTranspose(double A_21[3][3], double A_2I[3][3],
                        double A_1I[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns true if the rotation matrix is the identity matrix.
 */
bool Mat3d_isIdentity(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns true if the rotation matrix contains only zeros.
 */
bool Mat3d_isZero(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns true if the matrix is symmetric up to a per-element
 *         precision of eps.
 */
bool Mat3d_isSymmetric(double A[3][3], double eps);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns true if the matrix is diagonal. Off-diagonal elements are
 *         considered to be 0 if their absolute value is less than eps.
 */
bool Mat3d_isDiagonal(double A[3][3], double eps);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns true if the matrix elements are elementwise equal with a
 *         tolerance less than eps.
 */
bool Mat3d_isEqual(double A[3][3], double B[3][3], double eps);

/*! \ingroup RcsMat3dFunctions
 *  \brief Transposes the matrix.
 */
void Mat3d_transpose(double dst[3][3], double src[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Transposes the matrix in place.
 */
void Mat3d_transposeSelf(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief In-place inversion of matrix A. The function returns A's
 *         determinant. If the determinant is 0 (matrix not invertible), A
 *         is set to 0.
 */
double Mat3d_inverseSelf(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Inverts the matrix and returns its determinant. If the determinant
 *         is 0 (matrix not invertible), invA is set to 0. The memory of A
 *         and invA must not overlap, otherwise the result is undefined.
 */
double Mat3d_inverse(double invA[3][3], double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes the weighted pseudo-inverse in the form <br> <br>
 *         \f$
 *         \mathbf{J^{\#} =  W^{-1} J^T (J W^{-1} J^T}
 *         + \mathsf{diag} (\mbox{\boldmath$\lambda$}))^{-1}
 *         \f$
 *         <br><br>
 *         Vector \f$ \mathbf{W}^{-1} \f$ must be of dimensions 3, it is
 *         interpreted as the trace of the diagonal weighting matrix. If
 *         \f$ \mathbf{W}^{-1} \f$ is NULL, the weight matrix is assumed to
 *         be an identity matrix. Vector \f$ \mbox{\boldmath$\lambda$}\f$ is
 *         the regularizer to be added to the main diagonal of
 *         \f$ \mathbf{J} \mathbf{W}^{-1} \mathbf{J}^{T}\f$ and may be be
 *         either NULL or of dimension 3. If it is NULL, it is assumed to be
 *         a null vector. The function returns the determinant of the matrix
 *         to be inverted. If the matrix J is singular (determinant is 0),
 *         matrix \f$\mathbf{J^{\#}} \f$ will be set to a zero-matrix.
 */
double Mat3d_rwPinv(double J_pinv[3][3], double J[3][3], double invW[3],
                    double lambda[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes the weighted pseudo-inverse in the form <br> <br>
 *         \f$
 *         \mathbf{J^{\#} =  (J^T W_x J }
 *         + \mathsf{diag} (\mbox{\boldmath$\lambda$}))^{-1}
 *         \mathbf{J^T W_x}
 *         \f$
 *         <br><br>
 *         Vector \f$ \mathbf{W_x} \f$ must be of dimensions 3, it is
 *         interpreted as the trace of the diagonal weighting matrix. If
 *         \f$\mathbf{W_x}\f$  is NULL, the weight matrix is assumed to be an
 *         identity matrix. Vector \f$ \mbox{\boldmath$\lambda$}\f$ is the
 *         regularizer to be added to the main diagonal of
 *         \f$ \mathbf{J}^{T} \mathbf{W_x} \mathbf{J}\f$ and may be be either
 *         NULL or of dimension 3. If it is NULL, it is assumed to be a null
 *         vector. The function returns the determinant of the matrix to be
 *         inverted. If the matrix J is singular (determinant is 0), matrix
 *         \f$\mathbf{J^{\#}} \f$ will be set to a zero-matrix.
 */
double Mat3d_rwPinv2(double J_pinv[3][3], double J[3][3], double Wx[3],
                     double lambda[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes the angular velocity omega that drives the rotation
 *         matrix of A_prev to the one of A. Omega is represented in the
 *         frame of A. It's implemented according to text books as
 *         \f$
 *         \tilde{\omega} = \dot{A} \; A^{T}
 *         \f$
 *         where tilde is the skew symmetric product
 *         <br><br>
 *         \f$
 *         \tilde{\omega} =
 *         \mbox{$\left( \begin{array}{ccc}
 *                       {0} & {-\omega_z} & {\omega_y} \\ [0.2em]
 *                       {\omega_z} & {0} & {-\omega_x} \\ [0.2em]
 *                       {-\omega_y} & {\omega_x} & {0}
 *                       \end{array} \right)$}
 *         \f$
 *         But please take care: The funtion will only work properly for small
 *         differences between A and A_prev.
 */
void Mat3d_getOmega(double A[3][3], double A_prev[3][3], double omega[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Rotates the matrix about the body-fixed axis and the given angle.
 *         The right-hand rule applies to the direction.
 *         <br>
 *         A_BI = ElementaryRotation(axis, theta) * A_BI.
 *
 *  \param[in,out] A_BI Rotation matrix to be rotated in place
 *  \param[in] axis Axis index, must be 0, 1 or 2. Otherwise, the
 *                  function will exit with a fatal error.
 *  \param[in] angle Rotation angle in radians.
 */
void Mat3d_rotateSelfAboutXYZAxis(double A_BI[3][3], unsigned int axis,
                                  double angle);

/*! \ingroup RcsMat3dFunctions
 *  \brief Rotates the matrix A_src about the given axis with the given
 *         angle to matrix A_dst.
 *
 *  \param[out] A_2I  Resulting rotation matrix
 *  \param[in] I_axis Rotation axis, represented in the I-frame. It does not
 *                    need to be of unit length. If its length is 0, the result
 *                    A_BI is set to identity.
 *  \param[in] angle  Rotation angle in radians.
 *  \param[in] A_1I   Rotation matrix to be rotated
 */
void Mat3d_rotateAxisAngle(double A_2I[3][3], const double I_axis[3],
                           const double angle, double A_1I[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Rotates the matrix about the given axis with the given angle.
 *
 *  \param[in,out] A_BI Rotation matrix to be rotated in place
 *  \param[in] I_axis   Rotation axis, represented in the I-frame. It does not
 *                      need to be of unit length. If its length is 0, the
 *                      result A_BI is set to identity.
 *  \param[in] angle    Rotation angle in radians.
 */
void Mat3d_rotateAxisAngleSelf(double A_BI[3][3], const double I_axis[3],
                               const double angle);

/*! \ingroup RcsMat3dFunctions
 *  \brief Rotates the matrix with the given angular velocity. If
 *         worldFrame is true, omega will be interpreted around the
 *         axes of the reference-frame I. If false, it will be interpreted
 *         around the body-fixed axes of the B-frame.
 */
void Mat3d_rotateOmegaSelf(double A_BI[3][3], const double omega[3],
                           bool worldFrame);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes the minimum rotation angle between a start and
 *         target orientation when the start orientation is rotated
 *         around a given axis.
 *
 *  \param[in] A_SI     Start orientation
 *  \param[in] A_TI     Target orientation
 *  \param[in] axis     Unit-length rotation axis (Unit length is not checked)
 *  \return             Resulting minimum angle
 */
double Mat3d_getMinimumRotationAngle(double A_SI[3][3], double A_TI[3][3],
                                     const double axis[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief From a given set of Euler angles, this function returns the
 *         matrix H that projects the angular velocity vector on the Euler
 *         angle velocities:
 *         \f$
 *         \mbox{$\left( \begin{array}{c}
 *                       \dot{\alpha} \\ \dot{\beta} \\  \dot{\gamma}
 *                       \end{array} \right)$}
 *         =
 *         \mbox{$\left( \begin{array}{ccc}
 *                       {1} &
 *                       {\frac{\sin(\alpha) \sin(\beta)}{\cos(\beta)}} &
 *                       {-\frac{\cos(\alpha) \sin(\beta)}{\cos(\beta)}} \\
 *                       [0.2em]
 *                       {0} & {\cos(\alpha)} & {\sin(\alpha)} \\ [0.2em]
 *                       {0} &
 *                       { -\frac{\sin(\alpha)}{\cos(\beta)}} &
 *                       { \frac{\cos(\alpha)} {\cos(\beta)}}
 *                       \end{array} \right)$}
 *         \mbox{$\left( \begin{array}{c}
 *                       _I \dot{\omega}_y \\
 *                       _I \dot{\omega}_y \\
 *                       _I \dot{\omega}_z
 *                       \end{array} \right)$}
 *         \f$
 */
void Mat3d_getEulerVelocityMatrix(double H[3][3], const double ea[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief From a given set of angular angles, this function returns the
 *         matrix invH that projects the Euler velocity vector on the
 *         angular velocities (in the world frame):
 *         \f$
 *         \mbox{$\left( \begin{array}{c}
 *                       _I \dot{\omega}_y \\
 *                       _I \dot{\omega}_y \\
 *                       _I \dot{\omega}_z
 *                       \end{array} \right)$}
 *         =
 *         \mbox{$\left( \begin{array}{ccc}
 *                       {1} & {0} & {\sin(\beta)} \\ [0.2em]
 *                       {0} & {\cos(\alpha)} &
 *                       {-\sin(\alpha) \cos(\beta)} \\ [0.2em]
 *                       {0} & \sin(\alpha) & {\cos(\alpha) \cos(\beta)}
 *                       \end{array} \right)$}
 *         \mbox{$\left( \begin{array}{c}
 *                       \dot{\alpha} \\ \dot{\beta} \\  \dot{\gamma}
 *                       \end{array} \right)$}
 *         \f$
 */
void Mat3d_getInverseEulerVelocityMatrix(double invH[3][3],
                                         const double ea[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief \f$
 *         \frac{dH^{-1}}{dt}
 *         = \mbox{$\left( \begin{array}{ccc}
 *                       {0} & {0} & {\dot{\beta}\cos(\beta)} \\ [0.2em]
 *                       {0} & {-\dot{\alpha}\sin(\alpha)} &
 *                       {-\dot{\alpha}\cos(\alpha)\cos(\beta)+\dot{\beta}\sin(\alpha)\sin(\beta)} \\ [0.2em]
 *                       {0} & \dot{\alpha}\cos(\alpha) &
 *                       {-\dot{\alpha}\sin(\alpha)\cos(\beta)-\dot{\beta}\cos(\alpha)\sin(\beta)}
 *                       \end{array} \right)$}
 *        \f$
 */
void Mat3d_getInverseEulerVelocityMatrixDerivative(double dInvH[3][3],
                                                   const double ea[3],
                                                   const double eap[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Extracts the Euler angle error (x-y-z-order) from the rotation
 *         matrices of the given transformations:
 *         <br><br>
 *         \f$ \mathbf{e} =  \frac{1}{2}\left(
 *         \mathbf{n} \times \mathbf{n_{des}} +
 *         \mathbf{s} \times \mathbf{s_{des}} +
 *         \mathbf{a} \times \mathbf{a_{des}}
 *         \right)\f$ .
 *         <br>
 *         The algorithm has been introduced in: <br> <br>
 *         <b> Stefano Chiaverini: Singularity-robust task priority
 *         redundancy resolution for real-time kinematic control of robot
 *         manipulators,
 *         <br><i> IEEE Transactions on Robotics and
 *         Automation, Vol. 13, No. 3, June 1997. </i></b>
 */
void Mat3d_getEulerError(double e[3], double A_act[3][3],
                         double A_des[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Euler error partial derivative wrt. x_des: <br>
 *         de/dx = 0.5* (n x dx(nd) + s x dx(sd) + a x dx(ad)). <br>
 *         with n being the 1st row of the A_BI rotation matrix, s the second
 *         and a the third one. Vector x_des holds the desired Euler angles
 *         in the xyz order (alpha beta gamma).
 */
void Mat3d_dEulerErrorDx(double dedx[3][3], const double x_des[3],
                         double A_BI[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Constructs a rotation matrix from a x-y-z Euler angle vector.
 *         Function adapted from Ken Shoemake, 1993.
 */
void Mat3d_fromEulerAngles(double A_BI[3][3], const double ea[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Same as \ref Mat3d_fromEulerAngles, but with individual double
 *         values.
 */
void Mat3d_fromEulerAngles2(double A_BI[3][3], double a, double b, double c);

/*! \ingroup RcsMat3dFunctions
 *  \brief Extracts the Euler angles (x-y-z-order) from the rotation
 *         matrix of the given transformation. The function uses
 *         Shoemakers method.
 */
void Mat3d_toEulerAngles(double ea[3], double A_BI[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Constructs a rotation matrix from an axis and an angle. The
 *         axis will be normalized. If the axis is degenerate, an
 *         identity matrix will be copied to A_BI. <br>
 *         The implementation follows: <br>
 *         http://en.wikipedia.org/wiki/Rotation_matrix
 *
 *  \param[in] A_BI Rotation matrix
 *  \param[in] axis Direction vector. It does not need to be of unit length.
 *                  If its length is 0, the result A_BI is set to the
 *                  identity matrix.
 *  \param[in] angle Rotation angle in radians.
 */
void Mat3d_fromAxisAngle(double A_BI[3][3], const double axis[3],
                         const double angle);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes the axis angle representation to rotate A_1I on A_2I.
 *         This function follows the Springer Handbook of Robotics, Part A,
 *         p. 12, 2008, except that the rotation matrix is assumed to be
 *         column major. If the angle is 0 or 180 degrees, there is no unique
 *         solution to the rotation axis. In this case, it is set to (1 0 0).
 *         The function returns the angle, and copies the normalized rotation
 *         axis into argument axis.
 */
double Mat3d_getAxisAngle(double axis[3], double A_2I[3][3],
                          double A_1I[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes the axis angle representation of a matrix. This function
 *         follows the Springer Handbook of Robotics, Part A, p. 12, 2008,
 *         except that the rotation matrix is assumed to be column major.
 *         If the angle is 0 or 180 degrees, there is no unique solution to
 *         the rotation axis. In this case, it is set to (1 0 0). The function
 *         returns the angle, and copies the normalized rotation axis into
 *         argument axis.
 */
double Mat3d_getAxisAngleSelf(double axis[3], double rot[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Gram-Schmidt orthogonalisation. If the rotation matrix is far
 *         from right-handed, the resulting matrix might be left-handed.
 *         If the matrix cannot be orthonormalized, the function does
 *         not modify A and returns false, otherwise true.
 */
bool Mat3d_orthonormalizeSelf(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns the determinant of the 3x3 rotation matrix.
 */
double Mat3d_determinant(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns the trace of the 3x3 rotation matrix.
 */
double Mat3d_trace(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns false if any matrix element is not finite, true otherwise.
 */
bool Mat3d_isFinite(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns true if the rotation matrix is valid, false otherwise.
 *         The function checks for
 *         - NULL pointer
 *         - unit length of the coordinate axes
 *         - orthogonality of the coordinate axes (A^T * A = I)
 *         - right-handedness of the rotation matrix (det(A) = 1)
 *         - all elements to be finite.
 *         If the rotation matrix is not valid, the function gives some
 *         verbose output on debug level 4.
 */
bool Mat3d_isValid(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Partial derivative of a elementary rotation matrix with respect
 *         to the angular component. Index dir must correspond to x=0, y=1
 *         and z=2. If out of this range, the function exits fatally.
 *
 \verbatim
 Elementary rotation matrices:       Their derivatives:

        1   0     0                     0    0    0
 Ax =   0   cos   sin            dAx =  0   -sin  cos
        0  -sin   cos                   0   -cos -sin

        cos   0   -sin                 -sin   0    -cos
 Ay =   0     1    0             dAy =  0     0     0
        sin   0    cos                  cos   0    -sin

        cos   sin  0                   -sin   cos   0
 Az =  -sin   cos  0             dAz = -cos  -sin   0
        0     0    1                    0     0     0
 \endverbatim
 */
void Mat3d_dAdq(double dAdq[3][3], unsigned int dir, double q);

/*! \ingroup RcsMat3dFunctions
 *  \brief Interpolation of transformations using matrix exponential. Value
 *         t is the interpolation factor. A value between [0...1]
 *         interpolates between A_1I and A_2I, but t can also be negative or
 *         larget than t. The function determines the fixed rotation axis
 *         that brings A_1I on A_2I, and the angle around this axis. The
 *         resulting rotation matrix is rotated about this axis.
 */
void Mat3d_slerp(double A[3][3], double A_1I[3][3], double A_2I[3][3],
                 double t);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes an rotation matrix (Slerp interpolation) that is the
 *         interpolation between A_from and A_to, with the maximum distance
 *         maxAngle from A_from.
 */
double Mat3d_clip(double A_clip[3][3], double A_from[3][3], double A_to[3][3],
                  double maxAngle);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes the skew-symmetric matrix so that
 *         \f$ a \times b = \tilde{a} b\f$
 *
 *         \f$
 *         \mathbf{\tilde{r}} =
 *         \mbox{$\left( \begin{array}{ccc}
 *                       {0} & {-r_z} & {r_y} \\ [0.2em]
 *                       {r_z} & {0} & {-r_x} \\ [0.2em]
 *                       {-r_y} & {r_x} & {0}
 *                       \end{array} \right)$}
 *         \f$
 */
void Mat3d_skew(double rSkew[3][3], const double a[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes the transpose tilde operator, see \ref Mat3d_skew.
 *
 *         \f$
 *         \mathbf{\tilde{r}^T} =
 *         \mbox{$\left( \begin{array}{ccc}
 *                       {0} & {r_z} & {-r_y} \\ [0.2em]
 *                       {-r_z} & {0} & {r_x} \\ [0.2em]
 *                       {r_y} & {-r_x} & {0}
 *                       \end{array} \right)$}
 *         \f$
 */
void Mat3d_transposeSkew(double rSkew[3][3], const double r[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Prints the matrix to stdout.
 */
void Mat3d_print(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Prints the matrix to a file descriptor.
 */
void Mat3d_fprint(FILE* fd, double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Prints the matrix with a comment and n digits after the colon to
 *         stderr. If comment is NULL, it will be skipped.
 */
void Mat3d_printCommentDigits(const char* text, double mat[3][3],
                              unsigned int digits);

/*! \ingroup RcsMat3dFunctions
 *  \brief Prints the matrix with a comment and a given format string to
 *         stderr. The format string is considered for one number, e.g.
 *         Mat3d_printFormatted("Rotation matrix", "%g ", rm);
 *         If comment is NULL, it will be skipped.
 */
void Mat3d_printFormatted(const char* text, const char* format,
                          double M[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns the axis angle between the rotation matrices A1 and A2.
 */
double Mat3d_diffAngle(double A1[3][3], double A2[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns the axis angle between the rotation matrix A and the
 *         identity rotation matrix.
 */
double Mat3d_diffAngleSelf(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns the partial derivative of the axis angle with respect to
 *         the Euler angles.
 */
void Mat3d_dDiffAngleDEuler(double dAAdEA[3], const double ea[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Sets the diagonal of A to vector v. The off-diagonal elements
 *         remain unchanged.
 */
void Mat3d_setDiag(double A[3][3], const double v[3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Computes a rotation matrix from a vector. The rotation matrix has
 *         the normalized vector v pointing into the direction given by dir.
 *         Index dir must be 0 for the x-direction, 1 for the y-direction or
 *         2 for the z-direction. If index is none of these, the function
 *         will exit with a fatal error. If the vector v is of length zero,
 *         A will be set to the identity matrix. Vector v and matrix A must not
 *         overlap in memory, otherwise the result is undefined.
 */
void Mat3d_fromVec(double A[3][3], const double v[3], int dir);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns the Frobenius norm of the matrix, which is the square root
 *         of the squared sum of its elements.
 */
double Mat3d_getFrobeniusnorm(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Returns the highest (f)absolut value of the matrix.
 */
double Mat3d_maxAbsEle(double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Calculates eigenvectors and values for a symmetric 3x3 matrix A.
 *         If the eigen decomposition cannot be found (e.g. A is not
 *         symmetric or degenerate), the function returns false, and the
 *         arguments remain unchanged.
 *
 *         The eigenvectors are sorted in increasing order and form a
 *         right-handed coordinate system.
 *
 *  \param[out] V Eigenvectors in columns of V, corresponding eigenvalues in d
 *                The Eigenvectors are of unit-length.
 *  \param[out] d Eigenvalues in the same order as eigenvectors in V
 *  \param[in]  A  Symmetric matrix
 *  \return True for success, false otherwise (e.g. A is non-symmetric).
 */
bool Mat3d_getEigenVectors(double V[3][3], double d[3], double A[3][3]);

/*! \ingroup RcsMat3dFunctions
 *  \brief Compares the Eigenvectors and Eigenvalues and returns true if they
 *         are equal, false otherwise. The Eigenbases are considered the same
 *         invariant of the sign of the Eigenvector's direction.
 *
 *  \param[in]  lambda1  Eigenvalues of basis 1
 *  \param[in]  V1       Eigenvectors of basis 1, stored in the columns of V1
 *  \param[in]  lambda2  Eigenvalues of basis 2
 *  \param[in]  V2       Eigenvectors of basis 2, stored in the columns of V1
 *  \param[in]  eps      Angular threshold below which two vectors are
 *                       considered to be parallel
 *  \return True for success, false otherwise (e.g. A is non-symmetric).
 */
bool Mat3d_compareEigenbasis(double lambda1[3], double V1[3][3],
                             double lambda2[3], double V2[3][3],
                             double eps);

/*! \ingroup RcsMat3dFunctions
 *  \brief First order low pass filter for rotation matrices. The new rotation
 *         will be filtered with a SLERP, multiplied by the time constant. For
 *         tmc=1, the filtered rotation will be set to the new sample, for any
 *         smaller value, the filtered rotation will be smoothed.
 *
 *  \param[in,out]  A_filt   Fitered rotation
 *  \param[in]      A_new    New rotation matrix to be incorporated
 *  \param[in]      tmc      Time constant between [0...1]. Smaller is smoother.
 *  \return False if filtered matrix can't be orthonormalized, true for success.
 */
bool Mat3d_firstOrderLPF(double A_filt[3][3], double A_new[3][3], double tmc);


#ifdef __cplusplus
}
#endif

#endif   // RCS_MAT3D_H
