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

#ifndef RCS_HTR_H
#define RCS_HTR_H

#include "Rcs_bool.h"

#include <stdio.h>



#ifdef __cplusplus
extern "C" {
#endif



/*!
 * \defgroup RcsHTrFunctions HTr: Transformation functions
 *
 *         HTr stands for homogeneous transform. This data structure consists of
 *         a translation vector (org means origin), and a rotation matrix (rot).
 *         The rotation is applied after the translation.
 *
 *         The convention in Rcs is the following: Rotation matrices are assumed
 *         to be in row-major form. It means that A_21 denotes a rotation of
 *         frame 1 into frame 2. It also means that the rows of matrix A_21
 *         correspond to the unit vectors of frame 2, represented in frame 1.
 *
 */
typedef struct
{
  double org[3];     ///< Translation vector (org means origin)
  double rot[3][3];  ///< Rotation matrix, applied after translation
} HTr;


/*! \ingroup RcsHTrFunctions
 *  \brief Returns true if the HTr is valid, false otherwise. The function
 *         checks for
 *         - NULL pointer
 *         - origin vector elements to be finite
 *         - rotation matrix to be valid (see Mat3d_isValid())
 *         If the transformation is not valid, the function gives some
 *         verbose output on debug level 4.
 */
bool HTr_isValid(const HTr* A);

/*! \ingroup RcsHTrFunctions
 *  \brief Returns true if the rotation matrix of the HTr is the
 *         identity matrix and the origin vector is (0 0 0).
 */
bool HTr_isIdentity(const HTr* A);

/*! \ingroup RcsHTrFunctions
 *  \brief Returns true if the elements are elementwise equal with a
 *         tolerance less than eps. Both position and rotation matrix
 *         elements are considered.
 */
bool HTr_isEqual(const HTr* A, const HTr* B, double eps);

/*! \ingroup RcsHTrFunctions
 *  \brief Prints the transform to a file descriptor.
 */
void HTr_fprint(FILE* fd, const HTr* A);

/*! \ingroup RcsHTrFunctions
 *  \brief Prints the transform.
 */
void HTr_print(const HTr* A);

/*! \ingroup RcsHTrFunctions
 *  \brief Prints the transform with a comment string. If comment is NULL,
 *         it will be skipped.
 */
void HTr_printComment(const char* comment, const HTr* A);

/*! \ingroup RcsHTrFunctions
 *  \brief Returns an identity HTr from the heap. You have to take care
 *         that it gets deleted. The function terminates if no memory could
 *         be allocated.
 */
HTr* HTr_create(void);

/*! \ingroup RcsHTrFunctions
 *  \brief Returns an (deep) copy of src from the heap. You have to take care
 *         that it gets deleted. If src is NULL, no memory will be allocated
 *         and NULL is returned.
 */
HTr* HTr_clone(const HTr* src);

/*! \ingroup RcsHTrFunctions
 *  \brief Sets the matrix part of self to the identity matrix and the
 *         origin to [0 0 0].
 */
void HTr_setIdentity(HTr* self);

/*! \ingroup RcsHTrFunctions
 *  \brief Sets the matrix part to a random rotation matrix (valid) and the
 *         vector part to a random vector [-1 ... 1]
 */
void HTr_setRandom(HTr* self);

/*! \ingroup RcsHTrFunctions
 *  \brief Sets all elements of the transformation to 0.
 */
void HTr_setZero(HTr* self);

/*! \ingroup RcsHTrFunctions
 *  \brief Interpolation of transformations using matrix exponential. Value
 *         t is the interpolation factor. A value between [0...1]
 *         interpolates between A_1I and A_2I, but t can also be negative or
 *         larget than t. The function determines the fixed rotation axis
 *         that brings A_1I on A_2I, and the angle around this axis. The
 *         resulting rotation matrix is rotated about this axis. The
 *         translation between the transforms is linearly interpolated.
 */
void HTr_slerp(HTr* A, const HTr* A_1I, const HTr* A_2I, double t);

/*! \ingroup RcsHTrFunctions
 *  \brief Deep copy from src to dst.
 */
void HTr_copy(HTr* dst, const HTr* src);

/*! \ingroup RcsHTrFunctions
 *  \brief Invert transformation
 */
void HTr_transpose(HTr* A_12, const HTr* A_21);

/*! \ingroup RcsHTrFunctions
 *  \brief Invert transformation in place.
 */
void HTr_transposeSelf(HTr* A_12);

/*! \ingroup RcsHTrFunctions
 *  \brief Transformation from body to world coordinates
 *
 *         A_2I  = A_21 * A_1I
 *         I_r_2 = I_r_1 + A_I1 * 1_r_12
 */
void HTr_transform(HTr* A_2I, const HTr* A_1I, const HTr* A_21);

/*! \ingroup RcsHTrFunctions
 *  \brief See HTr_transform().
 */
void HTr_transformSelf(HTr* A_2I /* in as A_1I */, const HTr* A_21);

/*! \ingroup RcsHTrFunctions
 *  \brief Computes the relative transformation
 *         Rotation matrix: A_21   = A_2I * (A_1I)^T.
 *         Origin vector:   1_r_12 = A_1I * (I_r_2 - I_r_1)
 */
void HTr_invTransform(HTr* A_21, const HTr* A_1I, const HTr* A_2I);

/*! \ingroup RcsHTrFunctions
 *  \brief See HTr_invTransform().
 */
void HTr_invTransformSelf(HTr* A_21 /* in as A_2I */, const HTr* A_1I);

/*! \ingroup RcsHTrFunctions
 *  \brief Returns a pointer to an identity transform. This points to a
 *         static variable. Please NEVER change the contents of this pointer,
 *         since this affects every caller to this function.
 */
const HTr* HTr_identity();

/*! \ingroup RcsHTrFunctions
 *  \brief Writes the contents of a given HTr into a given string(char*) as
 *         one line
 *
 *  Length of the string should be at least 15 * 12 = 180 chars to assure
 *  there no invalid write
 */
void HTr_toString(char* str, const HTr* A);

/*! \ingroup RcsHTrFunctions
 *  \brief Reads the contents of a HTr from a given string (which has been
 *         previously generated using HTr_toString()
 *
 *  \return true for success, false otherwise:
 *          - A or str is NULL
 *          - not 12 space-separated entries found in str.
 */
bool HTr_fromString(HTr* A, const char* str);

/*! \ingroup RcsHTrFunctions
 *  \brief Constructs a HTr from 2 points, so that p1-p2 will become the
 *         unit z-axis of the rotation matrix. The x- and y-axis are
 *         computed just somehow to be orthogonal. The HTrs' origin
 *         will be set to p1. If the distance between p1 and p2 is zero,
 *         the matrix part is set to the identity matrix.
 */
void HTr_from2Points(HTr* A_KI, const double p1[3], const double p2[3]);

/*! \ingroup RcsHTrFunctions
 * \brief Fills the 12 vector's values from a given HTr. THe first 3 elements
 *        will be the position, the consecutive 9 elements will be the
 *        rotation matrix.
 *
 * \param vec 12D vector holding 3D position and rotation matrix to be filled.
 *            The rotation matrix will be copied in row-major form.
 * \param A_KI Transformation.
 */
void HTr_toVector(double vec[12], const HTr* A_KI);

/*! \ingroup RcsHTrFunctions
 * \brief Fills the HTr's values from a given 12D vector (3 position and
 *        9 rotation matrix elements)
 * \param A HTr to be filled
 * \param vec 12D vector holding 3D position and rotation matrix.
 */
void HTr_fromVector(HTr* A, const double vec[12]);

/*! \ingroup RcsHTrFunctions
 * \brief Fills the HTr's values from a given 6D vector (XYZ and Euler X-Y-Z
 *        in radians)
 *
 * \param A HTr to be filled
 * \param x 6D vector holding 3D position and 3D orientation in Euler X-Y-Z
 *          order (angular units are in radians)
 */
void HTr_from6DVector(HTr* A, const double x[6]);

/*! \ingroup RcsHTrFunctions
 * \brief Create a 6D vector (XYZ and Euler X-Y-Z in degrees) from a
 *        transformation.
 *
 * \param x 6D vector to be filled with 3D position and 3D orientation in
 *        Euler X-Y-Z order
 * \param A HTr holding the values
 */
void HTr_to6DVector(double x[6], const HTr* A);

/*! \ingroup RcsHTrFunctions
 * \brief Pole direction according to Springer Handbook of Robotics, pp. 16.,
 *        table 1.4.
 *
 * \param A_BW              Current transformation from (W)orld to (B)ody.
 *                          The rotation is represented in row-major form.
 * \param A_BW_prev         Prevoius transformation from (W)orld to (B)ody
 *                          The rotation is represented in row-major form.
 * \param polePt            One pole point on pole axis in (W)orld coordinates
 * \param poleDirection     Normalized pole direction in (W)orld coordinates
 * \param v_pol             Norm of linear velocity along pole axis. If
 *                          it is NULL, it is ignored.
 * \param om_pol            Norm of angular velocity around pole axis. If
 *                          it is NULL, it is ignored.
 * \return True if Chalses Pole could be calculated, false otherwise. This
 *         can happen if there is only a tiny rotation between the
 *         transformations.
 */
bool HTr_computeChaslesPole(const HTr* A_BW,
                            const HTr* A_BW_prev,
                            double polePt[3],
                            double poleDirection[3],
                            double* v_pol,
                            double* om_pol);

/*! \ingroup RcsHTrFunctions
 *  \brief Copies the transformation in src into the de-referenced dst.
 *         If that is NULL, it will be allocated and pointed to by dst.
 *         If it already exists but has not enough memory for src, it is
 *         re-alloced and pointed to. Otherwise, it is just copied.
 *         If src is NULL, the memory pointed to by dst will be freed
 *         (if it was allocated) and set to NULL.
 *
 *  \param[out] dst    Pointer to target transformation. The modified one
 *                     may have a different memory adress after re-allocation,
 *                     therefore we need to pass a pointer here.
 *  \param[in]  src    Transformation to be copied into dst.
 */
void HTr_copyOrRecreate(HTr** dst, const HTr* src);


#ifdef __cplusplus
}
#endif

#endif   // RCS_HTR_H
