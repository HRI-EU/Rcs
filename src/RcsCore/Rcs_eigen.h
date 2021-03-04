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

#ifndef RCS_EIGEN_H
#define RCS_EIGEN_H

#include "Rcs_MatNd.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \ingroup MatNdFunctions
 *  \brief Computes the singular value decomposition J = U S V^T of a matrix
 *         and returns the rank of the matrix. The singular value matrix is
 *         assumed to be a n x 1 diagonal vector. The dimensions of arrays
 *         U, S and V are adjusted by the function. The singular values in the
 *         vector S are sorted in decreasing order.
 *
 *         The columns of U are the eigenvectors of J*J^T (left singular
 *         vectors), the columns of V are the eigenvectors of J^T*J (right
 *         singular vectors).
 *
 *  \param[out] U     Matrix of dimension J->m x J->n
 *  \param[out] S     Singular vector of dimension J->n x 1
 *  \param[out] V     Matrix of dimension J->n x J->n
 *  \param[in]  J     Input matrix (dimension J->m x J->n)
 *  \param[in]  eps   Threshold to cut off Singular Values.
 *  \return           Rank of J. It is determined as the number of singular
 *                    values that are less than eps.
 */
unsigned int MatNd_SVD(MatNd* U, MatNd* S, MatNd* V, const MatNd* J,
                       double eps);

/*! \ingroup MatNdFunctions
 *  \brief Computes the solution x for the linear equation system Ax = b
 *         using the singular value decomposition. A must be square.
 *         Singular values that are below a condition number of 1.0e-12
 *         are considered as zero.
 *
 *  \param[out] x   Column vector (n x 1) holding the result
 *  \param[in]  A   Matrix (n x n)
 *  \param[in]  b   Right hand side column vector (n x 1)
 *  \return Determinant of A.
 */
double MatNd_SVDSolve(MatNd* x, const MatNd* A, const MatNd* b);

/*! \ingroup MatNdFunctions
 *  \brief Inverts a square matrix using SVD: inv = V diag(S^-1) U^T
 *         Only non-zero singular values are considered. A code example is
 *         in the tests. The rank of the matrix is returned. A matrix
 *         can be inverted in place. If the matrix dimension is 0 x 0, the
 *         determinant is set to 1, and inv is left unchanged.
 */
unsigned int MatNd_SVDInverse(MatNd* inv, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief In-place version of \ref MatNd_SVDInverse().
 */
unsigned int MatNd_SVDInverseSelf(MatNd* A);

/*! \ingroup MatNdFunctions
 *  \brief Returns the rank of a matrix using SVD. It is determined as the
 *         number of singular values that are less than eps.
 */
unsigned int MatNd_rank(const MatNd* J, double eps);

/*! \ingroup MatNdFunctions
 *  \brief Performs the QR decomposition of an arbitrary shaped matrix
 *         A = Q R
 *         <br> where <br>
 *         \f$
 *         \mathbf{Q^T Q} = \mathbf{I}
 *         \f$
 *         <br> and <br>
 *         \f$
 *         \mathbf{R x} = \mathbf{Q^T b}
 *         \f$
 *         <br>
 *         If A is square, the determinant can be calculated as the product
 *         of the diagonal elements of R. This is the case since the matrix
 *         Q is orthogonal, which makes its determinant det(Q)=1, and the
 *         determinant of a triangular matrix is the product of its diagonal
 *         elements.
 *
 *  \param[out] Q   Orthogonal Q-matrix, must be of dimensions m x m
 *  \param[out] R   Upper triangular R-matrix, must be of dimensions m x n
 *  \param[in]  A   m x n input matrix
 */
void MatNd_QRDecomposition(MatNd* Q, MatNd* R, const MatNd* A);

/*! \ingroup MatNdFunctions
 *  \brief Calculates eigenvectors and eigenvalues for a symmetric matrix A.
 *         The Eigenvectors correspond to the columns of V. The eigenvalues
 *         are sorted in increasing order. If the decomposition fails, the
 *         function returns false, and the arguments remain unchanged.
 *
 *  \param[out] V Eigenvectors in columns of V, corresponding eigenvalues in d
 *  \param[out] d Eigen values in increasing order
 *  \param[in] A  Symmetric 3x3 matrix
 *  \return True for success, false otherwies (e.g. A is non-symmetric).
 */
bool MatNd_getEigenVectors(MatNd* V, double* d, const MatNd* A);

/*! \ingroup MatNdFunctions
 *  \brief Solves the linear equation system A*X=B for X. All quantities may
 *         be in matrix form. The function uses the Eigen3 column-pivoting
 *         Housholder QR decomposition.
 */
void MatNd_HouseholderQR(MatNd* X, const MatNd* A, const MatNd* B);

#ifdef __cplusplus
}
#endif

#endif // RCS_EIGEN_H
