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

#ifndef RCS_MATND_H
#define RCS_MATND_H

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>



/*!
 * \defgroup MatNdFunctions MatNd: N-dimensional matrix functions
 *
 *           Functions for 2-dimensional arrays. Unless not specified in the
 *           documentation, all functions are re-entrant and thread-safe.
 *
 *           Unless stated differently, the functions assume the output arrays
 *           to be shaped to the correct dimensions.
 *           The size variable indicates the maximum number of double values
 *           which the array can store. However, all computations are done for
 *           the indicated shape (m rows and n columns) of the array. For
 *           instance, the MatNd_setZero() function sets the first m*n elements
 *           to zero, and not size elements.
 *
 *           A unique property of this library is its built-in concept to avoid
 *           dynamic memory allocation. The macros \ref MatNd_create2() and
 *           \ref MatNd_clone2() allocate array memory on the stack, if their
 *           dimensions do not exceed the ones given in
 *           \ref MATND_MAX_STACK_VECTOR_SIZE (for 1-dimensional arrays) and in
 *           \ref MATND_MAX_STACK_MATRIX_SIZE (for multi-dimensional arrays).
 *           It is possible to change these settings according to the problem
 *           at hand, or to easily disable stack memory if not desired.
 *
 */




/*! \ingroup MatNdFunctions
 *  \brief For some essential functions, we avoid heap allocation if the
 *         dimension of a vector is smaller than the number of doubles
 *         below.
 */
#define MATND_MAX_STACK_VECTOR_SIZE (36)

/*! \ingroup MatNdFunctions
 *  \brief For some essential functions, we avoid heap allocation if the
 *         dimension of a matrix is smaller than the number of doubles
 *         below.
 */
#define MATND_MAX_STACK_MATRIX_SIZE \
  (MATND_MAX_STACK_VECTOR_SIZE*MATND_MAX_STACK_VECTOR_SIZE)


/*! \ingroup MatNdFunctions
 *  \brief Simple array with 2 dimensions.
 */
typedef struct _MatNd
{
  double* ele;       ///< Pointer to the double array
  unsigned int m;    ///< Current number of rows
  unsigned int n;    ///< Current number of columns
  unsigned int size; ///< Number of double values in ele
  bool stackMem;     ///< If true, the ele pointer points to the stack
} MatNd;

/*! \ingroup MatNdFunctions
 *  \brief If this macro is defined, the bytes allocated with the below macros
 *         are counted, and the maximum is stored.
 */
//#define MATND_DEBUG_STACK

#ifdef MATND_DEBUG_STACK
extern unsigned int MatNd_stackBytes;
extern unsigned int MatNd_maxStackBytes;
#define MATND_COUNT_ALLOCA_BYTES_ROW_X_COL(rows,cols)               \
  MatNd_stackBytes += sizeof(MatNd) + (rows)*(cols)*sizeof(double); \
  if (MatNd_stackBytes>MatNd_maxStackBytes)                         \
  {                                                                 \
    MatNd_maxStackBytes = MatNd_stackBytes;                         \
  }
#define MATND_COUNT_ALLOCA_BYTES_SIZE(src)                          \
  MatNd_stackBytes += sizeof(MatNd) + src->size*sizeof(double);     \
  if (MatNd_stackBytes>MatNd_maxStackBytes)                         \
  {                                                                 \
    MatNd_maxStackBytes = MatNd_stackBytes;                         \
  }
#else
#define MATND_COUNT_ALLOCA_BYTES_ROW_X_COL(rows,cols)
#define MATND_COUNT_ALLOCA_BYTES_SIZE(src)
#endif

/*! \ingroup MatNdFunctions
 *  \hideinitializer
 *  \brief see \ref MatNd_set.
 *         This version is more efficient but does not do any bounds checking.
 */
#define MatNd_set2(self, row, col, val)         \
  self->ele[(row) * self->n + (col)] = val;

/*! \ingroup MatNdFunctions
 *  \hideinitializer
 *  \brief see \ref MatNd_get.
 *         This version is more efficient but does not do any bounds checking.
 */
#define MatNd_get2(self, row, col)              \
  self->ele[(row) * self->n + (col)]



#ifdef __cplusplus
extern "C" {
#endif





/**
 * @name CreationAndDestruction
 *
 * Creation and destruction
 */

///@{

/*! \ingroup MatNdFunctions
 *  \brief Allocates an MatNd array with m*n elements, sets
 *         the row number to m, the column number to n, and
 *         initializes all elements with 0.
 */
MatNd* MatNd_create(unsigned int m, unsigned int n);

/*! \ingroup MatNdFunctions
 *  \brief Returns a deep copy of src.
 *  \return Deep copy of src. If src is NULL, the function returns NULL.
 */
MatNd* MatNd_clone(const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Releases the memory for array self. If self is NULL, nothing
 *         will be done.
 */
void MatNd_destroy(MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Convenience function for releasing memory for several arrays. Use
 *         like: MatNd_destroyN(3, mat1, mat2, mat3); This function is not
 *         very safe. You must make sure that there are as many MatNd pointers
 *         after the nToDestroy as nToDestroy is. The function does not do
 *         any type checking neither. If you call it not properly, the
 *         behavior is undefined.
 */
void MatNd_destroyN(unsigned int nToDestroy, ...);

/*! \ingroup MatNdFunctions
 *  \brief Reallocates the MatNd to hold memory for m*n values. All original
 *         values of the array are preserved. If there are new elements, they
 *         are initialized with zero. If the array self has been allocated on
 *         the stack, the function exits fatally if a resize is necessary. The
 *         function also exits fatally if the call to realloc fails. If self
 *         is NULL, a new MatNd is returned. The return value is the new
 *         memory. The memory pointed to by self must be ignored, it may be
 *         invalid after calling this function.
 */
MatNd* MatNd_realloc(MatNd* self, unsigned int m, unsigned int n);

/*! \ingroup MatNdFunctions
 *  \brief Creates an array from a file. If the file doesn't exist or is
 *         malformed, the function returns NULL and complains on debug level
 *         1.
 */
MatNd* MatNd_createFromFile(const char* fileName);

/*! \ingroup MatNdFunctions
 *  \brief Creates a MatNd from a string. See MatNd_fromString.
 */
MatNd* MatNd_createFromString(const char* str);

/*! \ingroup MatNdFunctions
 * \brief Use MatNd_create to create a new matrix, filled with zeros,
 *        that has the same shape as the given matrix.
 *
 * \param[in] src input matrix
 * \return output matrix
 */
MatNd* MatNd_createLike(const MatNd* src);

/*!\ingroup MatNdFunctions
 * \brief Use MatNd_create to create a new matrix, filled with zeros, that has
 *        the same shape as the given matrix.
 *
 * \param[in] src input matrix
 * \param[out] dst output matrix
 */
void MatNd_createLike2(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \hideinitializer
 *  \brief see \ref MatNd_create.
 *         This version allocates memory on the stack if the array size is
 *         smaller than \ref MATND_MAX_STACK_MATRIX_SIZE. The memory is set to
 *         zero.
 */
#define MatNd_create2(self,rows,cols)            \
  if ((rows)*(cols)>MATND_MAX_STACK_MATRIX_SIZE) \
  {                                              \
    self = MatNd_create(rows,cols);              \
  }                                              \
  else                                           \
  {                                              \
    MatNd_fromStack(self,rows,cols);             \
  }

/*! \ingroup MatNdFunctions
 *  \hideinitializer
 *  \brief see \ref MatNd_create2.
 *         This version allocates memory on the stack. The memory is set to
 *         zero.
 */
#define MatNd_fromStack(self,rows,cols)                           \
  {                                                               \
    unsigned int size = (rows)*(cols) == 0 ? 1 : (rows)*(cols);   \
    self = (MatNd*) alloca(sizeof(MatNd) + size*sizeof(double));  \
    MATND_COUNT_ALLOCA_BYTES_ROW_X_COL(rows,cols);                \
    self->ele = (double*) (self + 1);                             \
    self->m = rows;                                               \
    self->n = cols;                                               \
    self->size = size;                                            \
    self->stackMem = true;                                        \
    memset(self->ele, 0, size*sizeof(double));                    \
  }


/*! \ingroup MatNdFunctions
 *  \hideinitializer
 *  \brief see \ref MatNd_clone.
 *         This version allocates on the stack if the array
 *         size is smaller than \ref MATND_MAX_STACK_MATRIX_SIZE.
 */
#define MatNd_clone2(self,src)                                         \
  if (src->size>MATND_MAX_STACK_MATRIX_SIZE)                           \
  {                                                                    \
    self = MatNd_clone(src);                                           \
  }                                                                    \
  else                                                                 \
  {                                                                    \
    self = (MatNd*) alloca(sizeof(MatNd) + src->size*sizeof(double));  \
    MATND_COUNT_ALLOCA_BYTES_SIZE(src);                                \
    self->ele = (double*) (self + 1);                                  \
    self->m = src->m;                                                  \
    self->n = src->n;                                                  \
    self->size = src->size;                                            \
    self->stackMem = true;                                             \
    memcpy(self->ele, src->ele, src->size*sizeof(double));             \
  }


///@}





/**
 * @name Print functions
 *
 * Plotting, printing and formatted output
 */

///@{

/*! \ingroup MatNdFunctions
 *  \brief Prints the array to stderr.
 */
void MatNd_print(const MatNd* M);

/*! \ingroup MatNdFunctions
 *  \brief Prints the transposed array to stderr.
 */
void MatNd_printTranspose(const MatNd* M);

/*! \ingroup MatNdFunctions
 *  \brief Prints the array to stderr, including information about size,
 *         rows and columns.
 */
void MatNd_printDetail(const MatNd* M);

/*! \ingroup MatNdFunctions
 *  \brief Prints the array with n digits after the colon to stderr.
 */
void MatNd_printDigits(const MatNd* M, int digits);

/*! \ingroup MatNdFunctions
 *  \brief Prints the array with a comment. If comment is NULL, it will be
 *         skipped.
 */
void MatNd_printComment(const char* comment, const MatNd* M);

/*! \ingroup MatNdFunctions
 *  \brief Prints the array with a comment and n digits after the colon to
 *         stderr. If comment is NULL, it will be skipped.
 */
void MatNd_printCommentDigits(const char* comment, const MatNd* M,
                              unsigned int digits);

/*! \ingroup MatNdFunctions
 *  \brief Prints the array with a comment and the specified format to
 *         stderr. If comment is NULL, it will be skipped.
 */
void MatNd_printFormatted(const char* text, const char* format,
                          const MatNd* M);

/*! \ingroup MatNdFunctions
 *  \brief Prints the dimensions of the array to stderr.
 */
void MatNd_printDims(const char* comment, const MatNd* M);

/*! \ingroup MatNdFunctions
 *  \brief Prints two arrays next to each other. The arrays must have the
 *         same number of rows.
 */
void MatNd_printTwoArrays(const MatNd* v1, const MatNd* v2, int digits);

/*! \ingroup MatNdFunctions
 *  \brief Prints two arrays and their difference next to each other. The
 *         arrays must have the same number of rows and clumns.
 */
void MatNd_printTwoArraysDiff(const MatNd* v1, const MatNd* v2, int digits);

/*! \ingroup MatNdFunctions
 *  \brief Prints three arrays next to each other. The arrays must have the
 *         same number of rows.
 */
void MatNd_printThreeArrays(const MatNd* v1, const MatNd* v2,
                            const MatNd* v3, int digits);

/*! \ingroup MatNdFunctions
 *  \brief Plots the columns of the array in a Gnuplot window.
 */
bool MatNd_gnuplot(const char* title, const MatNd* self);

///@}





/**
 * @name Linear algebra
 *
 * Decompositions, inverses and solving linear equations systems
 */

///@{


/*! \addtogroup LinearAlgebra
 *  \brief Computes the Cholesky decomposition following the algorithm by
 *         Tadeusz Banachiewicz Cholesky (Row-wise decomposition). It is taken
 *         from: Locher, Numerische Mathematik fuer Informatiker, 2nd edition,
 *         Springer Verlag, p. 248.
 *         \f$
 *         \mathbf{A} = \mathbf{L} \; \mathbf{L}^T
 *         \f$ .
 *         Matrix \f$ \mathbf{A} \f$ must be square and positive definite.
 *         Matrix \f$ \mathbf{L} \f$ is a lower
 *         triangular matrix. The function returns the determinant of
 *         \f$\mathbf{A}\f$. If it is 0.0, the decomposition failed and
 *         \f$\mathbf{L}\f$ is undefined. A code example is in the tests.
 *         If the dimension of \f$\mathbf{A}\f$ is smaller or equal
 *         MATND_MAX_STACK_VECTOR_SIZE doubles, no heap memory will be
 *         allocated. If the matrix is of size 0 x 0, the determinant is 1.
 */
double MatNd_choleskyDecomposition(MatNd* L, const MatNd* A);

/*! \ingroup MatNdFunctions
 *  \brief Inverts a square symmetric matrix using the Cholesky
 *         decomposition. The function returns the determinant of the matrix.
 *         If the matrix has no inverse (determinant is 0), the function will
 *         emit a warning on debug level 4, and set the inv array to 0. On
 *         debug level 5, the matrix is printed to the console.
 *         It is allowed to invert a matrix in place: inv may be the same as
 *         src. If the matrix dimension is 0 x 0, the determinant is set to 1.
 *         If the dimension of A is smaller or equal
 *         MATND_MAX_STACK_VECTOR_SIZE doubles, no heap memory will be
 *         allocated. If the matrix is of size 0 x 0, the determinant is 1.
 */
double MatNd_choleskyInverse(MatNd* inv, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Inverts a square matrix using Gaussian Elimination. The determinant
 *         of the matrix is returned. The function may be called with src and
 *         inv being the same. In this case, the function does an in-place
 *         inversion.
 *
 *  \param[out] inv   Inverse of src with the same dimensions. If the inverse
 *                    does not exist, inv is set to zero, and a determinant of
 *                    zero is returned.
 *                    dimensions don't match, the function exits fatally.
 *  \param[in] src    Square matrix to be inverted
 *  \return Determinant of src. In case the matrix is not invertible, it is 0.
 */
double MatNd_gaussInverse(MatNd* inv, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Inverts the diagonal of a matrix. Source and destination must be
 *         either a row vector or square, and have the same size. If a vector
 *         is passed a vector with elements 1/(element + lambda) is returned.
 *         For a square matrix the elements on the diagonal are inverted as
 *         above and all other entries are set to zero. If a diagonal element
 *         is zero, its inverse is set to DBL_MAX. Safe to have same matrix
 *         for src and dst. The determinant of the matrix is returned.
 *
 *  \param[out] inv   Inverse of src with the same dimensions.
 *  \param[in]  src   Row vector or square matrix to be inverted.
 *  \return Determinant of src. It is the product of the diagonal terms.
 */
double MatNd_inverseDiag(MatNd* inv, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Computes the solution x for the linear equation system
 *         \f$
 *         \mathbf{A x} = \mathbf{b}
 *         \f$
 *         using the Cholesky decomposition.
 *         Matrix \f$\mathbf{A}\f$ must be square and positive definite. The
 *         function returns the determinant of \f$\mathbf{A}\f$. If it is 0,
 *         \f$\mathbf{x}\f$ will be set to 0 and a warning
 *         will be issued on debug level 4. A code example is in the tests.
 *         If the dimension of \f$\mathbf{A}\f$ is smaller or equal
 *         MATND_MAX_STACK_VECTOR_SIZE doubles, no heap memory will be
 *         allocated.
 *
 *  \param[out] x   Solution vector or matrix of dimension n x d, where n is
 *                  the dimension of the square matrix A. Typically, n is 1,
 *                  however, the function can also solve for several right hand
 *                  side vectrs simultaneously. In this case, x and b must have
 *                  the same number of columns.
 *  \param[in]  A   Square matrix
 *  \param[in]  b   Right hand side vector or matrix, must have same
 *                  dimensions as x.
 *  \return Determinant of A.
 */
double MatNd_choleskySolve(MatNd* x, const MatNd* A, const MatNd* b);

/*! \ingroup MatNdFunctions
 *  \brief Solves A x = b using the iterative Gauss-Seidel algorithm.
 *
 *         Matrix A must be square. This algorithm works iteratively. This
 *         means if vector x is initialized closely to the solution, only a
 *         few iterations are needed. In this case, the algorithm might be
 *         faster than other solvers. eps is the stopping criterion, maxIter
 *         indicate the maximal number of iterations, maxIter < 1 sets it
 *         to unlimited.
 *
 *  \param[in,out] x    Solution vector or of dimension n x 1, where n is the
 *                      dimension of the square matrix A.
 *  \param[in]  A       Square matrix
 *  \param[in]  b       Right hand side vector, must have same dimensions as x.
 *  \param[in]  eps     Stopping criterion: Permissable difference of each
 *                      element of x between two iterations. If one component
 *                      falls below eps, the algorithm stops.
 *  \param[in]  maxIter Maximum permissible iterations before the algorithm
 *                      stops. If maxIter is less than 1, it will iterate until
 *                      converging to eps, or forever.
 *  \return Number of iterations. If the algorithm did not converge, it
 *          returns maxIter.
 */
int MatNd_gaussSeidelIterate(MatNd* x, const MatNd* A, const MatNd* b,
                             const double eps, const int maxIter);

/*! \ingroup MatNdFunctions
 *  \brief Solves A x = b using the iterative Gauss-Seidel algorithm. It calls
 *         \ref MatNd_gaussSeidelIterate() with maxIter=-1 and eps=1.0e-8.
 */
int MatNd_gaussSeidelSolve(MatNd* x, const MatNd* A, const MatNd* b);

/*! \ingroup MatNdFunctions
 *  \brief Generalized pseudo-inverse with weighting matrices for both
 *         dimensions. This function decides for the type of inverse by the
 *         number of rows and columns of argument J. The more efficient
 *         inverse is utilized.
 *
 *  Right inverse: J1# = Wq J^T (J Wq J^T + invWx)^-1     -> MatNd_rwPinv()
 *  Left inverse:  J2# = (J^T Wx J + invWq)^-1 J^T Wx     -> MatNd_rwPinv2()
 *
 */
double MatNd_generalizedInverse(MatNd* pinvJ, const MatNd* J,
                                const MatNd* Wx, const MatNd* invWq);

/*! \ingroup MatNdFunctions
 *  \brief Computes the right weighted pseudo-inverse in the form <br> <br>
 *         \f$
 *         \mathbf{J^{\#} =  W^{-1} J^T (J W^{-1} J^T}
 *         + \mathsf{diag} (\mbox{\boldmath$\lambda$}))^{-1}
 *         \f$
 *         <br><br>
 *         Array \f$ \mathbf{W}^{-1} \f$ must be either NULL or a vector of
 *         dimensions m x 1 (m is the number of rows of \f$ \mathbf{J} \f$).
 *         If \f$ \mathbf{W}^{-1} \f$ is NULL, the weight matrix is assumed
 *         to be an identity matrix. Array \f$ \mbox{\boldmath$\lambda$}\f$
 *         may be a positive definite m x m regularization matrix, a
 *         m x 1 regularization vector or a 1 x 1 regularization
 *         scalar. If it is NULL, it is assumed to be a null vector. The
 *         function returns the determinant of the square matrix to be
 *         inverted. If the matrix J is singular (determinant is 0), matrix
 *         \f$\mathbf{J^{\#}} \f$ will be set to a zero-matrix of the proper
 *         dimensions.
 *         If the size of m*n and m*m is smaller or equal
 *         MATND_MAX_STACK_MATRIX_SIZE doubles, no heap memory will be
 *         allocated.
 */
double MatNd_rwPinv(MatNd* pinvJ, const MatNd* J, const MatNd* invW,
                    const MatNd* lambda);

/*! \ingroup MatNdFunctions
 *  \brief Computes the left weighted pseudo-inverse in the form <br> <br>
 *         \f$
 *         \mathbf{J^{\#} =  (J^T W_x J }
 *         + \mathsf{diag} (\mbox{\boldmath$\lambda$}))^{-1}
 *         \mathbf{J^T W_x}
 *         \f$
 *         <br><br>
 *         Array \f$ \mathbf{W_x} \f$ must be a vector of dimensions m x 1
 *         (m is number of rows of \f$ \mathbf{J} \f$), it is interpreted as
 *         the trace of the diagonal weighting matrix. If \f$\mathbf{W_x}\f$
 *         is NULL, it is assumed to be of the proper size with all elements
 *         being 1.0. If \f$\mbox{\boldmath$\lambda$}\f$ is not NULL, it is
 *         depending on its dimension assumed either as a positive definite
 *         n x n regularization matrix, a  n x 1 regularization vector,
 *         or a 1 x 1 regularization scalar to be added to (the main diagonal
 *         of) \f$ \mathbf{J}^{T} \mathbf{W_x} \mathbf{J}\f$ . If it has
 *         different dimensions, the function will quit with a fatal error.
 *         If the matrix J is singular (determinant is 0), matrix J_pinv will
 *         be set to a zero-matrix of the proper dimensions.
 *         If the size of m*n and n*n is smaller or equal
 *         MATND_MAX_STACK_MATRIX_SIZE doubles, no heap memory will be
 *         allocated.
 */
double MatNd_rwPinv2(MatNd* J_pinv, const MatNd* J, const MatNd* Wx,
                     const MatNd* lambda);

/*! \ingroup MatNdFunctions
 *  \brief Same as \ref MatNd_rwPinv2, but buf is used as temporary storage.
 *         Array buf must be of size n*m + n*n (or larger).
 */
double MatNd_rwPinv2_(MatNd* J_pinv, const MatNd* J, const MatNd* Wx,
                      const MatNd* lambda, MatNd* buf);

/*! \ingroup MatNdFunctions
 *  \brief Miller matrix inversion for a sum of symmetric matrices. There's
 *         an Octave example in the source code documentation of this
 *         function.
 *
 *     See: Kenneth S. Miller: On the Inverse of the Sum of Matrices,
 *          Mathematics Magazine, Vol. 54, No. 2 (Mar., 1981), pp. 67-72.
 */
void MatNd_MillerPinv(MatNd* pinvJ, const MatNd* J, const MatNd* Wx,
                      const MatNd* lambda);

/*! \ingroup MatNdFunctions
 *  \brief Inverts a 2 x 2 matrix using the analytic textbook solution. The
 *         determinant of the matrix is returned.
 */
double MatNd_inverse2D(MatNd* invA, const MatNd* A);

/*! \ingroup MatNdFunctions
 *  \brief In-place inversion of A. See MatNd_inverse2D().
 */
double MatNd_inverse2DSelf(MatNd* A);

/*! \ingroup MatNdFunctions
 *  \brief Returns the determinant of a square matrix using LU decomposition.
 */
double MatNd_determinant(const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Computes the partial derivative of the pseudo inverse of J: <br>
 *         del_q(J#) = -J# del_q(J) J# + N invW del_q(J^T) (J invW J^T)^-1
 *
 *  \param[out]  dqJpinv  Pointer to which result will be copied. It must
 *                        have memory for nx*nq*nq values. It will be
 *                        reshaped to [nx*nq, nq], where nx is the Jacobians
 *                        row size, and nq is \ref RcsGraph::nJ.
 *  \param[in]   J        Jacobian
 *  \param[in]   dqJ      Hessian, must be shaped to [nx*nq, nq].
 *  \param[in]   invW     Joint space metric (nq x 1 vector). If it is NULL,
 *                        an identity scaling is assumed.
 *  \param[in]   invJWJt  Expression (J invW J^T)^-1. If it is NULL, it will
 *                        be computed.
 *  \param[in]   Jpinv    Pseudo-inverse of J, weighted with invWq. If it is
 *                        NULL, it will be computed.
 *  \param[in] transposed If true, the result is transposed.
 */
void MatNd_PinvHessian(MatNd* dqJpinv, const MatNd* J, const MatNd* dqJ,
                       const MatNd* invW, const MatNd* invJWJt,
                       const MatNd* Jpinv, bool transposed);

/*! \ingroup MatNdFunctions
 *  \brief Computes the partial derivative of the pseudo inverse
 *         \f$
 *         \mathbf{J^{\#} =  (J^T W_x J }
 *         + \mathsf{diag} (\mbox{\boldmath$\lambda$}))^{-1}
 *         \mathbf{J^T W_x}
 *         \f$:
 *         <br>
 *         \f$
 *         \mathbf{ \partial_q (J^{\#}) = -J^{\#} \partial_q (J) J^{\#} +
 *         (J^T W_x J)^{-1} (\partial_q J)^T W_x (I - J J^{\#}) }
 *         \f$
 *
 *  \param[out] dqJpinv   Pseudo-inverse derivative. It must have memory for
 *                        nx*nq*nq values. It will be reshaped to [nx*nq, nq],
 *                        where nx is the Jacobians row size, and nq is
 *                        \ref RcsGraph::nJ.
 *  \param[in] J          Jacobian \f$ \mathbf{ J } \f$
 *  \param[in] dqJ        Hessian \f$ \mathbf{ \partial_q J } \f$
 *  \param[in] invWx      Task space metric \f$ \mathbf{ W_x } \f$ (nx x 1
 *                        vector). If it is NULL, an identity scaling is
 *                        assumed.
 *  \param[in] invJtWJ    Expression \f$ \mathbf{ (J^T W_x J)^{-1} } \f$ .
 *                        If it is NULL, it will be computed.
 *  \param[in] Jpinv      Pseudo-inverse of J, weighted with invWx. If it is
 *                        NULL, it will be computed.
 *  \param[in] transposed If true, the result is transposed.
 */
void MatNd_PinvHessian2(MatNd* dqJpinv, const MatNd* J, const MatNd* dqJ,
                        const MatNd* invWx, const MatNd* invJtWJ,
                        const MatNd* Jpinv, bool transposed);

///@}





/**
 * @name File IO
 *
 * Reading from and writing to files
 */

///@{

/*! \ingroup MatNdFunctions
 *  \brief Prints the array to a file with the given file name.
 *  \return True for success, false otherwise. In the faliure case, a debug
 *          message is printed to the console on debug levels 1 or higher.
 */
bool MatNd_toFile(const MatNd* M, const char* fileName);

/*! \ingroup MatNdFunctions
 *  \brief Appends the array to a file. If the file doesn't exist, it will be
 *         created.
 */
void MatNd_appendToFile(const MatNd* M, const char* fileName);

/*! \ingroup MatNdFunctions
 *  \brief Counts rows and columns of an array in a file. Returns true for
 *         success, false otherwise. False means that the data in the file
 *         is malformed (different numbers of columns). A header line is
 *         counted for each line that does not begin with a
 *         - digit
 *         - white space
 *         - plus sign
 *         - minus sign
 *         - dot
 *         - e
 *         The file descriptor is moved to the position after the header
 *         lines.
 */
bool MatNd_arraySizeFromFile(FILE* fd, int* rows, int* cols,
                             int* nHeaderLines);

/*! \ingroup MatNdFunctions
 *  \brief Counts rows and columns of an array in a file. Returns true for
 *         success, false otherwise. False means that the data in the file
 *         is malformed (different numbers of columns). A header line is
 *         counted for each line that does not begin with a
 *         - digit
 *         - white space
 *         - plus sign
 *         - minus sign
 *         - dot
 *         - e
 *         This is a convenience function that takes a filename as an
 *         argument and can be used for counting only if the file descriptor
 *         is not needed outside.
 */
bool MatNd_arraySizeFromFilename(const char* filename, int* rows, int* cols,
                                 int* nHeaderLines);


/*! \ingroup MatNdFunctions
 *  \brief Reads an array from a file. If the file doesn't exist, or the file
 *         is malformed, the function returns false and complains on debug
 *         level 1. A code example is in the tests.
 */
bool MatNd_fromFile(MatNd* M, const char* fileName);

///@}





/**
 * @name Matrix multiplications
 *
 * Functions to multiplay matrices in various orders
 */

///@{

/*! \ingroup MatNdFunctions
 *  \brief C = A * B.
 */
void MatNd_mul(MatNd* C, const MatNd* A, const MatNd* B);

/*! \ingroup MatNdFunctions
 *  \brief C = C + A * B.
 */
void MatNd_mulAndAddSelf(MatNd* C, const MatNd* A, const MatNd* B);

/*! \ingroup MatNdFunctions
 *  \brief \f$\mathbf{C} = \mathbf{A  B A^T}\f$
 *
 *         This function works for matrix B
 *         - being square
 *         - being a vector (A->n x 1)
 *         - being a scalar value (1 x 1)
 *         - being NULL (A * A^T then)
 *         If the size of m*n of A and B is smaller or equal
 *         MATND_MAX_STACK_MATRIX_SIZE doubles, no heap memory will be
 *         allocated.
 */
void MatNd_sqrMulABAt(MatNd* C, const MatNd* A, const MatNd* B);

/*! \ingroup MatNdFunctions
 *  \brief \f$\mathbf{C} = \mathbf{A^T  B A}\f$
 *
 *         This function works for matrix \f$\mathbf{B}\f$
 *         - being square
 *         - being a vector (A->m x 1)
 *         - being NULL (\f$\mathbf{A^T  A}\f$ then).
 *
 *         If the size of m*n of \f$\mathbf{A}\f$ and \f$\mathbf{B}\f$ is
 *         smaller or equal MATND_MAX_STACK_MATRIX_SIZE doubles, no heap
 *         memory will be allocated.
 */
void MatNd_sqrMulAtBA(MatNd* C, const MatNd* A, const MatNd* B);

/*! \ingroup MatNdFunctions
 *  \brief \f$\mathbf{C} = \mathbf{C + A^T  B A}\f$
 *         See MatNd_sqrMulABAt().
 *         If the size of n*n of A is smaller or equal
 *         MATND_MAX_STACK_MATRIX_SIZE doubles, no heap memory will be
 *         allocated.
 */
void MatNd_sqrMulAndAddAtBA(MatNd* C, const MatNd* A, const MatNd* B);

/*! \ingroup MatNdFunctions
 *  \brief Rotates a 3xn array about the rotation matrix of A_KI:
 *         self = A_KI*self
 */
void MatNd_rotateSelf(MatNd* self, double A_KI[3][3]);

/*! \ingroup MatNdFunctions
 *  \brief Rotates a 3xn array about the transpose rotation matrix of A_KI:
 *         self = transpose(A_KI)*self
 */
void MatNd_invRotateSelf(MatNd* self, double A_KI[3][3]);

/*! \ingroup MatNdFunctions
 *  \brief A = B*A.
 *         If the size of A is smaller or equal
 *         MATND_MAX_STACK_MATRIX_SIZE doubles, no heap memory will be
 *         allocated.
 */
void MatNd_preMulSelf(MatNd* A, const MatNd* B);

/*! \ingroup MatNdFunctions
 *  \brief A = A*B.
 *         If the size of A is smaller or equal
 *         MATND_MAX_STACK_MATRIX_SIZE doubles, no heap memory will be
 *         allocated.
 */
void MatNd_postMulSelf(MatNd* A, const MatNd* B);

/*! \ingroup MatNdFunctions
 *  \brief Computes the dyadic product: dst = src^T * src.
 *         If m*n of src is smaller or equal MATND_MAX_STACK_MATRIX_SIZE
 *         doubles, no heap memory will be allocated.
 */
void MatNd_dyadicProduct(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Pre-multiplies dst with a diagonal matrix with the elements
 *         stored in vec: dst = vec*dst
 *         Array vec must have 1 row or the same number of rows as dst,
 *         and 1 column. Each row is multiplied with the corresponding
 *         value of vec.
 */
void MatNd_preMulDiagSelf(MatNd* dst, const MatNd* vec);

/*! \ingroup MatNdFunctions
 *  \brief Post-multiplies dst with a diagonal matrix with the elements
 *         stored in vec: dst = dst*vec
 *         Array vec must have 1 row or as many rows as there is columns
 *         in dst, and 1 column. Each row of dst is element-wise multiplied
 *         with vec. A code example is in the tests.
 */
void MatNd_postMulDiagSelf(MatNd* dst, const MatNd* vec);

///@}





/**
 * @name Column operations
 *
 * Column operations
 */

///@{

/*! \ingroup MatNdFunctions
 *  \brief Copies a set of columns from an array. The memory pointed to by
 *         cols must hold an index array of the columns to be used, which is
 *         terminated with a  value of -1. It must not point to NULL. The
 *         number of columns must not exceed the columns of the array dst.
 *         The order of the columns given in cols is preserved. It is also
 *         possible to have the same column several times. A code example
 *         is in the tests.
 */
void MatNd_copyColumns(MatNd* dst, const MatNd* src, const int* cols);

/*! \ingroup MatNdFunctions
 *  \brief Copies one column from the source to the destination matrix
 */
void MatNd_copyColumn(MatNd* dst, unsigned int dst_column,
                      const MatNd* src, unsigned int src_column);

/*! \ingroup MatNdFunctions
 *  \brief Sets the n elements of the c-th column of A with the values
 *         pointed to by p. The function will exit with a fatal error if
 *         - c is equal or larger than A->n
 *         - n is larget than A->m.
 */
void MatNd_setColumn(MatNd* A, int c, const double* p, int n);

/*! \ingroup MatNdFunctions
 *  \brief Appends the columns of src to the last column of dst. This is a
 *         convenience function and calls MatNd_insertColumns().
 */
void MatNd_appendColumns(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Adds the n elements of the c-th column of A with the values
 *         pointed to by p. The function will exit with a fatal error if
 *         - c is equal or larger than A->n
 *         - n is larget than A->m.
 */
void MatNd_addToColumn(MatNd* A, int c, const double* p, int n);

/*! \ingroup MatNdFunctions
 *  \brief Sets the elements of the column-th column of self with the values.
 */
void MatNd_setColumnToValue(MatNd* self, unsigned int column, double value);

/*! \ingroup MatNdFunctions
 *  \brief Sets all elements of the respective column to zero.
 */
void MatNd_setColumnZero(MatNd* M, unsigned int column);

/*! \ingroup MatNdFunctions
 *  \brief Copies the c-th column of A into B. Array B will be
 *         reshaped to match the dimensions. If c is out of range,
 *         the function will exit fatally.
 */
void MatNd_getColumn(MatNd* B, int c, const MatNd* A);

/*! \ingroup MatNdFunctions
 *  \brief Deletes the index-th column and reshapes the array.
 */
void MatNd_deleteColumn(MatNd* self, int index);

/*! \ingroup MatNdFunctions
 *  \brief Deletes the columns from first to last. First and last are
 *         included. The array is automatically reshaped.
 */
void MatNd_deleteColumns(MatNd* self, int first, int last);

/*! \ingroup MatNdFunctions
 *  \brief Returns the sum of the elements of the column-th column. If column
 *         is out of range, the function will exit with a fatal error.
 */
double MatNd_columnSum(const MatNd* self, unsigned int column);

/*! \ingroup MatNdFunctions
 *  \brief Performs the cross product on each column of a 3 x n array:
 *         column_i(dst) = vec x column_i(src)
 *         The function allows an in-place operation on dst. For
 *         convenience please use MatNd_columnCrossProductSelf().
 */
void MatNd_columnCrossProduct(MatNd* dst, const MatNd* src,
                              const double vec[3]);

/*! \ingroup MatNdFunctions
 *  \brief Performs the cross product on each column of a 3xn array:
 *         column_i(self) = vec x column_i(self)
 */
void MatNd_columnCrossProductSelf(MatNd* self, const double vec[3]);

/*! \ingroup MatNdFunctions
 *  \brief Does the same as MatNd_insertRows(), just for the transposed
 *         case. A code example is in the tests.
 *         If the size of m*n doubles of "from" is smaller or equal
 *         MATND_MAX_STACK_MATRIX_SIZE doubles, no heap memory will be
 *         allocated.
 */
void MatNd_insertColumns(MatNd* self, int colDst, const MatNd* from,
                         int colSrc, int nRows);

/*! \ingroup MatNdFunctions
 *  \brief Interprets the columns of src as a time series and copies its
 *         numerical derivative (1st order Euler approximation) into dst.
 *         The derivative is computed as dx = x[k+1] - x[k]. The last
 *         row is copied from the second last one.
 */
void MatNd_columnDerivative(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Interprets the columns of x as a time series and copies its
 *         numerical integral (sum) into x_int. If array x0 is not NULL, it
 *         is the initial value of x_int. In this case, x0 must have 1 row
 *         and the same number of columns as x, otherwise the function exits
 *         with a fatal error.
 */
void MatNd_columnIntegral(MatNd* x_int, const MatNd* x, const MatNd* x0);

/*! \ingroup MatNdFunctions
 *  \brief Interprets the columns of x as a time series and copies its
 *         absolute lengths into x_length.
 */
void MatNd_columnLength(MatNd* x_length, const MatNd* x);

///@}






/**
 * @name Row operations
 *
 * Row operations
 */

///@{

/*! \ingroup MatNdFunctions
 *  \brief Deletes the index-th row and reshapes the array.
 */
void MatNd_deleteRow(MatNd* self, int index);

/*! \ingroup MatNdFunctions
 *  \brief Deletes the rows from first to last. First and last are
 *         included. The array is automatically reshaped.
 */
void MatNd_deleteRows(MatNd* self, int first, int last);

/*! \ingroup MatNdFunctions
 *  \brief Sets the n elements of the r-th row of A with the values
 *         pointed to by p. If n is larger than A->n then subsequent
 *         rows are set too (but n needs to be a multiple of A->n).
 *         The memory pointed to by p may overlap with the arrays
 *         memory.
 */
void MatNd_setRow(MatNd* A, int r, const double* p, int n);

/*! \ingroup MatNdFunctions
 *  \brief Sets all elements of the respective row to zero.
 */
void MatNd_setRowZero(MatNd* M, unsigned int row);

/*! \ingroup MatNdFunctions
 *  \brief Copies the row-th column of A into B. Array B will be
 *         reshaped to match the dimensions.
 */
void MatNd_getRow(MatNd* B, int row, const MatNd* A);

/*! \ingroup MatNdFunctions
 *  \brief Returns a row-vector of the row-th row of B. The returned matrix
 *         is just a view on the row, the memory still belongs to B.
 */
MatNd MatNd_getRowView(const MatNd* B, int row);

/*! \ingroup MatNdFunctions
 *  \brief Returns a column-vector of the row-th row of B. The returned
 *         matrix is just a view on the row, the memory still belongs to B.
 */
MatNd MatNd_getRowViewTranspose(const MatNd* B, int row);

/*! \ingroup MatNdFunctions
 *  \brief Appends the rows of src to the last row of dst. This is a
 *         convenience function and calls \ref MatNd_insertRows().
 */
void MatNd_appendRows(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Inserts src before dst. This is a convenience function and calls
 *         \ref MatNd_insertRows().
 */
void MatNd_prependRows(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Insert nRows rows of array from after the rowDst-th row of self.
 *         If rowDst is -1, the rows will be prepended. Array self will be
 *         reallocated if its memory is too small.
 *         The function exits with a fatal error if
 *         - the arrays don't have the same number of columns
 *         - the row rowDst is not within [-1...self->m-1]
 *         - the block to be copied gets out of the memory of from
 */
void MatNd_insertRows(MatNd* self, int rowDst, const MatNd* from,
                      int rowSrc, unsigned int nRows);

/*! \ingroup MatNdFunctions
 *  \brief Copies every 1/ratio-th row of src to dst. Array dst will be
 *         reshaped accordingly. The value ratio must be within ]0...1[. If
 *         not, a warning is issued on debug level 1, and dst is not
 *         modified.
 */
void MatNd_condenseRows(MatNd* dst, const MatNd* src, double ratio);

/*! \ingroup MatNdFunctions
 *  \brief Perform linear interpolation to fill dst matrix from src matrix
 */
void MatNd_interpolateRows(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Perform linear interpolation to fill dst matrix from src matrix
 *         by assuming that the 3 columns of src and dst are Euler angles
 *
 */
void MatNd_interpolateRowsEuler(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Copies the interpolated row according to parameter s into dst.
 *         If s<=0, the first row is copied. If s >= 1, the last row is
 *         copied. If src has only 1 row, it is copied into dst. If src
 *         has 0 columns, the array dst is reshaped to 1 x 0, and nothing
 *         is copied.
 *
 *  \param[out] dst   Interpolated array, will be reshaped automatically
 *  \param[in]  src   Array to be interpolated. It must have at least one
 *                    row (otherwise the function will exit fatally)
 *  \param[in]  s     Interpolation value, should be between 0 and 1
 */
void MatNd_rowLerp(MatNd* dst, const MatNd* src, const double s);

///@}





/**
 * @name Querying matrices
 *
 * Functions to query and check matrices
 */

///@{

/*! \ingroup MatNdFunctions
 *  \brief Returns true if one of the arrays elements is NAN.
 *
 *  \param[in] self Array to be checked. The self->m*self->n elements
 *                  are checked.
 */
bool MatNd_isNAN(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns true if one or more of the array elements is infinite.
 *
 *  \param[in] self Array to be checked. The self->m*self->n elements
 *                  are checked.
 */
bool MatNd_isINF(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns true if all array elements are finite, false otherwise.
 *
 *  \param[in] self Array to be checked. The self->m*self->n elements
 *                  are checked.
 */
bool MatNd_isFinite(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Checks if a matrix is symmetric:
 *         \f$
 *         \mathbf{A} = \mathbf{A}^T
 *         \f$
 *         <br>
 *         Value eps is the tolerance the elements of the upper and lower
 *         triangle may differ. If the matrix is not square, the function
 *         terminates with a fatal error.
 */
bool MatNd_isSymmetric(const MatNd* A, double eps);

/*! \ingroup MatNdFunctions
 *  \brief Checks if a matrix is an identity matrix. Value eps is the
 *         threshold allowed to deviate from the identity matrix values.
 *         If the matrix is not square, the function returns false.
 */
bool MatNd_isIdentity(const MatNd* A, double eps);

/*! \ingroup MatNdFunctions
 *  \brief Checks if a matrix is diagonal. Value eps is the threshold allowed
 *         for the off-diagonal elements. If the matrix is not square, the
 *         function terminates with a fatal error.
 */
bool MatNd_isDiagonal(const MatNd* A, double eps);

/*! \ingroup MatNdFunctions
 *  \brief Returns true if the arrays are element-wise equal with a
 *         tolerance less than eps.
 */
bool MatNd_isEqual(const MatNd* m1, const MatNd* m2, double eps);

///@}





/**
 * @name Elementary functions
 *
 * Basic MatNd functions
 */

///@{

/*! \ingroup MatNdFunctions
 *  \brief Sets all elements of self to zero.
 */
void MatNd_setZero(MatNd* M);

/*! \ingroup MatNdFunctions
 *  \brief Sets all elements with an absolute value that is equal or smaller
 *         than the given zeroThreshold to zero, the others to 1. The parameter
 *         zeroThreshold is expected to be equal or larger than zero, otherwise
 *         all elements will be 1.
 */
void MatNd_binarizeSelf(MatNd* self, double zeroThreshold);

/*! \ingroup MatNdFunctions
 *  \brief Sets self to the identity matrix. Self must be square,
 *         otherwise the function exits fatally.
 */
void MatNd_setIdentity(MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Adds the identity matrix to self. Self must be square,
 *         otherwise the function exits fatally.
 */
void MatNd_addIdentity(MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Adds the value to all array elements.
 */
void MatNd_addConst(MatNd* dst, double value);

/*! \ingroup MatNdFunctions
 *  \brief Adds the value to the main diagonal. Array dst must be square,
 *         otherwise the function exits fatally.
 */
void MatNd_addConstToDiag(MatNd* dst, double value);

/*! \ingroup MatNdFunctions
 *  \brief Adds the value to the given column.
 */
void MatNd_addConstToColum(MatNd* dst, unsigned int column, double value);

/*! \ingroup MatNdFunctions
 *  \brief Sets all elements of dst to zero and the diagonal elements of
 *         dst from the vector diag. Array dst must be square, otherwise the
 *         function exits fatally . Array diag must be either 1 x 1, or
 *         dim(dst) x 1. In the first case, the value will be assigned to all
 *         elements of the main diagonal. In the second case, the vector
 *         elements will be copied to the corresponding elements of the main
 *         diagonal.
 */
void MatNd_setDiag(MatNd* dst, const MatNd* diag);

/*! \ingroup MatNdFunctions
 *  \brief Sets the diagonal elements of dst from the vector diag. Array dst
 *         must be square, otherwise the function exits fatally . Array diag
 *         must be either 1 x 1, or dim(dst) x 1. In the first case, the
 *         value will be assigned to all elements of the main diagonal. In
 *         the second case, the vector elements will be copied to the
 *         corresponding elements of the main diagonal.
 */
void MatNd_overwriteDiag(MatNd* dst, const MatNd* diag);

/*! \ingroup MatNdFunctions
 *  \brief Adds the elements of the vector diag to dst. Array dst
 *         must be square. Array diag must be either 1 x 1, or dim(dst) x 1.
 *         In the first case, the value will be added to all elements of the
 *         main diagonal. In the second case, the vector elements will be
 *         copied to the corresponding elements of the main diagonal.
 */
void MatNd_addDiag(MatNd* dst, const MatNd* diag);

/*! \ingroup MatNdFunctions
 *  \brief Sets all elements of dst to the given value.
 */
void MatNd_setElementsTo(MatNd* dst, double value);

/*! \ingroup MatNdFunctions
 *  \brief dst = m1 - m2.
 */
void MatNd_sub(MatNd* dst, const MatNd* m1, const MatNd* m2);

/*! \ingroup MatNdFunctions
 *  \brief dst = dst - m.
 */
void MatNd_subSelf(MatNd* dst, const MatNd* m);

/*! \ingroup MatNdFunctions
 *  \brief dst = m1 + m2.
 */
void MatNd_add(MatNd* dst, const MatNd* m1, const MatNd* m2);

/*! \ingroup MatNdFunctions
 *  \brief dst = dst + m.
 */
void MatNd_addSelf(MatNd* dst, const MatNd* m);

/*! \ingroup MatNdFunctions
 *  \brief Returns the Euclidean norm (L2-norm, Frobenius norm, length) of
 *         the array: <br>
 *         \f$
 *         \left| x \right| = \sqrt{x_{00}^2 + x_{01}^2 + ... +
 *                                  x_{0n}^2 + ... + x_{mn}^2 }
 *         \f$
 */
double MatNd_getNorm(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns the L1 norm of the array: <br>
 *         \f$
 *         \left| x \right| = \sum{ |x_{00}| + |x_{01}| + ... +
 *                                  |x_{0n}| + ... + |x_{mn}| }
 *         \f$
 */
double MatNd_getNormL1(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns the mean of the array.
 */
double MatNd_mean(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Copies src into dst. The arrays must have the same dimensions.
 *         The memory may overlap (internally memmove is used). Only the
 *         first m x n elements are copied (not src->size elements).
 */
void MatNd_copy(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Copies src into dst. The array dst must have enough memory to
 *         store src->m*src->n double values. Array dst is reshaped to the
 *         size of src. The memory may overlap (internally memmove is used).
 *         Only the first m x n elements are copied (not src->size elements).
 */
void MatNd_reshapeCopy(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Copies src into dst. The array dst Array dst is resized to the
 *         size of src.
 */
void MatNd_resizeCopy(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Sets the row number to m and the column number to n.
 *         m*n must be smaller or equal to the size of the array.
 */
void MatNd_reshape(MatNd* self, int m, int n);

/*! \ingroup MatNdFunctions
 *  \brief Sets the row number to m and the column number to n,
 *         and sets all elelents to 0. m*n must be smaller or
 *         equal to the size of the array.
 */
void MatNd_reshapeAndSetZero(MatNd* self, int m, int n);

/*! \ingroup MatNdFunctions
 *  \brief Assigns val to (row m, column n) of the array self. If an index
 *         is out of range, the function exits with a fatal error.
 */
void MatNd_set(MatNd* self, int m, int n, double val);

/*! \ingroup MatNdFunctions
 *  \brief Adds val to (row m, column n) of the array self. If an index
 *         is out of range, the function exits with a fatal error.
 */
void MatNd_addToEle(MatNd* self, int m, int n, double val);

/*! \ingroup MatNdFunctions
 *  \brief returns self(m,n). If an index is out of range, the function exits
 *         with a fatal error.
 */
double MatNd_get(const MatNd* self, int m, int n);

/*! \ingroup MatNdFunctions
 *  \brief returns the pointer to self(m,n). If an index is out of range, the
 *         function exits with a fatal error.
 */
double* MatNd_getElePtr(const MatNd* self, int m, int n);

/*! \ingroup MatNdFunctions
 *  \brief Swaps the element (m1,n1) with the element (m2,n2). The function
 *         checks if the indices are within the range of the matrices.
 */
void MatNd_swapElements(MatNd* A, int m1, int n1, int m2, int n2);

/*! \ingroup MatNdFunctions
 *  \brief Constructs the transpose from src into dst. The
 *         array size is automatically adjusted.
 */
void MatNd_transpose(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Transposes an array in place. The array size is automatically
 *         adjusted. The function allocates heap memory.
 */
void MatNd_transposeSelf(MatNd* dst);

/*! \ingroup MatNdFunctions
 *  \brief Multiplies all elements of an array with constant c:
 *         dst = src * c
 *         If m*n of src is smaller or equal MATND_MAX_STACK_MATRIX_SIZE
 *         doubles, no heap memory will be allocated.
 */
void MatNd_constMul(MatNd* dst, const MatNd* src, double c);

/*! \ingroup MatNdFunctions
 *  \brief dst = v1 + v2 * c
 */
void MatNd_constMulAndAdd(MatNd* dst, const MatNd* v1, const MatNd* v2,
                          double c);

/*! \ingroup MatNdFunctions
 *  \brief dst += v1 * c
 */
void MatNd_constMulAndAddSelf(MatNd* dst, const MatNd* v1, double c);

/*! \ingroup MatNdFunctions
 *  \brief Multiplies all elements of an array with constant c:
 *         self = self * c
 */
void MatNd_constMulSelf(MatNd* self, double c);

/*! \ingroup MatNdFunctions
 *  \brief Copies n elements from array p into self. The arrays'
 *         shape (m x n) is not changed. The memory pointed to by
 *         p may overlap with the arrays memory. If n is larger
 *         than the array's size, the function will exit fatally.
 *
 *  \param[out] self    Target matrix with size >=n
 *  \param[in]  p       Pointer to double array with at least n
 *                      valid values. If this is not the case, the
 *                      function behavior is undefined.
 *  \param[in]  n       Number of double values to be copied from p.
 */
void MatNd_fromArray(MatNd* self, const double* p, int n);

/*! \ingroup MatNdFunctions
 *  \brief Returns the sum of all (m x n) elements of the array.
 */
double MatNd_sumEle(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns the lowest (f)absolut value of the array. The norm of
 *          the values is used: <br>
 *        MatNd_minAbsEle([1 3 -5]) = 1 <br>
 *        MatNd_minAbsEle([-1 -3 5]) = 1.
 */
double MatNd_minAbsEle(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns the highest (f)absolut value of the array. The norm of
 *          the values is used: <br>
 *        MatNd_maxAbsEle([1 3 -5]) = 5 <br>
 *        MatNd_maxAbsEle([1 -3 5]) = 5.
 */
double MatNd_maxAbsEle(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns the smallest value of the array.
 */
double MatNd_minEle(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns the largest value of the array.
 */
double MatNd_maxEle(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns the index of the smallest absolut value.
 */
unsigned int MatNd_minAbsEleIndex(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns the index of the largest value.
 */
unsigned int MatNd_maxEleIndex(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Returns the index of the largest absolut value.
 */
unsigned int MatNd_maxAbsEleIndex(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Replaces each element of the array with the absolute value.
 */
void MatNd_fabsEleSelf(MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Raises each element to the power given in exponent.
 */
void MatNd_powEle(MatNd* dst, const MatNd* src, double exponent);

/*! \ingroup MatNdFunctions
 *  \brief Raises each element to the power given in exponent.
 */
void MatNd_powEleSelf(MatNd* self, double exponent);

/*! \ingroup MatNdFunctions
 *  \brief Element-wise logistic function: sigX = 1/(1+exp(-beta*x))
 */
void MatNd_sigmoidExponentialEle(MatNd* sigX, const MatNd* x, double beta);

/*! \ingroup MatNdFunctions
 *  \brief Scales the array such that the maximum absolut value doesn't
 *         exceed value s. The scaling factor is returned. If argument s is
 *         smaller than 0.0, it is assumed to be 0.0.
 */
double MatNd_scaleSelfToScalar(MatNd* self, double s);

/*! \ingroup MatNdFunctions
 *  \brief Scales the array such that the maximum absolut value of each
 *         element doesn't exceed the corresponding value given in limit. The
 *         scaling factor is returned. Limit is assumed to have only positive
 *         values. This is not checked. If limit is 1 x 1, then self is
 *         scaled to be below the value of limit.
 */
double MatNd_scaleSelf(MatNd* self, const MatNd* limit);

/*! \ingroup MatNdFunctions
 *  \brief Replaces self with element-wise maximum of self and other
 */
void MatNd_maxSelf(MatNd* self, const MatNd* other);

/*! \ingroup MatNdFunctions
 *  \brief Replaces self with element-wise minimum of self and other
 */
void MatNd_minSelf(MatNd* self, const MatNd* other);

/*! \ingroup MatNdFunctions
 *  \brief Saturates self element-wise to +-limit.
 *         Limit has to be either a 1x1 matrix or have the same size as self
 */
void MatNd_saturateSelf(MatNd* self, const MatNd* limit);

/*! \ingroup MatNdFunctions
 *  \brief Returns the sum of the squared distances of the array elements.
 *         The sizes of the arrays A and B must match, otherwise the function
 *         exits with a fatal error.
 */
double MatNd_sqrDistance(const MatNd* A, const MatNd* B);

/*! \ingroup MatNdFunctions
 *  \brief Returns the mean squared difference between a1 and a2. The dimensions
 *         of a1 and a2 must match, otherwise the function exits with a fatal
 *         error. If the arrays are degenerated (m=n=0), the function returns 0.
 */
double MatNd_msqError(const MatNd* a1, const MatNd* a2);

/*! \ingroup MatNdFunctions
 *  \brief Returns the root mean square difference between a1 and a2.
 */
double MatNd_rmsqError(const MatNd* a1, const MatNd* a2);

/*! \ingroup MatNdFunctions
 *  \brief Returns the pointer to the row-th row of the array.
 */
double* MatNd_getRowPtr(const MatNd* self, int row);

/*! \ingroup MatNdFunctions
 *  \brief Sets the elements of the array to uniformly distributed random
 *         numbers within the given range: lower <= result < upper. The
 *         random numbers are seeded with the computer clock, so that they
 *         do not give reproduceable results.
 *
 *         uses VecNd_setRandom()
 */
void MatNd_setRandom(MatNd* self, double lower, double upper);

/*! \ingroup MatNdFunctions
 *  \brief Element-wise multiplication of array elements dst with the
 *         elements of m. Both arrays have to have the same shape.
 */
void MatNd_eleMulSelf(MatNd* dst, const MatNd* m);

/*! \ingroup MatNdFunctions
 *  \brief Element-wise multiplication: dst = m1*m2.
 */
void MatNd_eleMul(MatNd* dst, const MatNd* m1, const MatNd* m2);

/*! \ingroup MatNdFunctions
 *  \brief Returns the variance of all m*n elements of the array: <br>
 *         \f$
 *         var = \Sigma_{i=1}^{m*n} (x - \mu)^2
 *         \f$
 */
double MatNd_variance(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Initializes a MatNd with a string. The elements must be
 *         separated with a blank, a new row is indicated with a comma. A
 *         comma must have a blank before and after.
 */
void MatNd_fromString(MatNd* J, const char* str);

/*! \ingroup MatNdFunctions
 *  \brief Writes the contents of an matrix to a string, so that it can be
 *         loaded again with MatNd_fromString()
 */
void MatNd_toString(const MatNd* A, char* str);

/*! \ingroup MatNdFunctions
 *  \brief Copies the lower triangle of src to dst. This does not include
 *         the main diagonal. The matrices must be square.
 */
void MatNd_copyLowerTriangle(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Copies the main diagonal of src to dst. The matrices must be
 *         square and have the same dimensions, otherwise the function will
 *         terminate with a fatal error.
 */
void MatNd_copyMainDiagonal(MatNd* dst, const MatNd* src);

/*! \ingroup MatNdFunctions
 *  \brief Line search algorithm adapted from Numerical Recipes in C.
 *
 *         Array x0 holds the initial condition, array dfdx the gradient
 *         of func at x0. Function func evaluates the cost for the given
 *         state. The function returns the line minimum, the respective
 *         vector is copied into x. If argument nEval is not NULL, the
 *         number of iterations is copied into it. If argument converged is
 *         not NULL, the function will set it to true if the line search has
 *         converged.
 *
 *         For more details, see
 *         Press W H, Teukolsky S A, Vetterling W T, Flannery B P.
 *         Numerical Recipes in C: the art of scientific computing.
 *         Cambrige University Press, New York, 2nd Edition, 1992 .
 *
 *  \param[out] x         Line minimum vector
 *  \param[in]  x0        Initial vector
 *  \param[in]  dfdx      Gradient direction
 *  \param[in]  func      Cost function
 *  \param[in]  data      Parameter pointer that is passed through to the
 *                        cost function
 *  \param[out] nEval     Number of iterations
 *  \param[in]  maxStep   Step length limit
 *  \param[out] converged Flag that indicates if the line search converged
 */
double MatNd_lineSearch(MatNd* x, const MatNd* x0, const MatNd* dfdx,
                        double(*func)(double[], void*), void* data,
                        int* nEval, double maxStep, bool* converged);

/*! \ingroup MatNdFunctions
 *  \brief See above, but in-place computation of x. A code example is in
 *         the tests.
 */
double MatNd_lineSearchSelf(MatNd* x, const MatNd* dfdx,
                            double(*func)(double[], void*), void* data,
                            int* nEval, double maxStep, bool* converged);

/* \ingroup MatNdFunctions
 * \brief Line search using Armijo Rule for (weak) Wolfe conditiions
 *        See also: https://en.wikipedia.org/wiki/Wolfe_conditions
 *
 * \param[in,out]       x State to start line search from
 * \param[in] searchDir Search direction for next update step
 * \param[in] objFunc   Objective function
 * \param[in] params    Pointer to parameters to be used in objFunc
 * \param[in,out] alpha Armijo parameter c2. May be NULL, then a default
 *                      of 0.9 is used. Otherwise, it should be ]1.0e-4 ... 1[
 *                      (c1 is internally set to 1.0e-4).
 * \return Cost value in line search minimum.
 */
double MatNd_lineSearchArmijo(MatNd* x, const MatNd* gradient,
                              double (*costFunction)(double[], void*),
                              void* params, double* alpha);

/*! \ingroup MatNdFunctions
 *  \brief Copies the block from src between row and column indices (m0 n0)
 *         and (m1 n1) to dst.
 */
void MatNd_copyBlock(MatNd* dst, const MatNd* src,
                     unsigned int m0, unsigned int n0,
                     unsigned int m1, unsigned int n1);

/*! \ingroup MatNdFunctions
 *  \brief Copies number_of_rows rows from src to dst, starting from indices
 *         src_start_row and dst_start_row respectively.
 */
void MatNd_copyRows(MatNd* dst, unsigned int dst_start_row, const MatNd* src,
                    unsigned int src_start_row, unsigned int number_of_rows);

/*! \ingroup MatNdFunctions
 *  \brief Copies one row from src to dst. The memory areas of src and dst
 *         may overlap. The funcion exits with a fatal error if
 *         - the number of columns of src and dst are not identical
 *         - the index src_idx is beyond the last column of src
 *         - the index dst_idx is beyond the last column of dst
 */
void MatNd_copyRow(MatNd* dst, unsigned int dst_idx, const MatNd* src,
                   unsigned int src_idx);

/*! \ingroup MatNdFunctions
 *  \brief Sets the block between row and column indices (m0 n0) and (m1 n1)
 *         to zero.
 */
void MatNd_setBlockZero(MatNd* dst, unsigned int m0, unsigned int n0,
                        unsigned int m1, unsigned int n1);

/*! \ingroup MatNdFunctions
 *  \brief Returns a MatNd with the given dimensions and the element
 *         pointer pointing to ptr. It is assumed that ptr points to
 *         memory with at leaset m*n doubles.
 *
 *  \param[in] m Number of rows
 *  \param[in] n Number of columns
 *  \param [in] ptr Pointer to memory with at leaset m*n doubles
 *  \return MatNd struct with dimension m x n pointing to ptr.
 */
MatNd MatNd_fromPtr(int m, int n, double* ptr);

/*! \ingroup MatNdFunctions
 *  \brief Returns a MatNd with the dimensions 3 x 3 and the element
 *         pointer pointing to the mat.
 *
 *  \param [in] mat 3x3 array
 *  \return MatNd struct with dimension 3 x 3 pointing to mat.
 */
MatNd MatNd_fromMat3d(double mat[3][3]);

/*! \ingroup MatNdFunctions
 *  \brief Computes the (symmetric) null space projection matrix N as <br>
 *         \f$ \mathbf{  N = I - J^{\#} \; J } \f$ <br>
 *         with \f$ \mathbf{ I } \f$ being the identity matrix and
 *         \f$ \mathbf{ J } \f$ the Jacobian.
 */
void MatNd_nullspace(MatNd* N, const MatNd* pinvJ, const MatNd* J);

/*! \ingroup MatNdFunctions
 *  \brief TODO: Please document! Please change signature so that target
 *               arguments come first (like in the other MatNd functions)
 */
void MatNd_calcMeanAndCovariance(const MatNd* A, double* mu, MatNd* sigma);

/*! \ingroup MatNdFunctions
 *  \brief Computes the line parameters A, B to fit the points given in
 *         the data array according to a least-squares fit: <br>
 *         \f$ \mathbf{y} = \mathbf{ A \; x + B } \f$
 *         <br>
 *         The data array must be a n x 2 array. The first column must hold
 *         the x-values, the second column must hold the corresponding
 *         y-values. To fit a line, at least 2 non-coinciding data points
 *         must exist. If the parameters cannot be determined, the
 *         function returns false and leaves A and B unchanged. If
 *         successful, the function returns true. The function can deal with
 *         ill-formed data points, such as lying all on a vertical line.
 *         Internally, the x- and y-coordinates are then swapped, the
 *         orthogonal line fit is computed, and the parameters are projected
 *         back.
 */
bool MatNd_lineFit2D(double* A, double* B, const MatNd* data);

/*! \ingroup MatNdFunctions
 *  \brief Computes the parabolic parapeters a, B to fit the points given
 *         in the data array according to a least-squares fit: <br>
 *         \f$ y = A \; x^2 + B \;  x + C \f$
 *         <br>
 *         The data array must be a n x 2 array. The first column must hold
 *         the x-values, the second column must hold the corresponding
 *         y-values. To fit a parabolic curve, at least 3 non-coinciding
 *         data points must exist. If the parameters cannot be determined,
 *         the function returns false and leaves A and B unchanged. If
 *         successful, the function returns true.
 */
bool MatNd_parabolicFit2D(double* A, double* B, double* C,
                          const MatNd* data);

/*! \ingroup MatNdFunctions
 *  \brief a dot b = Sum_i(a_i, b_i)
 */
double MatNd_innerProduct(const MatNd* a, const MatNd* b);

/*! \ingroup MatNdFunctions
 *  \brief phi = acos((a dot b) / ( |a| * |b|))
 */
double MatNd_diffAngle(const MatNd* a, const MatNd* b);

/*! \ingroup MatNdFunctions
 *  \brief Vector projection: Explanations from
 *         http://en.wikipedia.org/wiki/Vector_projection)
 *
 *         The vector projection of a on b is a vector c which is either
 *         null or parallel to b. More exactly,
 *
 *         c = 0 if phi = 90 degrees
 *         c and b have the same direction if 0 < phi < 90 degrees
 *         c and b have opposite directions if 90 < phi < 180 degrees
 *
 *         The function computes the vector projection as
 *
 *         c = [ (a dot b) / (b dot b) ] * b
 *
 *         The same, but less efficient (because of sqrt and vector by scalar
 *         division), is:
 *
 *         c = [ (a dot b) / |b| ] * [ b / |b| ]
 *
 *         Vectors a, b and c must have the same dimensions. Further, it is
 *         assumed that a, b, and c are column vectors (n=1). If this is not
 *         the case, the function exits fatally.
 */
void MatNd_vectorProjection(MatNd* c, const MatNd* b, const MatNd* a);

/*! \ingroup MatNdFunctions
 *  \brief Dynamic time warping function, adapted from the GSLExt library.
 *
 *         Performs dynamic time warping on a data set. Array src will be
 *         warped to array dst, the result is in array warped. The arrays
 *         hold the samples in the rows, there is as many samples as rows.
 *         Array warped is reshaped to the dimension of dst, array src may
 *         have a different number of rows (=samples). Array weight is a
 *         weight vector that modulates the warping of each data dimension.
 *         If it is NULL, a common weight of 1.0 is assumed for all
 *         dimensions. The function returns the distortion between warped
 *         and dst.
 *
 *  \param[out] warped The source matrix warped to length of the matrix dst
 *  \param[in] dst     Target matrix (Samples are in the rows)
 *  \param[in] src     Source matrix (Samples are in the rows)
 *  \param[in] weight  Weight vector for scaling the contribution of each
 *                     row
 */
double MatNd_DTW(MatNd* warped, const MatNd* dst, const MatNd* src,
                 const MatNd* weight);

/*! \ingroup MatNdFunctions
 *  \brief Apply a function to each element
 */
void MatNd_applyFctEle(MatNd* dst, double (*func)(double));

/*! \ingroup MatNdFunctions
 *  \brief Reverse the matrix row-wise --> last row will become first row,
 *         and so on.
 */
void MatNd_reverseSelf(MatNd* mat);

/*! \ingroup MatNdFunctions
 *  \brief Returns the trace of the square matrix self. If self is not
 *         square, the function exits with a fatal error.
 */
double MatNd_trace(const MatNd* self);

/*! \ingroup MatNdFunctions
 *  \brief Linear interpolation between rows. The size of the target array
 *         is set to subSteps*(rows-1)+1
 */
void MatNd_interpolateLinear(MatNd* dst, const MatNd* src,
                             unsigned int subSteps);

/*! \ingroup MatNdFunctions
 *  \brief 5th order polynomial interpolation between rows. The boundary
 *         conditions are computed so that the velocity and acceleration in
 *         the initial and final step is zero. The size of the
 *         target array is set to subSteps*(rows-1)+1
 */
void MatNd_interpolateFifthOrder(MatNd* dst, const MatNd* src,
                                 unsigned int subSteps);

/*! \ingroup MatNdFunctions
 *  \brief Moving mean filter over the rows of src. Argument size is the
 *         filter size. Inital and final conditions are preserved.
 */
void MatNd_filterMovingMean(MatNd* dst, const MatNd* src, unsigned int size);

/*! \ingroup MatNdFunctions
 *  \brief In-place version of \ref MatNd_filterMovingMean
 */
void MatNd_filterMovingMeanSelf(MatNd* self, unsigned int size);

/*! \ingroup MatNdFunctions
 *  \brief First order lag filtering of each column separately.
 *         filt_t = tmc * raw_t + (1-tmc) * filt_{t-1}
 */
void MatNd_filterFirstOrderLag(MatNd* dst, MatNd* src, double tmc);

/*! \ingroup MatNdFunctions
 *  \brief In-place version of \ref MatNd_filterFirstOrderLag
 */
void MatNd_filterFirstOrderLagSelf(MatNd* self, double tmc);

/*! \ingroup MatNdFunctions
 *  \brief Gets the interpolated row along the arc coordinate. If array s is
 *         not NULL, it must have one column, and as many rows as x. The
 *         values in s must be monotonously increasing. Result res must have
 *         one row, and as many columns as x.
 *
 *  \param[out] res    Interpolated row vector of x that corresponds to arc
 *                     coordinate s_des.
 *  \param[in] s       Normalized arc length vector. It must have 1 column,
 *                     and the same number of rows as in x. The consecutive
 *                     elements must contain the integrated arc length of
 *                     x, starting with 0 and ending with 1. It is checked
 *                     that each value is equal or larger than the previous
 *                     one. If argument s is NULL, the arc length vector is
 *                     constructed internally from the array x.
 *  \param[in] x       Input vector. The time index is in the row dimension,
 *                     the dimension of the array is in the columns.
 *  \param[in] s_des   Desired normalized arc length coordinate. If this
 *                     value is 0, the first row of x is copied to res. If
 *                     it is 1, the last row is copied to res. For anything
 *                     in between, the interpolated arc coordinate of x is
 *                     computed.
 *  \return            Normalized time where to find the result in the
 *                     array x.
 */
double MatNd_interpolateArcLength(MatNd* res, const MatNd* s, const MatNd* x,
                                  double s_des);

/*! \ingroup MatNdFunctions
 *  \brief Same as \ref MatNd_interpolateArcLength, except that the input
 *         array x must have 3 columns, and the interpolation is done by
 *         assuming the rows to be Euler-xyz coordinates (Slerp).
 */
void MatNd_interpolateArcLengthEuler(MatNd* res, const MatNd* s,
                                     const MatNd* x, double s_des);

/*! \ingroup MatNdFunctions
 *  \brief Computes a minimum jerk trajectory. If s or s_dot are NULL, it
 *         will be ignored. If both are NULL, the function will exit with a
 *         fatal error. If both arrays are not NULL, they must have the same
 *         number of rows, and one column.
 *         The velocity profile is computed such that its integral is 1. It
 *         means that the minimum jerk position will reach the value of 1 at
 *         the last entry.
 *
 *         The function follows the Morasso and Mussa-Ivaldi model, for
 *         details see:<br>
 *
 *         <b> R. Plamondon, A. M. Alimi, P. Yergeau, F. Leclerc: <br>
 *         Modelling velocity profiles of rapid movements: a comparative
 *         study <br>
 *         <i> Biological Cybernetics 69, pp. 119-128, 1993</i></b>
 *
 *  \param[out] s      Minimum jerk position profile
 *  \param[out] s_dot  Minimum jerk velocity profile
 */
void MatNd_computeMinimumJerkTrajectory(MatNd* s, MatNd* s_dot);

/*! \ingroup MatNdFunctions
 *  \brief This function calculates the number of steps required in order to
 *         to generate a minimum jerk trajectory that never exceeds the given
 *         velocity limit. This method is to be used in conjunction with
 *         MatNd_computeMinimumJerkTrajectory().
 *
 *  \param[in] max_velocity Maximum velocity that the minimum jerk trajectory
 *             should have.
 *  \return  Number of steps the minjerk trajectory needs to have to satisfy
 *           the maximum velocity constraint
 */
unsigned int minJerkTrajectoryLengthFromMaxVelocity(double max_velocity);

/*! \ingroup MatNdFunctions
 *  \brief Puts the diagonal of a square matrix into a vector. If mat is not
 *         square, or the dimensions of mat and vec don't match, the function
 *         will exit with a fatal error.
 *
 *  \param[out] vec      Row-vector holding the diagonal, must have 1 column.
 *  \param[in] mat       Square matrix.
 */
void MatNd_getDiag(MatNd* vec, const MatNd* mat);

/*! \ingroup MatNdFunctions
 *  \brief Gets all elements of matrix full where the corresponding element
 *         of logical is larger than or equal to 0.5. The returned matrix is
 *         a column vector. The matrices full and logical need to have the
 *         same dimensions. The size of subset needs to be sufficient to
 *         hold all entries of full. The order corresponds to the raw ele.
 */
void MatNd_getSubset(MatNd* subset, const MatNd* full, const MatNd* logical);

/*! \ingroup MatNdFunctions
 *  \brief Sets all elements of matrix full where the corresponding element
 *         of logical is larger than or equal to 0.5 to the values of subset.
 *         The matrices full and logical need to have the same  dimensions.
 *         The subset cannot have more entries than full.  The order
 *         corresponds to the raw ele.
 */
void MatNd_setSubset(MatNd* full, const MatNd* subset, const MatNd* logical);

/*! \ingroup MatNdFunctions
 *  \brief Returns the normalized time at which the arc length coordinate
 *         s_des of the array x is reached, given the current arc length
 *         trajectory s_curr. If s_curr is NULL, the arc length trajectory
 *         of x is computed by integrating the finite difference deltas from
 *         array x. Note that this will fail for coordinates that cannot be
 *         differentiated with a finite difference method, such as Euler
 *         angles etc.
 */
double MatNd_timeFromArcLength(const MatNd* x, double s_des,
                               const MatNd* s_curr);

/*! \ingroup MatNdFunctions
 *  \brief Manipulability index according to Yoshikawa: w = sqrt(det(J*W*J^T))
 *
 *  \param[in] J   Jacobian
 *  \param[in] W   n x 1 weight vector with n being the dimension of the
 *                 Jacobian's number of rows. If it is NULL, an identity
 *                 weighting is applied.
 *  \return        sqrt(det(J*W*J^T))
 */
double MatNd_computeManipulabilityIndex(const MatNd* J, const MatNd* W);

/*! \ingroup MatNdFunctions
 *  \brief Manipulability index gradient according to Yoshikawa. The sizes of
 *         Jacobian and Hessian must match: H->m==J->m*J->n and H->n==J->n,
 *         otherwise the function exits fatally.
 *
 *  \param[out] grad   Gradient of dimension 1 x J->n
 *  \param[in] J       Jacobian
 *  \param[in] H       Hessian
 *  \param[in] W       n x 1 weight vector with n being the dimension of the
 *                     Jacobian's number of rows. If it is NULL, an identity
 *                     weighting is applied.
 *  \return            1 x J->n gradient of the partial derivative of
 *                     sqrt(det(J*W*J^T))
 */
double MatNd_computeManipulabilityIndexGradient(MatNd* grad,
                                                const MatNd* H,
                                                const MatNd* W,
                                                const MatNd* J);

/*! Compute the column-wise softmax
 *  \f$ \sigma(x)_{i,j} = \frac{e^{x_{i,j}}}{\sum_{k=1}^{K} e^{x_{k,j}}} \f$.
 *  The entries of the resulting matrix sum to one
 *
 * \param[in]  src input matrix
 * \param[in]  beta scaling factor, as beta goes to infinity we get the argmax
 * \param[out] dst output matrix
 */
void MatNd_softMax(MatNd* dst, const MatNd* src, double beta);


///@}



#ifdef __cplusplus
}
#endif

#endif   // RCS_MATND_H
