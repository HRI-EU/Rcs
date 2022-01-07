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

#ifndef RCS_VECND_H
#define RCS_VECND_H


#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * \defgroup RcsVecNFunctions VecNd: N-dimensional vector functions
 */


/*! \ingroup RcsVecNFunctions
 *  \brief Returns the highest (f)absolute value of the array with n elements.
 */
double VecNd_maxAbsEle(const double* x, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns the highest value of the array with n elements.
 */
double VecNd_maxEle(const double* x, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns the smallest value of the array with n elements.
 */
double VecNd_minEle(const double* x, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns the index of the entry with the highest value of the array
 *         with n elements.
 */
int VecNd_indexMax(const double* x, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns the index of the entry with the lowest value of the array
 *         with n elements.
 */
int VecNd_indexMin(const double* x, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns true if the vectors are elementwise equal with a
 *         tolerance less than eps.
 */
bool VecNd_isEqual(const double* v1, const double* v2, unsigned int n,
                   double eps);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns the sum of all vector elements
 */
double VecNd_sum(const double* v, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns the number of vector elements that differ more than the
 *         threshold eps.
 */
unsigned int VecNd_numDifferentEle(const double* v, unsigned int n, double eps);

/*! \ingroup RcsVecNFunctions
 *  \brief Adds x to each element of v
 */
void VecNd_constAddSelf(double* v, double x, unsigned int n);

/*! \ingroup RcsVecNFunctions
   *  \brief Multiplies x with each element of B and writes the result to A
   */
void VecNd_constMul(double* A, const double* B, double x, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Multiplies x with each element of v
 */
void VecNd_constMulSelf(double* v, double x, unsigned int n);

/*! \ingroup RcsVecNFunctions
*  \brief dst = v1 + v2 * c
*/
void VecNd_constMulAndAdd(double* dst, const double* v1, const double* v2,
                          double c, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief dst += v1 * c
 */
void VecNd_constMulAndAddSelf(double* dst, const double* v1, double c,
                              unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Replaces each element of v with its absolute value
 */
void VecNd_absSelf(double* v, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns the length of an n-dimensional vector.
 */
double VecNd_getLength(const double* vec, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Copies n doubles from src to dst. The memory areas may overlap.
 */
void VecNd_copy(double* dst, const double* src, unsigned int n);

/*! \ingroup RcsVecNdFunctions
 *  \brief Copies the normalized vector src to dst. The length of the
 *         original vector is returned. If it is 0, no normalization
 *         takes place, and dst remains unchanged.
 */
double VecNd_normalize(double* dst, const double* src, unsigned int n);

/*! \ingroup RcsVecNdFunctions
 *  \brief Normalizes the vector to unit length. The length of the original
 *         vector is returned. If it is 0, no normalization takes place.
 */
double VecNd_normalizeSelf(double* v, unsigned int n);

/*! \ingroup RcsVecNdFunctions
 *  \brief Computes the p-norm of vector v, equal to sum(abs(v).^p)^(1/p). If
 *         p is 0, the function exits with a fatal error.
 */
double VecNd_norm(const double* v, double p, unsigned int n);

/*! \ingroup VecNdFunctions
 *  \brief Raises each element to the power given in exponent.
 */
void VecNd_powEleSelf(double* self, double exponent, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Sets all elements of the vector to 0.
 */
void VecNd_setZero(double* dst, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Sets n elements of dst to the given value.
 */
void VecNd_setElementsTo(double* dst, double value, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Sets 6 elements of dst to the given value a ... f.
 */
void VecNd_set6(double* vec, double a, double b, double c, double d, double e,
                double f);

/*! \ingroup RcsVecNFunctions
 *  \brief C = A - B
 */
void VecNd_sub(double* C, const double* A, const double* B, unsigned int n);

/*! \ingroup RcsVecNFunctions
   *  \brief A -= B
   */
void VecNd_subSelf(double* A, const double* B, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief C = A + B
 */
void VecNd_add(double* C, const double* A, const double* B, unsigned int n);

/*! \ingroup RcsVecNFunctions
   *  \brief A += B
   */
void VecNd_addSelf(double* A, const double* B, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns a clone of n elements of A. The returned pointer points
 *         to newly allocated heap memory. The caller is responsible for
 *         destruction. If memory allocation for the cloned vector fails, the
 *         function returns NULL.
 */
double* VecNd_clone(const double* A, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief returns the sum of squared difference of v1 and v2
 */
double VecNd_sqrDiff(const double* v1, const double* v2, unsigned int n);

/*! \ingroup RcsVecNFunctions
 * \brief Calculates the covariance betwenn the two vectors v1 and v2
 *
 * cov(v1,v2) = E((v1 - E(v1)) (v2 - E(v2))^T)
 * with E being the 'expected value' (i.e., the mean of a vector)
 */
double VecNd_covariance(const double* v1, const double* v2, unsigned int n);

/*! \ingroup RcsVecNFunctions
 * \brief Calculates the mean of all elements in vector v
 */
double VecNd_mean(const double* v, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns true if all elements are finite.
 */
bool VecNd_isFinite(const double* v, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns true if any element is NaN
 */
bool VecNd_isNaN(const double* v, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns true if all n elements are 0, false otherwise.
 */
bool VecNd_isZero(const double* v, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Element-wise multiplication of C = A * B.
 */
void VecNd_eleMul(double* C, const double* A, const double* B,
                  unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Multiplies n elements of array dst element-wise with src.
 */
void VecNd_eleMulSelf(double* dst, const double* src, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Prints n elements of vec to stderr.
 */
void VecNd_print(const double* vec, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Prints n elements of vec to stderr in one line.
 */
void VecNd_printTranspose(const double* vec, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Prints the difference of two vectors
 */
void VecNd_printDiff(const double* vec1, const double* vec2, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Prints v1, v2 and the difference of the two vectors
 */
void VecNd_printTwoArraysDiff(const double* vec1, const double* vec2,
                              unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Prints a comment and n elements of vec to stderr.
 */
void VecNd_printComment(const char* comment, const double* vec,
                        unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Prints the n double values of pointer vec to the string. The
 *         argument str must have enough memory to hold them. The
 *         function returns the pointer to str. The string is
 *         formatted as a row vector.
 */
const char* VecNd_toStr(const double* vec, unsigned int n, char* str);

/*! \ingroup RcsVecNFunctions
 *  \brief Computes the squared length of the vector. It is the sum of the
 *         squared elements.
 */
double VecNd_sqrLength(const double* self, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Sets the elements of the vector to uniformly distributed random
 *         numbers within the given range: lower <= result < upper. The
 *         random numbers are seeded with the computer clock, so that they
 *         do not give reproduceable results.
 */
void VecNd_setRandom(double* x, double lower, double upper,
                     unsigned int nEle);

/*! \ingroup RcsVecNFunctions
 *  \brief Adds to the elements of the vector uniformly distributed random
 *         numbers within the given range: lower <= result < upper. The
 *         random numbers are seeded with the computer clock, so that they
 *         do not give reproduceable results.
 *
 *         This is a convenience function to avoid allocating additional temp
 *         vectors if the values should just be varied a bit
 */
void VecNd_addRandom(double* x, double lower, double upper,
                     unsigned int nEle);

/*! \ingroup RcsVecNFunctions
   *  \brief Clips x element-wise to limits.
   */
void VecNd_clipSelf(double* x, double min, double max, unsigned int nEle);

/*! \ingroup RcsVecNFunctions
 *  \brief Scales the array such that the maximum absolut value of each
 *         element doesn't exceed the corresponding value given in limit. The
 *         scaling factor is returned. Limit is assumed to have only positive
 *         values. This is not checked.
 */
double VecNd_scaleSelf(double* self, const double* limit, unsigned int n);

/*! \ingroup RcsVecNFunctions
 *  \brief Returns median of a vector.
 */
double VecNd_median(double* x, unsigned int nEle);

/*! \ingroup RcsVecNdFunctions
 *  \brief Scales the vector so that its length does not exceed the limit
 *         value. If limit is smaller or equal to zero, self is set to zero.
 *         The function returns the scaling factor.
 */
double VecNd_constSaturateSelf(double* self, const double limit,
                               unsigned int nEle);

/*! \ingroup RcsVecNdFunctions
 *  \brief Makes x_out a sorted (in ascending order) array of x_in.
 */
void VecNd_sort(double* x_out, const double* x_in, unsigned int nEle);

/*! \ingroup RcsVecNdFunctions
 *  \brief Sorts the array in ascending order in place.
 */
void VecNd_sortSelf(double* x, unsigned int nEle);

/*! \ingroup RcsVecNdFunctions
 *  \brief Reverts the elements of the vector so that the last one becomes the
 *         first one etc.
 */
void VecNd_reverseSelf(double* self, unsigned int nEle);

/*! \ingroup RcsVecNdFunctions
 *  \brief Returns the inner product of two vectors.
 */
double VecNd_innerProduct(const double* v1, const double* v2, unsigned int n);



#ifdef __cplusplus
}
#endif

#endif   // RCS_VECND_H
