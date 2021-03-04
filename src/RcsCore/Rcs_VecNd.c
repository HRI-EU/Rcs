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

#include "Rcs_VecNd.h"
#include "Rcs_basicMath.h"
#include "Rcs_macros.h"
#include "Rcs_timer.h"



/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_maxAbsEle(const double* x, unsigned int n)
{
  unsigned int i;
  double res = fabs(x[0]);

  for (i = 1; i < n; i++)
  {
    if (fabs(x[i]) > res)
    {
      res = fabs(x[i]);
    }
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_maxEle(const double* x, unsigned int n)
{
  unsigned int i;
  double res = x[0];

  for (i = 1; i < n; i++)
  {
    if (x[i] > res)
    {
      res = x[i];
    }
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_minEle(const double* x, unsigned int n)
{
  unsigned int i;
  double res = x[0];

  for (i = 1; i < n; i++)
  {
    if (x[i] < res)
    {
      res = x[i];
    }
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
int VecNd_indexMax(const double* x, unsigned int n)
{
  unsigned int i;
  unsigned int res = 0;

  for (i = 0; i < n; i++)
  {
    if (x[i] > x[res])
    {
      res = i;
    }
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
int VecNd_indexMin(const double* x, unsigned int n)
{
  unsigned int i;
  unsigned int res = 0;

  for (i = 0; i < n; i++)
  {
    if (x[i] < x[res])
    {
      res = i;
    }
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool VecNd_isEqual(const double* v1, const double* v2, unsigned int n,
                   double eps)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    if (fabs(v1[i] - v2[i]) > eps)
    {
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_sum(const double* v, unsigned int n)
{
  unsigned int i;
  double s = 0;

  for (i = 0; i < n; i++)
  {
    s += v[i];
  }

  return s;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned int VecNd_numDifferentEle(const double* v, unsigned int n, double eps)
{
  unsigned int sameEle = 0;

  for (unsigned int i = 0; i < n; ++i)
  {
    for (unsigned int j = i+1; j < n; ++j)
    {
      if (fabs(v[i]-v[j])<eps)
      {
        sameEle++;
      }
    }
  }

  return n-sameEle;
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_constAddSelf(double* v, double x, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    v[i] += x;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_constMul(double* A, const double* B, double x, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    A[i] = B[i] * x;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_constMulSelf(double* v, double x, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    v[i] *= x;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_constMulAndAdd(double* dst, const double* v1, const double* v2,
                          double c, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    dst[i] = v1[i] + c*v2[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_constMulAndAddSelf(double* dst, const double* v1, double c,
                              unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    dst[i] += c*v1[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_absSelf(double* v, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    v[i] = fabs(v[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_getLength(const double* vec, unsigned int n)
{
  double sqrLen = 0.0;
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    sqrLen += vec[i] * vec[i];
  }

  return sqrt(sqrLen);
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_copy(double* dst, const double* src, unsigned int n)
{
  memmove(dst, src, n * sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_normalize(double* dst, const double* src, unsigned int n)
{
  double len = VecNd_getLength(src, n);

  if (len > 0.0)
  {
    double c = 1.0 / len;
    VecNd_constMul(dst, src, c, n);
  }

  return len;
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_normalizeSelf(double* v, unsigned int n)
{
  double len = VecNd_getLength(v, n);

  if (len > 0.0)
  {
    double c = 1.0 / len;
    VecNd_constMulSelf(v, c, n);
  }

  return len;
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_norm(const double* v, double p, unsigned int n)
{
  RCHECK(p != 0.0);

  double no = 0.0;

  for (unsigned int i = 0; i < n; ++i)
  {
    no += pow(fabs(v[i]), p);
  }

  return pow(no, 1.0/p);
}


/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_powEleSelf(double* self, double exponent, unsigned int n)
{
  for (unsigned int i = 0; i < n; i++)
  {
    self[i] = pow(self[i], exponent);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_setZero(double* dst, unsigned int n)
{
  memset(dst, 0, n * sizeof(double));
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_setElementsTo(double* dst, double value, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    dst[i] = value;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_set6(double* vec,
                double a, double b, double c, double d, double e, double f)
{
  vec[0] = a;
  vec[1] = b;
  vec[2] = c;
  vec[3] = d;
  vec[4] = e;
  vec[5] = f;
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_sub(double* C, const double* A, const double* B, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    C[i] = A[i] - B[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_subSelf(double* A, const double* B, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    A[i] -= B[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_add(double* C, const double* A, const double* B, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    C[i] = A[i] + B[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_addSelf(double* A, const double* B, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    A[i] += B[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double* VecNd_clone(const double* A, unsigned int n)
{
  double* self = RNALLOC(n, double);
  if (self == NULL)
  {
    return NULL;
  }
  memcpy(self, A, n * sizeof(double));
  return self;
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_sqrDiff(const double* v1, const double* v2, unsigned int n)
{
  unsigned int i;
  double ret = 0.0;

  for (i = 0; i < n; i++)
  {
    double factor = v1[i] - v2[i];
    ret += factor * factor;
  }

  return ret;
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_covariance(const double* v1, const double* v2, unsigned int n)
{
  if (n == 0)
  {
    return 0.0;
  }

  double mean1 = VecNd_mean(v1, n);
  double mean2 = VecNd_mean(v2, n);

  double cov = 0.0;
  for (unsigned int i = 0; i < n; i++)
  {
    cov += (v1[i] - mean1) * (v2[i] - mean2);
  }

  cov /= n;
  return cov;
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_mean(const double* v, unsigned int n)
{
  double mean = 0.0;
  for (unsigned int i = 0; i < n; i++)
  {
    mean += v[i];
  }

  mean /= n;
  return mean;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool VecNd_isFinite(const double* v, unsigned int n)
{
  for (unsigned int i = 0; i < n; i++)
  {
    if (Math_isFinite(v[i])==0)
    {
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool VecNd_isNaN(const double* v, unsigned int n)
{
  for (unsigned int i = 0; i < n; i++)
  {
    if (Math_isNAN(v[i]))
    {
      return true;
    }
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool VecNd_isZero(const double* v, unsigned int n)
{
  for (unsigned int i = 0; i < n; i++)
  {
    if (v[i] != 0.0)
    {
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_eleMul(double* C, const double* A, const double* B, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    C[i] = A[i]*B[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_eleMulSelf(double* dst, const double* src, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    dst[i] *= src[i];
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_print(const double* vec, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    fprintf(stderr, "   %f\n", vec[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_printTranspose(const double* vec, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    fprintf(stderr, "%f  ", vec[i]);
  }
  fprintf(stderr, "\n");
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_printDiff(const double* vec1, const double* vec2, unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    fprintf(stderr, "   %f\n", vec1[i] - vec2[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_printTwoArraysDiff(const double* vec1, const double* vec2,
                              unsigned int n)
{
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    fprintf(stderr, "   %+f\t%+f\t%+f\n", vec1[i], vec2[i], vec1[i] - vec2[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_printComment(const char* comment, const double* vec, unsigned int n)
{
  unsigned int i;

  fprintf(stderr, "%s:\n", comment);

  for (i = 0; i < n; i++)
  {
    fprintf(stderr, "   %f\n", vec[i]);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
const char* VecNd_toStr(const double* vec, unsigned int n, char* str)
{
  char tmp[32];

  if (vec == NULL)
  {
    strcpy(str, "[ NULL ]");
    return str;
  }

  strcpy(str, "[ ");

  for (unsigned int i=0; i<n-1; i++)
  {
    sprintf(tmp, "%f   ", vec[i]);
    strcat(str, tmp);
  }

  sprintf(tmp, "%f ]", vec[n-1]);
  strcat(str, tmp);

  return str;
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_sqrLength(const double* self, unsigned int nEle)
{
  unsigned int i;
  double sqrLength = 0.0;

  for (i = 0; i < nEle; i++)
  {
    sqrLength += self[i] * self[i];
  }

  return sqrLength;
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_setRandom(double* x, double lower, double upper, unsigned int nEle)
{
  for (unsigned int i = 0; i < nEle; i++)
  {
    x[i] = Math_getRandomNumber(lower, upper);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_addRandom(double* x, double lower, double upper, unsigned int nEle)
{
  for (unsigned int i = 0; i < nEle; i++)
  {
    x[i] += Math_getRandomNumber(lower, upper);
  }
}

/*******************************************************************************
 * Clips x element-wise to limits.
 ******************************************************************************/
void VecNd_clipSelf(double* x, double min, double max, unsigned int nEle)
{
  unsigned int i;

  for (i = 0; i < nEle; i++)
  {

    if (x[i] < min)
    {
      x[i] = min;
    }
    else if (x[i] > max)
    {
      x[i] = max;
    }

  }
}

/*******************************************************************************
 * Scales the array such that the maximum absolut value of each element
 * doesn't exceed the corresponding value given in limit. The scaling
 * factor is returned. Limit is assumed to have only positive values.
 * This is not checked.
 ******************************************************************************/
double VecNd_scaleSelf(double* self, const double* limit, unsigned int n)
{
  double scale = 1.0, absEle, scale_i;
  unsigned int i;

  for (i=0; i<n; i++)
  {
    absEle = fabs(self[i]);

    if (absEle>limit[i])
    {
      scale_i = limit[i]/absEle;

      if (scale_i<scale)
      {
        scale = scale_i;
      }
    }
  }

  VecNd_constMulSelf(self, scale, n);

  return scale;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int DoubleComparator(const void* x, const void* y) //needed by qsort
{
  if (*(double*) x > *(double*) y)
  {
    return 1;
  }
  else if (*(double*) x < *(double*) y)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_median(double* x, unsigned int nEle)
{
  qsort(x, nEle, sizeof(double), DoubleComparator);

  if (nEle & 1)   // If the number of items is odd, return middle number
  {
    return x[nEle / 2];
  }
  else   // If it is even, return mean of middle numbers
  {
    double a = x[nEle / 2];
    double b = x[nEle / 2 - 1];
    return 0.5*(a + b);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_sort(double* x_out, const double* x_in, unsigned int nEle)
{
  VecNd_copy(x_out, x_in, nEle);
  qsort(x_out, nEle, sizeof(double), DoubleComparator);
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_sortSelf(double* x, unsigned int nEle)
{
  qsort(x, nEle, sizeof(double), DoubleComparator);
}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_constSaturateSelf(double* self, const double limit,
                               unsigned int nEle)
{
  if (limit <= 0.0)
  {
    VecNd_setZero(self, nEle);
    return 0.0;
  }

  double scale = 1.0;
  double len = VecNd_getLength(self, nEle);

  if (len > limit)
  {
    scale = limit / len;
    VecNd_constMulSelf(self, scale, nEle);
  }

  return scale;
}

/*******************************************************************************
 *
 ******************************************************************************/
void VecNd_reverseSelf(double* self, unsigned int nEle)
{
  for (unsigned int i = 0; i < nEle / 2; i++)
  {
    double tmp = self[i];
    self[i] = self[nEle-i-1];
    self[nEle-i-1] = tmp;
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
double VecNd_innerProduct(const double* v1, const double* v2, unsigned int n)
{
  double res = 0.0;

  for (unsigned int i=0; i<n; ++i)
  {
    res += v1[i]*v2[i];
  }

  return res;
}
