#include "Rcs_Mat3d.h"
#include "Rcs_MatNd.h"
#include "Rcs_Vec3d.h"
#include "Rcs_basicMath.h"
#include "Rcs_macros.h"

#include <float.h>



/*******************************************************************************

  Graphics Gems III: Arvo, James, Fast Random Rotation Matrices,
                     p. 117-120, code: p. 463-464

  from http://tog.acm.org/resources/GraphicsGems/gemsiii/rand_rotation.c

  EULA: The Graphics Gems code is copyright-protected. In other words, you
  cannot claim the text of the code as your own and resell it. Using the
  code is permitted in any program, product, or library, non-commercial or
  commercial. Giving credit is not required, though is a nice gesture. The
  code comes as-is, and if there are any flaws or problems with any Gems
  code, nobody involved with Gems - authors, editors, publishers, or
  webmasters - are to be held responsible. Basically, don't be a jerk,
  and remember that anything free comes with no guarantee.

  This routine maps three values (x[0], x[1], x[2]) in the range [0,1]
  into a 3x3 rotation matrix, M.  Uniformly distributed random variables
  x0, x1, and x2 create uniformly distributed random rotation matrices.
  To create small uniformly distributed "perturbations", supply
  samples in the following ranges

      x[0] in [ 0, d ]
      x[1] in [ 0, 1 ]
      x[2] in [ 0, d ]

  where 0 < d < 1 controls the size of the perturbation.  Any of the
  random variables may be stratified (or "jittered") for a slightly more
  even distribution.

*******************************************************************************/
void Mat3d_setRandomRotation(double A[3][3])
{
  double phi, theta, z, x[3];
  const double PITIMES2 = 2.0*M_PI;

  Vec3d_setRandom(x, 0.0, 1.0);
  theta = x[0] * PITIMES2; // Rotation about the pole (Z).
  phi   = x[1] * PITIMES2; // For direction of pole deflection.
  z     = x[2] * 2.0;      // For magnitude of pole deflection.

  // Compute a vector V used for distributing points over the sphere
  // via the reflection I - V Transpose(V).  This formulation of V
  // will guarantee that if x[1] and x[2] are uniformly distributed,
  // the reflected points will be uniform on the sphere.  Note that V
  // has length sqrt(2) to eliminate the 2 in the Householder matrix.
  double r  = sqrt(z);
  double Vx = sin(phi) * r;
  double Vy = cos(phi) * r;
  double Vz = sqrt(2.0 - z);

  // Compute the row vector S = Transpose(V) * R, where R is a simple
  // rotation by theta about the z-axis.  No need to compute Sz since
  // it's just Vz.
  double st = sin(theta);
  double ct = cos(theta);
  double Sx = Vx * ct - Vy * st;
  double Sy = Vx * st + Vy * ct;

  // Construct the rotation matrix  (V Transpose(V) - I) R, which
  // is equivalent to V S - R.
  A[0][0] = Vx * Sx - ct;
  A[0][1] = Vx * Sy - st;
  A[0][2] = Vx * Vz;

  A[1][0] = Vy * Sx + st;
  A[1][1] = Vy * Sy - ct;
  A[1][2] = Vy * Vz;

  A[2][0] = Vz * Sx;
  A[2][1] = Vz * Sy;
  A[2][2] = 1.0 - z;   // This equals Vz * Vz - 1.0
}

/******************************************************************************
 *  Adapted from: Geometric Tools LLC, Redmond WA 98052
 *  Copyright (c) 1998-2015
 *  Distributed under the Boost Software License, Version 1.0.
 *  http://www.boost.org/LICENSE_1_0.txt
 *  http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 ******************************************************************************/
double Mat3d_getAxisAngleSelf(double axis[3], double rot[3][3])
{
  double trace = rot[0][0] + rot[1][1] + rot[2][2];
  double cs = 0.5*(trace - 1.0);
  double angle = Math_acos(cs);  // in [0,PI]

  if (angle > 0.0)
  {
    if (angle < M_PI)
    {
      axis[0] = rot[1][2] - rot[2][1];
      axis[1] = rot[2][0] - rot[0][2];
      axis[2] = rot[0][1] - rot[1][0];
      Vec3d_normalizeSelf(axis);
    }
    else
    {
      // angle is PI
      double halfInverse;
      if (rot[0][0] >= rot[1][1])
      {
        // r00 >= r11
        if (rot[0][0] >= rot[2][2])
        {
          // r00 is maximum diagonal term
          axis[0] = 0.5*sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]);
          halfInverse = 0.5/axis[0];
          axis[1] = halfInverse*rot[1][0];
          axis[2] = halfInverse*rot[2][0];
        }
        else
        {
          // r22 is maximum diagonal term
          axis[2] = 0.5*sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]);
          halfInverse = 0.5/axis[2];
          axis[0] = halfInverse*rot[2][0];
          axis[1] = halfInverse*rot[2][1];
        }
      }
      else
      {
        // r11 > r00
        if (rot[1][1] >= rot[2][2])
        {
          // r11 is maximum diagonal term
          axis[1] = 0.5*sqrt(1.0 + rot[1][1] - rot[0][0] - rot[2][2]);
          halfInverse  = 0.5/axis[1];
          axis[0] = halfInverse*rot[1][0];
          axis[2] = halfInverse*rot[2][1];
        }
        else
        {
          // r22 is maximum diagonal term
          axis[2] = 0.5*sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]);
          halfInverse = 0.5/axis[2];
          axis[0] = halfInverse*rot[2][0];
          axis[1] = halfInverse*rot[2][1];
        }
      }
    }
  }
  else
  {
    // The angle is 0 and the matrix is the identity.  Any axis will
    // work, so just use the x-axis.
    axis[0] = 1.0;
    axis[1] = 0.0;
    axis[2] = 0.0;
  }

  return angle;
}

/******************************************************************************
 *  Adapted from: Geometric Tools LLC, Redmond WA 98052
 *  Copyright (c) 1998-2015
 *  Distributed under the Boost Software License, Version 1.0.
 *  http://www.boost.org/LICENSE_1_0.txt
 *  http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 ******************************************************************************/
bool Mat3d_orthonormalizeSelf(double A[3][3])
{
  int i;
  double ex[3], ey[3], ez[3], fDot0, fDot1, len;

  Vec3d_copy(ex, A[0]);
  Vec3d_copy(ey, A[1]);
  Vec3d_copy(ez, A[2]);

  // compute u0
  len = Vec3d_normalizeSelf(ex);

  if (len==0.0)
  {
    return false;
  }

  // compute u1
  fDot0 = Vec3d_innerProduct(ex, ey);

  for (i = 0; i < 3; i++)
  {
    ey[i] -= fDot0 * ex[i];
  }

  len = Vec3d_normalizeSelf(ey);

  if (len==0.0)
  {
    return false;
  }

  // compute u2
  fDot1 = Vec3d_innerProduct(ey, ez);
  fDot0 = Vec3d_innerProduct(ex, ez);

  for (i = 0; i < 3; i++)
  {
    ez[i] -= fDot0 * ex[i] + fDot1 * ey[i];
  }

  len = Vec3d_normalizeSelf(ez);

  if (len==0.0)
  {
    return false;
  }

  Vec3d_copy(A[0], ex);
  Vec3d_copy(A[1], ey);
  Vec3d_copy(A[2], ez);

  return true;
}

/******************************************************************************
 *  Adapted from: Geometric Tools LLC, Redmond WA 98052
 *  Copyright (c) 1998-2015
 *  Distributed under the Boost Software License, Version 1.0.
 *  http://www.boost.org/LICENSE_1_0.txt
 *  http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 ******************************************************************************/
bool MatNd_parabolicFit2D(double* A, double* B, double* C, const MatNd* data)
{
  unsigned int i;
  double x = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0, y = 0.0, xy = 0.0, x2y = 0.0;
  double xi, xi2, yi, det, rMat[3][3], invMat[3][3];

  if (data->m < 3)
  {
    RLOG(4, "Can't perform parabolic fit for less than 3 points! "
         "You gave me only %d",
         data->m);
    return false;
  }

  RCHECK(data->n == 2);

  for (i = 0; i < data->m; i++) // number of samples
  {
    xi = MatNd_get(data, i, 0);
    yi = MatNd_get(data, i, 1);

    xi2 = xi * xi;
    x += xi;
    x2 += xi2;
    x3 += xi2 * xi;
    x4 += xi2 * xi2;
    y += yi;
    xy += xi * yi;
    x2y += xi2 * yi;
  }

  rMat[0][0] = x4;
  rMat[0][1] = x3;
  rMat[0][2] = x2;
  rMat[1][0] = x3;
  rMat[1][1] = x2;
  rMat[1][2] = x;
  rMat[2][0] = x2;
  rMat[2][1] = x;
  rMat[2][2] = data->m;

  det = Mat3d_inverse(invMat, rMat);

  if (det == 0.0)
  {
    RLOG(4, "Parabolic fit failed!");
    return false;
  }

  *A = invMat[0][0] * x2y + invMat[0][1] * xy + invMat[0][2] * y;
  *B = invMat[1][0] * x2y + invMat[1][1] * xy + invMat[1][2] * y;
  *C = invMat[2][0] * x2y + invMat[2][1] * xy + invMat[2][2] * y;

  return true;
}

/******************************************************************************
 *  Adapted from: Geometric Tools LLC, Redmond WA 98052
 *  Copyright (c) 1998-2015
 *  Distributed under the Boost Software License, Version 1.0.
 *  http://www.boost.org/LICENSE_1_0.txt
 *  http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 ******************************************************************************/
double MatNd_gaussInverse(MatNd* invA, const MatNd* A)
{
  RCHECK_EQ(A->m, A->n);

  if (A->m==0)
  {
    return 1.0;
  }

  // Don't copy if in-place inversion
  if (invA != A)
  {
    MatNd_copy(invA, A);
  }

  bool odd = false;
  double determinant = 1.0;
  int dim = A->m;
  int* colIndex = NULL;
  int* rowIndex = NULL;
  int* pivoted = NULL;

  if (dim > MATND_MAX_STACK_VECTOR_SIZE)
  {
    colIndex = RNALLOC(dim, int);
    rowIndex = RNALLOC(dim, int);
    pivoted  = RNALLOC(dim, int);
  }
  else
  {
    colIndex = RNSTALLOC(dim, int);
    rowIndex = RNSTALLOC(dim, int);
    pivoted  = RNSTALLOC(dim, int);
  }
  memset(pivoted, 0, dim*sizeof(int));


  // Elimination by full pivoting.
  int i1, i2, row = 0, col = 0;

  for (int i0 = 0; i0 < dim; ++i0)
  {
    // Search matrix (excluding pivoted rows) for maximum absolute entry.
    double maxValue = 0.0;

    for (i1 = 0; i1 < dim; ++i1)
    {
      if (!pivoted[i1])
      {
        for (i2 = 0; i2 < dim; ++i2)
        {
          if (!pivoted[i2])
          {
            double absValue = fabs(MatNd_get2(invA, i1, i2));

            if (absValue > maxValue)
            {
              maxValue = absValue;
              row = i1;
              col = i2;
            }
          }
        }
      }
    }

    // Matrix not invertible
    if (maxValue == 0.0)
    {
      MatNd_setZero(invA);

      if (dim > MATND_MAX_STACK_VECTOR_SIZE)
      {
        RFREE(colIndex);
        RFREE(rowIndex);
        RFREE(pivoted);
      }

      return 0.0;
    }

    pivoted[col] = true;

    // Swap rows so that the pivot entry is in row 'col'.
    if (row != col)
    {
      odd = !odd;
      for (int i = 0; i < dim; ++i)
      {
        MatNd_swapElements(invA, row, i, col, i);
      }
    }

    // Keep track of the permutations of the rows.
    rowIndex[i0] = row;
    colIndex[i0] = col;

    // Scale the row so that the pivot entry is 1.
    double diagonal = MatNd_get2(invA, col, col);
    determinant *= diagonal;
    double inv = 1.0/diagonal;
    MatNd_set2(invA, col, col, 1.0);

    for (i2 = 0; i2 < dim; ++i2)
    {
      *MatNd_getElePtr(invA, col, i2) *= inv;
    }

    // Zero out the pivot column locations in the other rows.
    for (i1 = 0; i1 < dim; ++i1)
    {
      if (i1 != col)
      {
        double save = MatNd_get2(invA, i1, col);
        MatNd_set2(invA, i1, col, 0.0);

        for (i2 = 0; i2 < dim; ++i2)
        {
          *MatNd_getElePtr(invA, i1, i2) -= MatNd_get2(invA, col, i2)*save;
        }
      }
    }
  }

  // Reorder rows to undo any permutations in Gaussian elimination.
  for (i1 = dim - 1; i1 >= 0; --i1)
  {
    if (rowIndex[i1] != colIndex[i1])
    {
      for (i2 = 0; i2 < dim; ++i2)
      {
        MatNd_swapElements(invA, i2, rowIndex[i1], i2, colIndex[i1]);
      }
    }
  }

  if (odd)
  {
    determinant = -determinant;
  }

  if (dim > MATND_MAX_STACK_VECTOR_SIZE)
  {
    RFREE(colIndex);
    RFREE(rowIndex);
    RFREE(pivoted);
  }

  return determinant;
}
