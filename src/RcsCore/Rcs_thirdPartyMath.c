#include "Rcs_math.h"
#include "Rcs_macros.h"
#include "Rcs_mesh.h"

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

/*******************************************************************************
 *
 * Code extract from:
 * "Polyhedral Mass Properties (Revisited)" by David Eberly
 *  http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
 *
 * The pseudo-code for computing the integrals is quite simple. The polyhedron
 * vertices are passes as the array p[]. The number of triangles is tmax. The
 * array index[] has tmax triples of integers that are indices into the vertex
 * array. The return values are the mass, the center of mass, and the inertia
 * tensor relative to the center of mass. The code assumes that the rigid body
 * has constant density 1. If your rigid body has constant density D, then you
 * need to multiply the output mass by D and the output inertia tensor by D.
 ******************************************************************************/
static void Subespressions(double* w0, double* w1, double* w2,
                           double* f1, double* f2, double* f3,
                           double* g0, double* g1, double* g2)
{
  double temp0 = (*w0)+(*w1);
  *f1 = temp0+(*w2);
  double temp1 = (*w0)*(*w0);
  double temp2 = temp1+(*w1)*temp0;
  *f2 = temp2+(*w2)*(*f1);
  *f3 = (*w0)*temp1+(*w1)*temp2+(*w2)*(*f2);
  *g0 = (*f2)+(*w0)*((*f1)+(*w0));
  *g1 = (*f2)+(*w1)*((*f1)+(*w1));
  *g2 = (*f2)+(*w2)*((*f1)+(*w2));
}

static double Compute(const double* p, int tmax, const unsigned int index[],
                      double com[3], double inertia[3][3])
{
  const double mult[10]  = { 1.0/6.0, 1.0/24.0, 1.0/24.0, 1.0/24.0, 1.0/60.0, 1.0/60.0, 1.0/60.0, 1.0/120.0, 1.0/120.0, 1.0/120.0 };

  // order: 1, x, y, z, x^2, y^2 , z^2, xy, yz, zx
  double intg[10];
  for (int i=0; i<10; ++i)
  {
    intg[i] = 0.0;
  }

  for (int t=0 ; t < tmax ; t++)
  {
    // get vertices of triangle t
    int i0 = index[3*t];
    int i1 = index[3*t+1];
    int i2 = index[3*t+2];

    double x0 = p[i0*3+0];
    double y0 = p[i0*3+1];
    double z0 = p[i0*3+2];

    double x1 = p[i1*3+0];
    double y1 = p[i1*3+1];
    double z1 = p[i1*3+2];

    double x2 = p[i2*3+0];
    double y2 = p[i2*3+1];
    double z2 = p[i2*3+2];

    // get edges and cross product of edges
    double a1 = x1-x0;
    double b1 = y1-y0;
    double c1 = z1-z0;
    double a2 = x2-x0;
    double b2 = y2-y0;
    double c2 = z2-z0;
    double d0 = b1*c2-b2*c1;
    double d1 = a2*c1-a1*c2;
    double d2 = a1*b2-a2*b1;

    double f1x, f1y, f1z, f2x, f2y, f2z, f3x, f3y, f3z;
    double g0x, g0y, g0z, g1x, g1y, g1z, g2x, g2y, g2z;

    // compute integral terms
    Subespressions(&x0, &x1, &x2, &f1x, &f2x, &f3x, &g0x, &g1x, &g2x);
    Subespressions(&y0, &y1, &y2, &f1y, &f2y, &f3y, &g0y, &g1y, &g2y);
    Subespressions(&z0, &z1, &z2, &f1z, &f2z, &f3z, &g0z, &g1z, &g2z);

    // updte integrals
    intg[0] += d0* f1x;
    intg[1] += d0* f2x;
    intg[2] += d1* f2y;
    intg[4] += d0* f3x;
    intg[5] += d1* f3y;
    intg[7] += d0 *(y0 * g0x+y1 * g1x+y2 * g2x);
    intg[8] += d1 *(z0 * g0y+z1 * g1y+z2 * g2y);
    intg[9] += d2 *(x0 * g0z+x1 * g1z+x2 * g2z);
    intg[3] += d2* f2z;
    intg[6] += d2* f3z;
  }

  for (int i=0 ; i<10 ; i++)
  {
    intg[i] *= mult[i];
  }

  double mass = intg[0];

  // center of mass
  com[0] = intg[1] / mass;
  com[1] = intg[2] / mass;
  com[2] = intg[3] / mass;

  // inertia tensor relative to center of mass
  inertia[0][0] = intg[5] + intg[6] - mass*(com[1]*com[1] + com[2]*com[2]);
  inertia[1][1]= intg[4] + intg[6] - mass*(com[2] *com[2] + com[0]*com[0]);
  inertia[2][2] = intg[4] + intg[5] - mass*(com[0]*com[0] + com[1]*com[1]);
  inertia[0][1] = -(intg[7] - mass*com[0] * com[1]);
  inertia[1][2] = -(intg[8] - mass*com[1] * com[2]);
  inertia[0][2] = -(intg[9] - mass*com[2] * com[0]);
  inertia[1][0] = inertia[0][1];
  inertia[2][0] = inertia[0][2];
  inertia[2][1] = inertia[1][2];

  return mass;
}

/*! \brief Computes the mesh's center of mass and the inertia tensor around the
 *         COM. If argument mesh is NULL, both I and com are set to zero.
 *
 * Code extract from:
 * "Polyhedral Mass Properties (Revisited)" by David Eberly
 *  http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
 *
 *  \param[in] mesh Mesh data. If it is invalid, the behavior is undefined.
 *  \param[out] I   Inertia tensor around COM
 *  \param[out] com Center of mass
 */
void RcsMesh_computeInertia(RcsMeshData* mesh, double I[3][3], double com[3])
{
  if (mesh==NULL)
  {
    Vec3d_setZero(com);
    Mat3d_setZero(I);
    return;
  }

  Compute(mesh->vertices, mesh->nFaces, mesh->faces, com, I);
}
