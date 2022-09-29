/******************************************************************************

  This code comes from
  Graphics Gems IV, Paul Heckbert (editor), Academic Press, 1994,
  ISBN: 0123361559 (Mac: 0123361567).

  License (see e.g. http://www.realtimerendering.com/resources/GraphicsGems):

  EULA: The Graphics Gems code is copyright-protected. In other words, you
        cannot claim the text of the code as your own and resell it. Using the
        code is permitted in any program, product, or library, non-commercial
        or commercial. Giving credit is not required, though is a nice gesture.
        The code comes as-is, and if there are any flaws or problems with any
        Gems code, nobody involved with Gems - authors, editors, publishers,
        or webmasters - are to be held responsible. Basically, don't be a jerk,
        and remember that anything free comes with no guarantee.

******************************************************************************/

#include "EulerAngles.h"

#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>

EulerAngles Eul_(double ai, double aj, double ah, int order)
{
  EulerAngles ea;
  ea.x = ai;
  ea.y = aj;
  ea.z = ah;
  ea.order = order;
  return (ea);
}

/* Construct quaternion from Euler angles (in radians). */
ShoemakeQuat Eul_ToQuat(EulerAngles ea)
{
  ShoemakeQuat qu;
  double a[3], ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
  int i,j,k,h,n,s,f;
  (void) h; // to avoid "assigned but unused" warning
  EulGetOrd(ea.order,i,j,k,h,n,s,f);
  if (f==EulFrmR)
  {
    double t = ea.x;
    ea.x = ea.z;
    ea.z = t;
  }
  if (n==EulParOdd)
  {
    ea.y = -ea.y;
  }
  ti = ea.x*0.5;
  tj = ea.y*0.5;
  th = ea.z*0.5;
  ci = cos(ti);
  cj = cos(tj);
  ch = cos(th);
  si = sin(ti);
  sj = sin(tj);
  sh = sin(th);
  cc = ci*ch;
  cs = ci*sh;
  sc = si*ch;
  ss = si*sh;
  if (s==EulRepYes)
  {
    a[i] = cj*(cs + sc);  /* Could speed up with */
    a[j] = sj*(cc + ss);  /* trig identities. */
    a[k] = sj*(cs - sc);
    qu.w = cj*(cc - ss);
  }
  else
  {
    a[i] = cj*sc - sj*cs;
    a[j] = cj*ss + sj*cc;
    a[k] = cj*cs - sj*sc;
    qu.w = cj*cc + sj*ss;
  }
  if (n==EulParOdd)
  {
    a[j] = -a[j];
  }
  qu.x = a[ShoemakeIdx_X];
  qu.y = a[ShoemakeIdx_Y];
  qu.z = a[ShoemakeIdx_Z];
  return (qu);
}

/* Construct matrix from Euler angles (in radians). */
void Eul_ToHMatrix(EulerAngles ea, HMatrix M)
{
  double ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
  int i,j,k,h,n,s,f;
  (void) h; // to avoid "assigned but unused" warning
  EulGetOrd(ea.order,i,j,k,h,n,s,f);
  if (f==EulFrmR)
  {
    double t = ea.x;
    ea.x = ea.z;
    ea.z = t;
  }
  if (n==EulParOdd)
  {
    ea.x = -ea.x;
    ea.y = -ea.y;
    ea.z = -ea.z;
  }
  ti = ea.x;
  tj = ea.y;
  th = ea.z;
  ci = cos(ti);
  cj = cos(tj);
  ch = cos(th);
  si = sin(ti);
  sj = sin(tj);
  sh = sin(th);
  cc = ci*ch;
  cs = ci*sh;
  sc = si*ch;
  ss = si*sh;
  if (s==EulRepYes)
  {
    M[i][i] = cj;
    M[i][j] =  sj*si;
    M[i][k] =  sj*ci;
    M[j][i] = sj*sh;
    M[j][j] = -cj*ss+cc;
    M[j][k] = -cj*cs-sc;
    M[k][i] = -sj*ch;
    M[k][j] =  cj*sc+cs;
    M[k][k] =  cj*cc-ss;
  }
  else
  {
    M[i][i] = cj*ch;
    M[i][j] = sj*sc-cs;
    M[i][k] = sj*cc+ss;
    M[j][i] = cj*sh;
    M[j][j] = sj*ss+cc;
    M[j][k] = sj*cs-sc;
    M[k][i] = -sj;
    M[k][j] = cj*si;
    M[k][k] = cj*ci;
  }
  M[ShoemakeIdx_W][ShoemakeIdx_X] = 0.0;
  M[ShoemakeIdx_W][ShoemakeIdx_Y] = 0.0;
  M[ShoemakeIdx_W][ShoemakeIdx_Z] = 0.0;
  M[ShoemakeIdx_X][ShoemakeIdx_W] = 0.0;
  M[ShoemakeIdx_Y][ShoemakeIdx_W] = 0.0;
  M[ShoemakeIdx_Z][ShoemakeIdx_W] = 0.0;
  M[ShoemakeIdx_W][ShoemakeIdx_W] = 1.0;
}

/*******************************************************************************
 * Convert matrix to Euler angles (in radians).
 ******************************************************************************/
EulerAngles Eul_FromHMatrix(HMatrix M, int order)
{
  EulerAngles ea;
  int i,j,k,h,n,s,f;
  (void) h; // to avoid "assigned but unused" warning
  EulGetOrd(order,i,j,k,h,n,s,f);
  if (s==EulRepYes)
  {
    double sy = sqrt(M[i][j]*M[i][j] + M[i][k]*M[i][k]);
    if (sy > 16.0*FLT_EPSILON)
    {
      ea.x = atan2(M[i][j], M[i][k]);
      ea.y = atan2(sy, M[i][i]);
      ea.z = atan2(M[j][i], -M[k][i]);
    }
    else
    {
      ea.x = atan2(-M[j][k], M[j][j]);
      ea.y = atan2(sy, M[i][i]);
      ea.z = 0.0;
    }
  }
  else
  {
    double cy = sqrt(M[i][i]*M[i][i] + M[j][i]*M[j][i]);
    if (cy > 16.0*FLT_EPSILON)
    {
      ea.x = atan2(M[k][j], M[k][k]);
      ea.y = atan2(-M[k][i], cy);
      ea.z = atan2(M[j][i], M[i][i]);
    }
    else
    {
      ea.x = atan2(-M[j][k], M[j][j]);
      ea.y = atan2(-M[k][i], cy);
      ea.z = 0.0;
    }
  }
  if (n==EulParOdd)
  {
    ea.x = -ea.x;
    ea.y = - ea.y;
    ea.z = -ea.z;
  }
  if (f==EulFrmR)
  {
    double t = ea.x;
    ea.x = ea.z;
    ea.z = t;
  }
  ea.order = order;
  return (ea);
}

/* Convert quaternion to Euler angles (in radians). */
EulerAngles Eul_FromQuat(ShoemakeQuat q, int order)
{
  HMatrix M;
  double Nq = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
  double xs = q.x*s,    ys = q.y*s,  zs = q.z*s;
  double wx = q.w*xs,   wy = q.w*ys,   wz = q.w*zs;
  double xx = q.x*xs,   xy = q.x*ys,   xz = q.x*zs;
  double yy = q.y*ys,   yz = q.y*zs,   zz = q.z*zs;
  M[ShoemakeIdx_X][ShoemakeIdx_X] = 1.0 - (yy + zz);
  M[ShoemakeIdx_X][ShoemakeIdx_Y] = xy - wz;
  M[ShoemakeIdx_X][ShoemakeIdx_Z] = xz + wy;
  M[ShoemakeIdx_Y][ShoemakeIdx_X] = xy + wz;
  M[ShoemakeIdx_Y][ShoemakeIdx_Y] = 1.0 - (xx + zz);
  M[ShoemakeIdx_Y][ShoemakeIdx_Z] = yz - wx;
  M[ShoemakeIdx_Z][ShoemakeIdx_X] = xz - wy;
  M[ShoemakeIdx_Z][ShoemakeIdx_Y] = yz + wx;
  M[ShoemakeIdx_Z][ShoemakeIdx_Z] = 1.0 - (xx + yy);
  M[ShoemakeIdx_W][ShoemakeIdx_X] = 0.0;
  M[ShoemakeIdx_W][ShoemakeIdx_Y] = 0.0;
  M[ShoemakeIdx_W][ShoemakeIdx_Z] = 0.0;
  M[ShoemakeIdx_X][ShoemakeIdx_W] = 0.0;
  M[ShoemakeIdx_Y][ShoemakeIdx_W] = 0.0;
  M[ShoemakeIdx_Z][ShoemakeIdx_W] = 0.0;
  M[ShoemakeIdx_W][ShoemakeIdx_W] = 1.0;
  return (Eul_FromHMatrix(M, order));
}

EulerAngles Eul_FromEuler(EulerAngles ea, int order)
{
  HMatrix temp;
  Eul_ToHMatrix(ea, temp);
  return Eul_FromHMatrix(temp, order);
}

void convertEulerAngles(double out[3], int outOrder, double in[3], int inOrder)
{
  EulerAngles inAngs = {in[0], in[1], in[2], inOrder};
  EulerAngles outAngs = Eul_FromEuler(inAngs, outOrder);
  out[0] = outAngs.x;
  out[1] = outAngs.y;
  out[2] = outAngs.z;
}

void convertEulerAnglesSelf(double inout[3], int outOrder, int inOrder)
{
  double out[3];
  convertEulerAngles(out, outOrder, inout, inOrder);
  inout[0] = out[0];
  inout[1] = out[1];
  inout[2] = out[2];
}


/* EulerSample.c - Read angles as quantum mechanics, write as aerospace */
void eulerTest(void)
{
  EulerAngles outAngs, inAngs = {0,0,0,EulOrdXYXr};
  HMatrix R;
  printf("Phi Theta Psi (radians): ");
  int nrd = scanf("%lf %lf %lf",&inAngs.x,&inAngs.y,&inAngs.z);
  if (nrd!=3)
  {
    printf("Less than 3 items read from stdin!\n");
  }
  Eul_ToHMatrix(inAngs, R);
  outAngs = Eul_FromHMatrix(R, EulOrdXYZs);
  printf(" Roll   Pitch  Yaw    (radians)\n");
  printf("%6.3f %6.3f %6.3f\n", outAngs.x, outAngs.y, outAngs.z);
}






/*******************************************************************************
 * Convert Rcs RotMatrix to Shoemake \ref HMatrix
 * H is a 4x4 matrix, the last column and row are 1 on the diagonal and 0
 * elsewhere
 ******************************************************************************/
static void RotMatrix2HMatrix(HMatrix H, double R[3][3])
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
 * Computes the current value of the task variable: Euler angles
 ******************************************************************************/
void computeEulerAngles(double ea[3],
                        double A_BI[3][3],
                        int eulerOrder)
{
  // use Shoemake code to convert RotMatrix to EulerAngles
  HMatrix H;
  RotMatrix2HMatrix(H, A_BI);
  EulerAngles outAngs;
  outAngs = Eul_FromHMatrix(H, eulerOrder);

  // convert Shoemake EulerAngles to states
  ea[0] = outAngs.x;
  ea[1] = outAngs.y;
  ea[2] = outAngs.z;
}

/*******************************************************************************
 * Prints all Euler angle combinations of a given rotation matrix to the
 * console.
 ******************************************************************************/
void printAllEulerAngles(double rm[3][3])
{
  double tmp[3];

  computeEulerAngles(tmp, rm, EulOrdXYZs);
  printf("EulOrdXYZs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdXYXs);
  printf("EulOrdXYXs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdXZYs);
  printf("EulOrdXZYs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdXZXs);
  printf("EulOrdXZXs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdYZXs);
  printf("EulOrdYZXs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdYZYs);
  printf("EulOrdYZYs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdYXZs);
  printf("EulOrdYXZs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdYXYs);
  printf("EulOrdYXYs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdZXYs);
  printf("EulOrdZXYs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdZXZs);
  printf("EulOrdZXZs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdZYXs);
  printf("EulOrdZYXs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdZYZs);
  printf("EulOrdZYZs: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);

  computeEulerAngles(tmp, rm, EulOrdZYXr);
  printf("EulOrdZYXr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdXYXr);
  printf("EulOrdXYXr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdYZXr);
  printf("EulOrdYZXr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdXZXr);
  printf("EulOrdXZXr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdXZYr);
  printf("EulOrdXZYr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdYZYr);
  printf("EulOrdYZYr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdZXYr);
  printf("EulOrdZXYr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdYXYr);
  printf("EulOrdYXYr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdYXZr);
  printf("EulOrdYXZr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdZXZr);
  printf("EulOrdZXZr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdXYZr);
  printf("EulOrdXYZr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
  computeEulerAngles(tmp, rm, EulOrdZYZr);
  printf("EulOrdZYZr: %f   %f   %f\n", tmp[0], tmp[1], tmp[2]);
}
