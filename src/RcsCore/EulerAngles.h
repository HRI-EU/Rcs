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

#ifndef RCS_EULERANGLES_H
#define RCS_EULERANGLES_H

typedef struct
{
  double x, y, z, w;
} ShoemakeQuat;

enum ShoemakeIdx
{
  ShoemakeIdx_X, ShoemakeIdx_Y, ShoemakeIdx_Z, ShoemakeIdx_W
};

typedef double HMatrix[4][4]; /* Right-handed, for column vectors */

typedef struct
{
  double x, y, z;
  int order;
} EulerAngles;



/*
  Order type constants, constructors, extractors
  There are 24 possible conventions, designated by:
     o EulAxI = axis used initially
     o EulPar = parity of axis permutation
     o EulRep = repetition of initial axis as last
     o EulFrm = frame from which axes are taken
  Axes I,J,K will be a permutation of X,Y,Z.
  Axis H will be either I or K, depending on EulRep.
  Frame S takes axes from initial static frame.
  If ord = (AxI=X, Par=Even, Rep=No, Frm=S), then
  {a,b,c,ord} means Rz(c)Ry(b)Rx(a), where Rz(c)v
  rotates v around Z by c radians.
*/

#define EulFrmS      0
#define EulFrmR      1
#define EulFrm(ord)  ((unsigned)(ord)&1)
#define EulRepNo     0
#define EulRepYes    1
#define EulRep(ord)  (((unsigned)(ord)>>1)&1)
#define EulParEven   0
#define EulParOdd    1
#define EulPar(ord)  (((unsigned)(ord)>>2)&1)
#define EulSafe      "\000\001\002\000"
#define EulNext      "\001\002\000\001"
#define EulAxI(ord)  ((int)(EulSafe[(((unsigned)(ord)>>3)&3)]))
#define EulAxJ(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)==EulParOdd)]))
#define EulAxK(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)!=EulParOdd)]))
#define EulAxH(ord)  ((EulRep(ord)==EulRepNo)?EulAxK(ord):EulAxI(ord))
/* EulGetOrd unpacks all useful information about order simultaneously. */
#define EulGetOrd(ord,i,j,k,h,n,s,f) {unsigned o=ord;f=o&1;o>>=1;s=o&1;o>>=1;\
    n=o&1;o>>=1;i=EulSafe[o&3];j=EulNext[i+n];k=EulNext[i+1-n];h=s?k:i;}
/* EulOrd creates an order value between 0 and 23 from 4-tuple choices. */
#define EulOrd(i,p,r,f)    (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
/* Static axes */
#define EulOrdXYZs    EulOrd(ShoemakeIdx_X,EulParEven,EulRepNo,EulFrmS)
#define EulOrdXYXs    EulOrd(ShoemakeIdx_X,EulParEven,EulRepYes,EulFrmS)
#define EulOrdXZYs    EulOrd(ShoemakeIdx_X,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdXZXs    EulOrd(ShoemakeIdx_X,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdYZXs    EulOrd(ShoemakeIdx_Y,EulParEven,EulRepNo,EulFrmS)
#define EulOrdYZYs    EulOrd(ShoemakeIdx_Y,EulParEven,EulRepYes,EulFrmS)
#define EulOrdYXZs    EulOrd(ShoemakeIdx_Y,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdYXYs    EulOrd(ShoemakeIdx_Y,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdZXYs    EulOrd(ShoemakeIdx_Z,EulParEven,EulRepNo,EulFrmS)
#define EulOrdZXZs    EulOrd(ShoemakeIdx_Z,EulParEven,EulRepYes,EulFrmS)
#define EulOrdZYXs    EulOrd(ShoemakeIdx_Z,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdZYZs    EulOrd(ShoemakeIdx_Z,EulParOdd,EulRepYes,EulFrmS)
/* Rotating axes */
#define EulOrdZYXr    EulOrd(ShoemakeIdx_X,EulParEven,EulRepNo,EulFrmR)
#define EulOrdXYXr    EulOrd(ShoemakeIdx_X,EulParEven,EulRepYes,EulFrmR)
#define EulOrdYZXr    EulOrd(ShoemakeIdx_X,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdXZXr    EulOrd(ShoemakeIdx_X,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdXZYr    EulOrd(ShoemakeIdx_Y,EulParEven,EulRepNo,EulFrmR)
#define EulOrdYZYr    EulOrd(ShoemakeIdx_Y,EulParEven,EulRepYes,EulFrmR)
#define EulOrdZXYr    EulOrd(ShoemakeIdx_Y,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdYXYr    EulOrd(ShoemakeIdx_Y,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdYXZr    EulOrd(ShoemakeIdx_Z,EulParEven,EulRepNo,EulFrmR)
#define EulOrdZXZr    EulOrd(ShoemakeIdx_Z,EulParEven,EulRepYes,EulFrmR)
#define EulOrdXYZr    EulOrd(ShoemakeIdx_Z,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdZYZr    EulOrd(ShoemakeIdx_Z,EulParOdd,EulRepYes,EulFrmR)

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief "Constructor" based on Euler angle components and order.
 */
EulerAngles Eul_(double ai, double aj, double ah, int order);

/*! \brief Conversion from Euler angles to quaternion.
 */
ShoemakeQuat Eul_ToQuat(EulerAngles ea);

/*! \brief Conversion from Euler angles to homogenuous 4 x 4 matrix.
 */
void Eul_ToHMatrix(EulerAngles ea, HMatrix M);

/*! \brief Conversion from homogenuous 4 x 4 matrix to Euler angles of a given
 *         order.
 */
EulerAngles Eul_FromHMatrix(HMatrix M, int order);

/*! \brief Conversion from quaternion to Euler angles of a given order.
 */
EulerAngles Eul_FromQuat(ShoemakeQuat q, int order);

/*! \brief Convert between Euler types
 *
 *  \param[in] ea Input Euler angles
 *  \param[in] order Type of the returned Euler angles
 *  \return Euler angles of type order
 */
EulerAngles Eul_FromEuler(EulerAngles ea, int order);

/*! \brief Convert between Euler types with one function call
 *
 *  \param[out] out Output Euler angles
 *  \param[in] outOrder Output Euler type
 *  \param[in] in Input Euler angles
 *  \param[in] inOrder Input Euler type
 */
void convertEulerAngles(double out[3], int outOrder, double in[3], int inOrder);

/*! \brief Convert between Euler types with one function call
 *
 *  \param[in,out] inout Input and output Euler angles
 *  \param[in] outOrder Output Euler type
 *  \param[in] inOrder Input Euler type
 */
void convertEulerAnglesSelf(double inout[3], int outOrder, int inOrder);

/*! \brief Computes the Euler angles of the given order from the given
 *         rotation matrix.
 */
void computeEulerAngles(double ea[3], double A_BI[3][3], int eulerOrder);

/*! \brief Prints the Euler angles of the rotation matrix in all orders
 *         to the console.
 */
void printAllEulerAngles(double rm[3][3]);

#ifdef __cplusplus
}
#endif

#endif   // RCS_EULERANGLES_H
