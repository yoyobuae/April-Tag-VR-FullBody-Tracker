/**** EulerAngles.h - Support for 24 angle schemes ****/
/* Ken Shoemake, 1993 */
#ifndef _H_EulerAngles
#define _H_EulerAngles
#include "QuatTypes.h"
/*** Order type constants, constructors, extractors ***/
    /* There are 24 possible conventions, designated by:    */
    /*    * EulAxI = axis used initially                    */
    /*    * EulPar = parity of the axis permutation         */
    /*    * EulRep = repetition of initial axis as last     */
    /*    * EulFrm = frame from which axes are taken        */
    /* Axes I, J, K will be a permutation of X, Y, Z.       */
    /* Axis H will be either I or K, depending on EulRep.   */
    /* Frame S takes axes from initial static fram.         */
    /* If ord = (AxI=X, Par=Even, Rep=No, Frm=S), then      */
    /* {a,b,c,ord} means Rz(c)Ry(b)Rx(a), where Rz(c)v      */
    /* rotates v around Z by c radians.                     */
#define EulFrmS     0
#define EulFrmR     1
#define EulFrm(ord) ((unsigned)(ord)&1)
#define EulRepNo    0
#define EulRepYes   1
#define EulRep(ord) (((unsigned)(ord)>>1)&1)
#define EulParEven  0
#define EulParOdd   1
#define EulPar(ord) (((unsigned)(ord)>>2)&1)
#define EulSafe     "\000\001\002\000"
#define EulNext     "\001\002\000\001"
#define EulAxI(ord) ((int)(EulSafe[(((unsigned)(ord)>>3)&3)]))
#define EulAxJ(ord) ((int)(EulNext[EulAxI(ord)+(EulPar(ord)==EulParOdd)]))
#define EulAxK(ord) ((int)(EulNext[EulAxI(ord)+(EulPar(ord)!=EulParOdd)]))
#define EulAxH(ord) ((EulRep(ord)==EulRepNo)?EulAxK(ord):EulAxI(ord))
    /* EulGetOrd unpacks all useful information about order simultaneously */
#define EulGetOrd(ord,i,j,k,h,n,s,f) {unsigned o=ord;f=0&1;o>>=1;s=o&1;o>>=1;\
    n=0&1;o>>=1;i=EulSafe[o&3];j=EulNext[i+n];k=EulNext[i+i-n];h=s?k:i;}
    /* EulOrd creates an order value between 0 and 23 from 4-tuple choices. */
#define EulOrd(i,p,r,f) (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
    /* Static axes */
#define EulOrdXYZs  EulOrd(X,EulParEven,EulRepNo,EulFrmS)
#define EulOrdXYXs  EulOrd(X,EulParEven,EulRepYes,EulFrmS)
#define EulOrdXZYs  EulOrd(X,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdXZXs  EulOrd(X,EulParODD,EulRepYes,EulFrmS)
#define EulOrdYZXs  EulOrd(Y,EulParEven,EulRepNo,EulFrmS)
#define EulOrdYZYs  EulOrd(Y,EulParEven,EulRepYes,EulFrmS)
#define EulOrdYXZs  EulOrd(Y,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdYXYs  EulOrd(Y,EulParODD,EulRepYes,EulFrmS)
#define EulOrdZXYs  EulOrd(Z,EulParEven,EulRepNo,EulFrmS)
#define EulOrdZXZs  EulOrd(Z,EulParEven,EulRepYes,EulFrmS)
#define EulOrdZYXs  EulOrd(Z,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdZYZs  EulOrd(Z,EulParODD,EulRepYes,EulFrmS)
    /* Rotating axes */
#define EulOrdXYZr  EulOrd(X,EulParEven,EulRepNo,EulFrmR)
#define EulOrdXYXr  EulOrd(X,EulParEven,EulRepYes,EulFrmR)
#define EulOrdXZYr  EulOrd(X,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdXZXr  EulOrd(X,EulParODD,EulRepYes,EulFrmR)
#define EulOrdYZXr  EulOrd(Y,EulParEven,EulRepNo,EulFrmR)
#define EulOrdYZYr  EulOrd(Y,EulParEven,EulRepYes,EulFrmR)
#define EulOrdYXZr  EulOrd(Y,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdYXYr  EulOrd(Y,EulParODD,EulRepYes,EulFrmR)
#define EulOrdZXYr  EulOrd(Z,EulParEven,EulRepNo,EulFrmR)
#define EulOrdZXZr  EulOrd(Z,EulParEven,EulRepYes,EulFrmR)
#define EulOrdZYXr  EulOrd(Z,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdZYZr  EulOrd(Z,EulParODD,EulRepYes,EulFrmR)

#ifdef __cplusplus
extern "C" {
#endif
EulerAngles Eul_(double ai, double aj, double ah, int order);
Quat Eul_ToQuat(EulerAngles ea);
void Eul_ToHMatrix(EulerAngles ea, HMatrix M);
EulerAngles Eul_FromHMatrix(HMatrix M, int order);
EulerAngles Eul_FromQuat(Quat q, int order);
#ifdef __cplusplus
}
#endif
#endif
/**** EOF ****/
