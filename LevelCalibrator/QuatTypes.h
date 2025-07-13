/**** QuatTypes.h - Basic type declarations ****/
#ifndef _H_QuatTypes
#define _H_QuatTypes

/*** Definitions ***/
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { double x, y, z, w; } Quat; /* Quaternion */
enum QuatPart { X, Y, Z, W };
typedef double HMatrix[4][4]; /* Right-handed, for column vectors */
typedef Quat EulerAngles;    /* (x,y,z)=ang 1,2,3, w=order code  */
#ifdef __cplusplus
}
#endif
#endif
/**** EOF ****/
