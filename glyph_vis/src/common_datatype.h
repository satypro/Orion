#ifndef DATATYPE_COMMON_H
#define DATATYPE_COMMON_H

#include <teem/ten.h>
#include <stdio.h>

using namespace std;
/*****************************************************************************************
******************************************************************************************

Global Variables

******************************************************************************************
*****************************************************************************************/
typedef float	xyzType;
typedef int	idxType;
typedef unsigned short inten;



struct metaDatat
	{
	xyzType XYZ[3];
	xyzType eval[3];
	xyzType evec[9];
	xyzType UV[2];
	xyzType ABC[3];
	idxType intensity;
	xyzType clcpcs[3];
	xyzType curvature;
	};


#endif