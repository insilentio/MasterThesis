/* ---------------------------------------------------
   FILE:     mathWorld.h
	AUTHOR:   Josh Bongard
	DATE:     October 3, 2000
	FUNCTION: This is a container class for as much
				 of the mathEngine housekeeping material
				 as possible.
 -------------------------------------------------- */

#include "stdafx.h"

#ifndef _MATH_WORLD_H
#define _MATH_WORLD_H

#include "McdFrame.h"
#include "Mdt.h"
#include "MeViewer.h"


class MATH_WORLD {

public:
//	MdtBodyID				target;

private:
	MdtBodyID				ground;
	RGraphicsDescription *groundG;
	McdModelID				groundCM;
	McdGeometryID			ground_prim;

//	RGraphicsDescription *targetG;
//	McdModelID				targetCM;
//	McdGeometryID			target_prim;

public:
	MATH_WORLD(void);
	~MATH_WORLD(void);

private:
//	void CreateTarget(void);
//	void DeleteTarget(void);
};

#endif