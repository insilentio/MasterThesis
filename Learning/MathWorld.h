/* ---------------------------------------------------
   FILE:     MathWorld.h
	AUTHOR:   Josh Bongard
	DATE:     October 3, 2000
	FUNCTION: This is a container class for as much
				 of the mathEngine housekeeping material
				 as possible.
 -------------------------------------------------- */

#include "Stdafx.h"

#ifndef _MathWorld_H
#define _MathWorld_H

#include "McdFrame.h"
#include "Mdt.h"
#include "MeViewer.h"


class MathWorld {

public:
	MdtBodyID				target;

private:
	MdtBodyID				ground;
	RGraphicsDescription *groundG;
	McdModelID				groundCM;
	McdGeometryID			ground_prim;

	RGraphicsDescription *targetG;
	McdModelID				targetCM;
	McdGeometryID			target_prim;

public:
	MathWorld(void);
	~MathWorld(void);

private:
	void CreateTarget(void);
	void DeleteTarget(void);
};

#endif