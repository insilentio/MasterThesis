/* ---------------------------------------------------
   FILE:     MathWorld.cpp
	AUTHOR:   Josh Bongard
	DATE:     October 3, 2000
	FUNCTION: This is a container class for as much of
				 the MathEngine specific code as possible.
 -------------------------------------------------- */

#include "Stdafx.h"

#ifndef _MathWorld_CPP
#define _MathWorld_CPP

#include "MathWorld.h"
#include "SimParams.h"
#include "McdFrame.h"
#include "McdPrimitives.h"
#include "McdDtBridge.h"
#include "Mdt.h"
#include "MeViewer.h"

extern MeReal				gravity[3];
extern MeReal				floorDims[3];
extern float				blue[3];
extern float				red[3];
extern int					MAX_BODIES;
extern McdSpaceID			space;
extern McdDtBridgeID		cdHandler;
extern MeReal				groundTransform[4][4];
extern float				GROUND_MASS;
extern MdtWorldID			world;
extern SimParams			*simParams;
extern RRender				*rc;
extern void					*mem;
extern struct				MeMemoryOptions opts;
extern long int				MEMORY_TO_USE;
extern int					MAX_BODIES_ALLOC;
extern int					argNumber;
extern const char			**args;
extern float				TARGET_SIZE;
extern float				TARGET_DIST;
extern int                  floorMaterial;
extern int                  leftFootMaterial;
extern int                  rightFootMaterial;
extern int					waistMaterial;
extern int					FRICTION;
extern bool					firstWorld;

MathWorld::MathWorld(void) {

	if (firstWorld)
	{
		cout << "creating a math world" << endl;
		firstWorld = false;
	}
	else
		cout << "recreating the math world" << endl;

	if ( !simParams->rendererCreated ) {

		const RRenderType render = RParseRenderType(&argNumber, &args);

		rc = RNewRenderContext(render, kRQualityFlat);
		simParams->rendererCreated = true;
	}

	MeMemorySetDefaults(&opts);
	(*MeMemoryAPI.setOptions)(&opts);

	mem = malloc(MEMORY_TO_USE);
	world = MdtWorldCreate(MAX_BODIES_ALLOC,MAX_BODIES_ALLOC*MAX_BODIES_ALLOC,mem, MEMORY_TO_USE);

	MdtWorldSetGravity(world, gravity[0], gravity[1], gravity[2]);
	McdInit(McdPrimitivesGetTypeCount());
	McdPrimitivesRegisterTypes();
	McdPrimitivesRegisterInteractions();

	/* initialize the bridge between collision and dynamics */
	McdDtBridgeInit(100);
	space = McdSpaceAxisSortCreate(McdAllAxes, MAX_BODIES_ALLOC,(MAX_BODIES_ALLOC)*(MAX_BODIES_ALLOC));
	McdPairHandlerRegisterSpace(space);
	cdHandler = McdDtBridgeCreate();

    /* ground graphic */
	groundG = RCreateCube(rc, 2*floorDims[0], 2*floorDims[1], 2*floorDims[2], blue, 0);
	RSetTexture(groundG, "floor");
	groundG->m_matrixArray[13] = -floorDims[1];

	/* wall collision */
	ground_prim = McdPlaneCreate();
	groundCM = McdModelCreate(ground_prim);
	McdSpaceInsertModel(space, groundCM);

	McdDtBridgeSetBody(cdHandler, groundCM, 0);
    McdModelSetTransformPtr(groundCM, groundTransform);
	
	// Set parameters for contacts.
	MdtContactParamsID params;

	floorMaterial = McdDtBridgeGetDefaultMaterialID();
	leftFootMaterial = McdDtBridgeGetNewMaterialID();
    rightFootMaterial = McdDtBridgeGetNewMaterialID(); 	

    params = McdDtBridgeGetContactParams(floorMaterial, floorMaterial);
    MdtContactParamsSetType(params, MdtContactTypeFriction2D);
	MdtContactParamsSetFriction(params, 10000);

    params = McdDtBridgeGetContactParams(leftFootMaterial, floorMaterial);
    MdtContactParamsSetType(params, MdtContactTypeFriction2D);
	MdtContactParamsSetFriction(params, FRICTION);

	params = McdDtBridgeGetContactParams(rightFootMaterial, floorMaterial);
    MdtContactParamsSetType(params, MdtContactTypeFriction2D);
	MdtContactParamsSetFriction(params, FRICTION);

	McdSpaceBuild(space);
}

MathWorld::~MathWorld(void) {

	extern MdtWorldID world;
	extern RRender    *rc;
	extern void       *mem;

	McdModelRemoveFromSpace(groundCM);
	McdModelDestroy(groundCM);
	McdGeometryDestroy(ground_prim);

	RDeleteGraphic(groundG);

	McdSpaceDestroy(space);
	McdDtBridgeDestroy(cdHandler);
	McdDtBridgeTerm();
	McdTerm();

	MdtWorldDestroy(world);
	free(mem);
}

// ----------------------------------------------------------------
//                           Private methods
// ----------------------------------------------------------------

void MathWorld::CreateTarget(void) {

	target = MdtBodyCreate(world);

	MdtBodySetPosition(target, 0.0, TARGET_SIZE*26/2.0, -TARGET_DIST);
	MdtBodySetMass(target, TARGET_SIZE*TARGET_SIZE*TARGET_SIZE*1000);

    targetG = RCreateCube(rc, TARGET_SIZE*6, TARGET_SIZE*18 , TARGET_SIZE*50, red, MdtBodyGetTransformPtr(target));

    target_prim = McdBoxCreate(TARGET_SIZE*6,TARGET_SIZE*18,TARGET_SIZE*50);
    targetCM = McdModelCreate(target_prim);
    McdModelSetUserData(targetCM, NULL);

    McdDtBridgeSetBody(cdHandler, targetCM, target);
    McdSpaceInsertModel(space, targetCM);

	MdtBodyEnable(target);

}

void MathWorld::DeleteTarget(void) {

	MdtBodyDisable(target);
	MdtBodyDestroy(target);

	McdModelRemoveFromSpace(targetCM);
	McdModelDestroy(targetCM);
	McdGeometryDestroy(target_prim);

	RDeleteGraphic(targetG);
}

#endif