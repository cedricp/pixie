#include <math.h>

#include "stochastic.h"
#include "memory.h"
#include "random.h"

// This macro is used to allocate fragments
#define	newFragment(__a)	if (freeFragments == NULL)	{						\
								__a					=	new CFragment;			\
								if (CRenderer::numExtraSamples > 0) {							\
									__a->extraSamples = new float[CRenderer::numExtraSamples]; 	\
								}																\
							} else {											\
								__a					=	freeFragments;			\
								freeFragments		=	freeFragments->next;	\
							}													\
							numFragments++;

// And deallocate macro
#define	deleteFragment(__a)	__a->next				=	freeFragments;			\
							freeFragments			=	__a;					\
							numFragments--;

///////////////////////////////////////////////////////////////////////
// Class				:	CStochastic
// Method				:	rasterDrawPrimitives
// Description			:	Draw bunch of primitives
// Return Value			:	-
// Comments				:
void		CStochastic::rasterDrawPrimitives(CRasterGrid *grid) {

// Instantiate the dispatch switch
#define DEFINE_STOCHASTIC_SWITCH
	#include "stochasticPrimitives.h"
#undef DEFINE_STOCHASTIC_SWITCH
}



// The following macros help various fragment operations
#define depthFilterIfZMin()
#define depthFilterElseZMin()
#define depthFilterTouchNodeZMin()	touchNode(pixel->node,z);

#define depthFilterIfZMid()			pixel->zold		=	pixel->z;
#define depthFilterElseZMid()		else {	pixel->zold	=	minn(pixel->zold,z);	}
#define depthFilterTouchNodeZMid()	touchNode(pixel->node,pixel->zold);


// This macro is used to insert a fragment into the linked list for a pixel
#define	findSample(__dest,__z) { 																	\
	CFragment *lSample	=	pixel->update;															\
	if (__z >= lSample->z)	{																		\
		CFragment		*cSample;																	\
		for (cSample=lSample->next;__z >= cSample->z;lSample=cSample,cSample=cSample->next);		\
		assert(__z >= lSample->z);																	\
		assert(__z <= cSample->z);																	\
		newFragment(__dest);																		\
		__dest->next	=	cSample;																\
		__dest->prev	=	lSample;																\
		cSample->prev	=	__dest;																	\
		lSample->next	=	__dest;																	\
	} else {																						\
		CFragment		*cSample;																	\
		for (cSample=lSample->prev;__z < cSample->z;lSample=cSample,cSample=cSample->prev);			\
		assert(__z >= cSample->z);																	\
		assert(__z <= lSample->z);																	\
		newFragment(__dest);																		\
		__dest->next	=	lSample;																\
		__dest->prev	=	cSample;																\
		cSample->next	=	__dest;																	\
		lSample->prev	=	__dest;																	\
	}																								\
	pixel->update	=	__dest;																		\
}

// This macro is called when an opaque fragment is inserted
// Note: On the assumption that the opacity really is nearly opaque, we don't really need
// to bother messing with pixel->last though it might technicaly be more correct to do so
// so these sections are commented out in updateOpaque and updateTransparent


#define updateOpaque() {																			\
	CFragment *cSample=pixel->last.prev;															\
	while(cSample->z > z) {																			\
		CFragment *nSample	=	cSample->prev;														\
		nSample->next		=	&pixel->last;														\
		pixel->last.prev	=	nSample;															\
		assert(cSample != &pixel->first);															\
		deleteFragment(cSample);																	\
		cSample				=	nSample;															\
	}																								\
	/*initv(pixel->last.accumulatedOpacity,1);*/													\
	pixel->update			=	cSample;															\
}

// Note: due to the way we insert samples, we may have inserted a new one behind the
// maximum opaque depth - in which case we must flush the new sample and everything
// behind it.  Otherwise, we need to update accumulated opacity, and cull samples
// behind the point where we become opaque

#define debugTransparencyStack(cSample) {						\
	printf(">> cull opac %.6f %.6f %.6f\n",O[0],O[1],O[2]);		\
	CFragment *ds=cSample;										\
	while(ds) {													\
		printf("opac %.6f %.6f %.6f\tropac %.6f %.6f %.6f",ds->opacity[0],ds->opacity[1],ds->opacity[2],	\
			ds->accumulatedOpacity[0],ds->accumulatedOpacity[1],ds->accumulatedOpacity[2]);					\
		if(ds==nSample) {										\
			if(ds==&pixel->last) printf("*");					\
			printf("*\n");										\
		} else {												\
			printf("\n");										\
		}														\
		ds = ds->prev;											\
	}															\
	printf("\n");												\
}

#define updateTransparent(dfIf,dfElse) {															\
	vector O,rO;																					\
	const float *Oc;																				\
	CFragment *cSample	=	nSample->prev;															\
	movvv(O,cSample->accumulatedOpacity);															\
	if (O[0] < CRenderer::opacityThreshold[0] && O[1] < CRenderer::opacityThreshold[1] && O[2] < CRenderer::opacityThreshold[2]) {	\
		/* not already opaque */																	\
		cSample = nSample;																			\
	}																								\
	/* adjust accumulated opacities and test against threshold */									\
	initv(rO,1-O[0],1-O[1],1-O[2]);																	\
	while(cSample) {																				\
		Oc = cSample->opacity;																		\
		if (Oc[0] < 0 || Oc[1] < 0 || Oc[2] < 0) {													\
			rO[0] *= 1+Oc[0];																		\
			rO[1] *= 1+Oc[1];																		\
			rO[2] *= 1+Oc[2];																		\
		} else {																					\
			O[0] += Oc[0]*rO[0];																	\
			O[1] += Oc[1]*rO[1];																	\
			O[2] += Oc[2]*rO[2];																	\
			rO[0] *= 1-Oc[0];																		\
			rO[1] *= 1-Oc[1];																		\
			rO[2] *= 1-Oc[2];																		\
		}																							\
		movvv(cSample->accumulatedOpacity,O);														\
																									\
		if (O[0] > CRenderer::opacityThreshold[0] && O[1] > CRenderer::opacityThreshold[1] && O[2] > CRenderer::opacityThreshold[2]) {	\
			/* opaque after this point */															\
			CFragment *dSample	=	cSample->next;													\
			if (dSample && dSample != &pixel->last) {												\
				while(dSample && dSample != &pixel->last) {											\
					CFragment *tSample	=	dSample->next;											\
					deleteFragment(dSample);														\
					dSample				=	tSample;												\
				}																					\
				cSample->next		=	&pixel->last;												\
				pixel->last.prev	=	cSample;													\
				pixel->update		=	cSample;													\
				/*initv(pixel->last.color,0);				*/	\
				/*initv(pixel->last.opacity,0);				*/	\
				/*initv(pixel->last.accumulatedOpacity,1);	*/	\
				/*pixel->last.z = CRenderer::clipMax;		*/	\
				/*initv(cSample->accumulatedOpacity,1);		*/	\
			}																						\
			const float z			=	cSample->z;													\
			if (z < pixel->z) {																		\
				dfIf();																				\
				pixel->z			=	z;															\
				depthFilterTouchNode();																\
			} dfElse();																				\
			break;																					\
		}																							\
		cSample = cSample->next;																	\
	}																								\
}

#define DEFINE_STOCHASTIC_FUNCTIONS
#include "stochasticPrimitives.h"
#undef DEFINE_STOCHASTIC_FUNCTIONS

#undef depthFilterIfZMin
#undef depthFilterElseZMin
#undef depthFilterIfZMid
#undef depthFilterElseZMid
#undef findSample
#undef updateOpaque
