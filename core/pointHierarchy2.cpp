//////////////////////////////////////////////////////////////////////
//
//                             Pixie
//
// Copyright © 1999 - 2003, Okan Arikan
//
// Contact: okan@cs.utexas.edu
//
//	This library is free software; you can redistribute it and/or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//	This library is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//	Lesser General Public License for more details.
//
//	You should have received a copy of the GNU Lesser General Public
//	License along with this library; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
//
//  File				:	pointHierarchy.cpp
//  Classes				:	CPointHierarchy
//  Description			:
//
////////////////////////////////////////////////////////////////////////
#include "pointHierarchy2.h"
#include "error.h"
#include "random.h"
#include "shading.h"
#include "global_illum.h"

// The predefined names for the area and the radiosity channels
const	char	*areaChannelName		=	"_area";
const	char	*radiosityChannelName	=	"_radiosity";

///////////////////////////////////////////////////////////////////////
// Class				:	CPointHierarchy
// Method				:	CPointHierarchy
// Description			:	Ctor
// Return Value			:
// Comments				:
CPointHierarchy2::CPointHierarchy2(const char *n,const float *from,const float *to,FILE *in) : CMap<CPointCloudPoint>(), CTexture3d(n,from,to) {
	// Try to read the point cloud

	// Read the header
	readChannels(in);

	// Read the points
	CMap<CPointCloudPoint>::read(in);

	// Reserve the actual space
	data.reserve(numItems*dataSize);

	// Read the data
	fread(data.array,sizeof(float),numItems*dataSize,in);
	data.numItems	=	numItems*dataSize;

	// Close the file
	fclose(in);

	// Find the indices for area and radiosity
	areaIndex		=	-1;
	radiosityIndex	=	-1;
	int	i;
	for (i=0;i<numChannels;i++) {
		if		((strcmp(channels[i].name,areaChannelName) == 0)		&& (channels[i].numSamples == 1))	areaIndex		=	channels[i].sampleStart;
		else if ((strcmp(channels[i].name,radiosityChannelName) == 0)	&& (channels[i].numSamples == 3))	radiosityIndex	=	channels[i].sampleStart;
	}

    int numPoints = CMap<CPointCloudPoint>::numItems;

    std::vector<const CPointCloudPoint*> workspace(numPoints);

    Imath::Box3f bound;
#pragma omp for
    for(size_t i = 0; i < numPoints; ++i)
    {
        CPointCloudPoint* ppp = CMap<CPointCloudPoint>::items + i;
        workspace[i] =	ppp;
        const float* p = ppp->P;
        bound.extendBy(Imath::V3f(p[0], p[1], p[2]));
    }


    node_root = buildHierarchy(0, &workspace[0], numPoints, dataSize, bound);
}

///////////////////////////////////////////////////////////////////////
// Class				:	CPointHierarchy
// Method				:	~CPointHierarchy
// Description			:	Dtor
// Return Value			:
// Comments				:
CPointHierarchy2::~CPointHierarchy2() {
    deleteTree(node_root);
}

void CPointHierarchy2::deleteTree( pointOctreeNode* n){
    if(!n) return;
    for(int i = 0; i < 8; ++i)
        deleteTree(n->children[i]);
    delete n;
}

void CPointHierarchy2::lookup(float *Cl,const float *Pl,const float *dPdul,const float *dPdvl,const float *Nl,CShadingContext *context){

	const CShadingScratch	*scratch		=	&(context->currentShadingState->scratch);
	const float				maxSolidAngle	=	scratch->occlusionParams.maxSolidAngle;
	const float				bias        	=	scratch->occlusionParams.bias;
	const float				coneAngle     	=	scratch->occlusionParams.coneAngle;
	const float				fallOff     	=	scratch->occlusionParams.fallOff;
	const int				cuberes     	=	(int)scratch->occlusionParams.cubeResolution;

	vector					P,N,Ntemp,Nnorm;

	// Transform the lookup point to the correct coordinate system
	mulmp(P,to,Pl);
	mulmn(N,from,Nl);

    normalizev(Nnorm,N);
	mulvf(Ntemp,Nnorm,bias);
	addvv(P,Ntemp);

	// Clear the data
	for (int i=0;i<dataSize;i++)	Cl[i]	=	0;

    if ( radiosityIndex > 0 ){
        RadiosityIntegrator integrator(cuberes);

        float cosConeAngle = cos(coneAngle);
        float sinConeAngle = sin(coneAngle);

        Imath::V3f PP(P[0], P[1], P[2]);
        Imath::V3f NN(N[0], N[1], N[2]);

        renderNode(integrator, PP, NN, cosConeAngle, sinConeAngle, maxSolidAngle, dataSize+6, node_root, radiosityIndex, areaIndex);

        float occ = 0;
        Imath::C3f col = integrator.radiosity(NN, coneAngle, &occ);

        Cl[0] = col.x ;
        Cl[1] = col.y ;
        Cl[2] = col.z ;
        Cl[3] = occ;
    } else {
        OcclusionIntegrator integrator(cuberes);

        float cosConeAngle = cos(coneAngle);
        float sinConeAngle = sin(coneAngle);

        Imath::V3f PP(P[0], P[1], P[2]);
        Imath::V3f NN(N[0], N[1], N[2]);

        renderNode(integrator, PP, NN, cosConeAngle, sinConeAngle, maxSolidAngle, dataSize+6, node_root, radiosityIndex, areaIndex);

        float occ = integrator.occlusion(NN, coneAngle);

        Cl[3] = occ;
    }
}

pointOctreeNode* CPointHierarchy2::buildHierarchy(int depth, const CPointCloudPoint** points, size_t npoints, int datasize,const Imath::Box3f& bnd){
    pointOctreeNode* node = new pointOctreeNode;

    node->bound = bnd;

    const int realDataSize = datasize + 6;

    Imath::V3f c = bnd.center();
    node->center = c;
    Imath::V3f diag = bnd.size();
    node->boundRadius = diag.length()/2.0f;
    node->npoints = 0;
    size_t pointsPerLeaf = 8;

    // Limit max depth of tree to prevent infinite recursion when
    // greater than pointsPerLeaf points lie at the same position in
    // space.  floats effectively have 24 bit of precision in the
    // significand, so there's never any point splitting more than 24
    // times.
    int maxDepth = 24;

    if(npoints <= pointsPerLeaf || depth >= maxDepth)
    {
        if ( npoints > 8){
            node->npoints = 0;
            return node;
        }
        // Small number of child points: make this a leaf node and
        // store the points directly in the data member.
        node->npoints = npoints;
        // Copy over data into node.
        node->data.reserve(npoints*realDataSize);
        float sumA = 0;
        Imath::V3f sumP(0);
        Imath::V3f sumN(0);
        Imath::C3f sumCol(0);

        for(size_t j = 0; j < npoints; ++j)
        {
            const float* p = points[j]->P;
            const float* n = points[j]->N;
            const float* dd = data.array + points[j]->entryNumber ;

            int offset = j*realDataSize;
            // copy extra data
            node->data[offset + 0] = p[0];
            node->data[offset + 1] = p[1];
            node->data[offset + 2] = p[2];

            node->data[offset + 3] = n[0];
            node->data[offset + 4] = n[1];
            node->data[offset + 5] = n[2];

            for(int z = 0; z < datasize; ++z){
                node->data[offset + z + 6] = dd[z];
            }

            // compute averages (area weighted)
            float area = sqrtf(dd[0]/M_PI);
            float A = area*area;
            sumA += A;
            sumP += A*Imath::V3f(p[0], p[1], p[2]);
            sumN += A*Imath::V3f(n[0], n[1], n[2]);
            sumCol += A*Imath::C3f(dd[1], dd[2], dd[3]);
        }

        node->aggP = 1.0f/sumA * sumP;
        node->aggN = sumN.normalized();
        node->aggR = sqrtf(sumA);
        node->aggCol = 1.0f/sumA * sumCol;

        return node;
    }
    // allocate extra workspace for storing child points (ugh!)
    std::vector<const CPointCloudPoint*> workspace(8*npoints,NULL);

    const CPointCloudPoint** w = &workspace[0];
    const CPointCloudPoint** P[8] = {
        w,             w + npoints,   w + (2*npoints), w + (3*npoints),
        w + (4*npoints), w + (5*npoints), w + (6*npoints), w + (7*npoints)
    };

    // Partition points into the eight child nodes
    size_t np[8] = {0};

    for(size_t i = 0; i < npoints; ++i)
    {
        const CPointCloudPoint* p = points[i];
        int cellIndex = 4*(p->P[2] > c.z) + 2*(p->P[1] > c.y) + (p->P[0] > c.x);
        P[cellIndex][np[cellIndex]++] = p;
    }

    // Recursively generate child nodes and compute position, normal
    // and radius for the current node.
    float sumA = 0;
    Imath::V3f sumP(0);
    Imath::V3f sumN(0);
    Imath::C3f sumCol(0);
    for(int i = 0; i < 8; ++i)
    {
        if(np[i] == 0)
            continue;
        Imath::Box3f bnd2;
        bnd2.min.x = (i     % 2 == 0) ? bnd.min.x : c.x;
        bnd2.min.y = ((i/2) % 2 == 0) ? bnd.min.y : c.y;
        bnd2.min.z = ((i/4) % 2 == 0) ? bnd.min.z : c.z;
        bnd2.max.x = (i     % 2 == 0) ? c.x : bnd.max.x;
        bnd2.max.y = ((i/2) % 2 == 0) ? c.y : bnd.max.y;
        bnd2.max.z = ((i/4) % 2 == 0) ? c.z : bnd.max.z;
        pointOctreeNode* child = buildHierarchy(depth+1, P[i], np[i], datasize, bnd2);
        node->children[i] = child;
        // Weighted average with weight = disk surface area.
        float A = child->aggR * child->aggR;
        sumA += A;
        sumP += A * child->aggP;
        sumN += A * child->aggN;
        sumCol += A * child->aggCol;
    }
    node->aggP = 1.0f/sumA * sumP;
    node->aggN = sumN.normalized();
    node->aggR = sqrtf(sumA);
    node->aggCol = 1.0f/sumA * sumCol;
    return node;
}

