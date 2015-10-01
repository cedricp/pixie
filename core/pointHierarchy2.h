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
//  File				:	pointHierarchy.h
//  Classes				:
//  Description			:
//
////////////////////////////////////////////////////////////////////////
#ifndef POINTHIERARCHY2_H
#define POINTHIERARCHY2_H

#include "common/global.h"
#include "common/containers.h"
#include "map.h"
#include "texture3d.h"
#include "options.h"
#include "pointCloud.h"
#include <vector>
#include <OpenEXR/ImathVec.h>
#include <OpenEXR/ImathBox.h>

class pointOctreeNode;

struct PointArray
{
public:
    int stride;
    std::vector<float> data;

    /// Get number of points in cloud
    size_t size() const { return data.size()/stride; }

    /// Get centroid of point cloud.
    Imath::V3f centroid() const
    {
        Imath::V3f sum(0);
        for(std::vector<float>::const_iterator p = data.begin();
            p < data.end(); p += stride)
        {
            sum += Imath::V3f(p[0], p[1], p[2]);
        }
        return (1.0f/data.size()*stride) * sum;
    }
};

///////////////////////////////////////////////////////////////////////
// Class				:	CPointHierarchy
// Description			:	A hierarchy of points
// Comments				:
class	CPointHierarchy2 : public CTexture3d, public CMap<CPointCloudPoint> {

public:


    CPointHierarchy2(const char *,const float *from,const float *to,FILE *);
    ~CPointHierarchy2();

	void					lookup(float *,const float *,const float *,const float *,const float *,CShadingContext *);
	void					store(const float *,const float *,const float *,float)	{	assert(FALSE);	}
	void					lookup(float *,const float *,const float *,float)		{	assert(FALSE);	}
	int                     nodeDataSize() const { return node_datasize; }
	const                   pointOctreeNode* root() const { return node_root; }
	pointOctreeNode*        buildHierarchy(int depth, const CPointCloudPoint** points, size_t npoints, int dataSize,const Imath::Box3f& bound);

protected:
	void					draw() { }
	void					bound(float *bmin,float *bmax) { }

private:
    void                    deleteTree( pointOctreeNode* n);

	CArray<float>			data;					// This is where we actually keep the data
	int						areaIndex;				// Index of the area variable
	int						radiosityIndex;			// Index of the radiosity variable

	int                     node_datasize;
	pointOctreeNode*        node_root;


};

#endif
