#ifdef USE_VDB

#include "vdb_read.h"
#include <openvdb/openvdb.h>

template <class T>
inline
T trilinear_base(T V000, T V100,
				 T V010, T V001,
				 T V101, T V011,
				 T V110, T V111,
				 float x, float y, float z)
{
	T V_out =	V000 * (1 - x) * (1 - y) * (1 - z) +
				V100 * x * (1 - y) * (1 - z) +
				V010 * (1 - x) * y * (1 - z) +
				V001 * (1 - x) * (1 - y) * z +
				V101 * x * (1 - y) * z +
				V011 * (1 - x) * y * z +
				V110 * x * y * (1 - z) +
				V111 * x * y * z;

	return V_out;
}

struct vdb_grid
{
	openvdb::CoordBBox 		bbox;
	openvdb::Coord 			res;
	openvdb::Vec3d			size;
	openvdb::Vec3d			offset;
	openvdb::GridBase::Ptr 	vdb_grid;
};

struct vdb_grids
{
	openvdb::io::File* 			file;
	std::map< int, vdb_grid* > 	vdb_grids;
};


static VdbReader* openVdbReaderInstance = NULL;

VdbReader::VdbReader()
{
    openvdb::initialize();
	osCreateMutex(mutex);
}

VdbReader::~VdbReader()
{
	std::map< std::string, vdb_grids* >::iterator it;
	for (it = vdb_map.begin(); it != vdb_map.end(); ++it){
		it->second->file->close();
		delete it->second->file;
		delete it->second;
	}
	osDeleteMutex(mutex);
}

vdb_grid* VdbReader::getGrid(std::string filename, int gridnum)
{
	vdb_grids* current_vdb_grids = NULL;
	vdb_grid*  current_grid      = NULL;
    openvdb::GridBase::Ptr baseGrid;

	if (vdb_map.find(filename) != vdb_map.end()){
		current_vdb_grids = vdb_map[filename];
	} else {
		vdb_grids* vdbgrids = new vdb_grids;
		vdb_map[filename] = vdbgrids;

		openvdb::io::File* vdb_file = new openvdb::io::File(filename);

		try {
			vdb_file->open();
		} catch (openvdb::Exception& e) {
			std::cerr << e.what() << " (" << filename << ")" << std::endl;
			return NULL;
		}
		vdbgrids->file 		= vdb_file;

		current_vdb_grids = vdbgrids;
	}

	if (current_vdb_grids->vdb_grids.find(gridnum) != current_vdb_grids->vdb_grids.end()){
		current_grid = current_vdb_grids->vdb_grids[gridnum];
	} else {
		if (gridnum >= current_vdb_grids->file->getGrids()->size()){
			std::cerr <<"Vdb file " << filename << " : grid overflow : #" << gridnum << std::endl;
			return NULL;
		}
		current_grid = new vdb_grid;
		current_vdb_grids->vdb_grids[gridnum] = current_grid;
		baseGrid = current_vdb_grids->file->readGrid(current_vdb_grids->file->getGrids()->at(gridnum)->getName());
	    current_vdb_grids->vdb_grids[gridnum]->vdb_grid = baseGrid;
	    current_grid->vdb_grid = baseGrid;
		current_grid->bbox = baseGrid->evalActiveVoxelBoundingBox();
		current_grid->res  = current_grid->bbox.dim();
		current_grid->size = openvdb::Vec3d(baseGrid->voxelSize());
		openvdb::math::Vec3d XYZ(current_grid->bbox.getCenter().x(),
								 current_grid->bbox.getCenter().y(),
								 current_grid->bbox.getCenter().z());
		current_grid->offset = baseGrid->indexToWorld(XYZ);
	}

    return current_grid;
}

bool VdbReader::getSize(std::string filename, int gridnum, float pos[3])
{
	osLock(mutex);
	vdb_grid* active_grid = getGrid(filename, gridnum);
	osUnlock(mutex);

	if (!active_grid){
		return false;
	}

	pos[0] = active_grid->size.x();
	pos[1] = active_grid->size.y();
	pos[2] = active_grid->size.z();
	return true;
}

bool VdbReader::getPosition(std::string filename, int gridnum, float pos[3])
{
	osLock(mutex);
	vdb_grid* active_grid = getGrid(filename, gridnum);
	osUnlock(mutex);

	if (!active_grid){
		return false;
	}

	pos[0] = active_grid->offset.x();
	pos[1] = active_grid->offset.y();
	pos[2] = active_grid->offset.z();
	return true;
}

bool VdbReader::getElement(float color[3], std::string filename, int gridnum, const float pos[3], int linear)
{
	osLock(mutex);
	vdb_grid* active_grid = getGrid(filename, gridnum);
	osUnlock(mutex);

	if (!active_grid)
		return .7;
	openvdb::math::Coord pnt((int)pos[0], (int)pos[1], (int)pos[2]);
	if (!active_grid->bbox.isInside(pnt)){
		color[0] = color[1] = color[2] = 0.;
		return false;
	}
	openvdb::GridBase::Ptr baseGrid = active_grid->vdb_grid;
	std::string value_type = baseGrid->valueType();

	openvdb::Coord xyz1((int)floor(pos[0]), (int)floor(pos[1]), (int)floor(pos[2]));
	openvdb::Coord xyz2((int) ceil(pos[0]), (int)floor(pos[1]), (int)floor(pos[2]));
	openvdb::Coord xyz3((int)floor(pos[0]), (int) ceil(pos[1]), (int)floor(pos[2]));
	openvdb::Coord xyz4((int)floor(pos[0]), (int)floor(pos[1]), (int) ceil(pos[2]));
	openvdb::Coord xyz5((int) ceil(pos[0]), (int)floor(pos[1]), (int) ceil(pos[2]));
	openvdb::Coord xyz6((int)floor(pos[0]), (int) ceil(pos[1]), (int) ceil(pos[2]));
	openvdb::Coord xyz7((int) ceil(pos[0]), (int) ceil(pos[1]), (int)floor(pos[2]));
	openvdb::Coord xyz8((int) ceil(pos[0]), (int) ceil(pos[1]), (int) ceil(pos[2]));

	if (value_type == "float"){
		openvdb::FloatGrid::Ptr 	 grid_ptr 	= openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
		openvdb::FloatGrid::Accessor accessor	= grid_ptr->getAccessor();
		float ret;
		if (!linear)
			ret =- accessor.getValue(xyz1);
		else
			ret = (float)trilinear_base(accessor.getValue(xyz1), accessor.getValue(xyz2),
										 accessor.getValue(xyz3), accessor.getValue(xyz4),
										 accessor.getValue(xyz5), accessor.getValue(xyz6),
										 accessor.getValue(xyz7), accessor.getValue(xyz8),
										 fmodf(pos[0], 1.0), fmodf(pos[1], 1.0), fmodf(pos[2], 1.0));
		color[0] = ret;
		color[1] = ret;
		color[2] = ret;
		return true;
	}

	if (value_type == "Vec3f"){
		openvdb::Vec3fGrid::Ptr 	 grid_ptr 	= openvdb::gridPtrCast<openvdb::Vec3fGrid>(baseGrid);
		openvdb::Vec3fGrid::Accessor accessor	= grid_ptr->getAccessor();
		openvdb::Vec3f ret;

		if (!linear)
			ret = accessor.getValue(xyz1);
		else
			ret = (openvdb::Vec3f)trilinear_base(accessor.getValue(xyz1), accessor.getValue(xyz2),
												 accessor.getValue(xyz3), accessor.getValue(xyz4),
												 accessor.getValue(xyz5), accessor.getValue(xyz6),
												 accessor.getValue(xyz7), accessor.getValue(xyz8),
												 fmodf(pos[0], 1.0), fmodf(pos[1], 1.0), fmodf(pos[2], 1.0));
		color[0] = ret.x();
		color[1] = ret.y();
		color[2] = ret.z();
		return true;
	}

	std::cerr << "Bad value type : " << value_type << std::endl;
	return false;
}

VdbReader* VdbReader::getVdbReader()
{
	if(openVdbReaderInstance == NULL){
		openVdbReaderInstance =  new VdbReader;
	}

	return openVdbReaderInstance;
}

#endif
