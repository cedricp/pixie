#ifdef USE_VDB

#ifndef VDB_READ_H
#define VDB_READ_H

#include <common/os.h>
#include <string>
#include <map>

struct vdb_grids;
struct vdb_grid;
struct active_grid;

class VdbReader{
public:
    static VdbReader* getVdbReader();

    vdb_grid* getGrid(std::string filename, int gridnum);
    bool 	getElement(float color[3], std::string filename, int gridnum, const float pos[3], int linear);
    bool    getSize(std::string filename, int gridnum, float size[3]);
    bool    getPosition(std::string filename, int gridnum, float pos[3]);
private:
    VdbReader();
    ~VdbReader();
	std::map<std::string, vdb_grids*> vdb_map;
	TMutex	  mutex;
};

#endif

#endif
