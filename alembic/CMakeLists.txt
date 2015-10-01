set   (SOURCES ArbAttrUtil.cpp  ProcArgs.cpp  SampleUtil.cpp  WriteGeo.cpp
               PathUtil.cpp     SubDTags.cpp  WriteMaterial.cpp
               AbcArchive.cpp)
		
set   (HEADERS  ArbAttrUtil.h  ProcArgs.h    SubDTags.h  WriteMaterial.h
                PathUtil.h     SampleUtil.h  WriteGeo.h  AbcArchive.h)

set   (INCS ${ALEMBIC_INCLUDE_DIR} ${OPENEXR_INCLUDE_DIRS} ../core ./)

include_directories(${INCS})

add_library( alembic STATIC ${SOURCES} ${HEADERS} )
target_link_libraries(alembic)


