set   (SOURCES attributes.cpp linsys.cpp rendererFiles.cpp brickmap.cpp memory.cpp rendererJobs.cpp bsplinePatchgrid.cpp
		noise.cpp rendererMutexes.cpp bundles.cpp object.cpp rendererNetwork.cpp cubemap.cpp occlusion.cpp resource.cpp
		curves.cpp options.cpp reyes.cpp debug.cpp patches.cpp patchgrid.cpp ribOut.cpp
		delayed.cpp photon.cpp ri.cpp depository.cpp photonMap.cpp riInterface.cpp displayChannel.cpp pl.cpp
	    dlobject.cpp pointCloud.cpp	shader.cpp shaderPl.cpp error.cpp pointHierarchy.cpp
		shading.cpp execute.cpp points.cpp show.cpp executeMisc.cpp polygons.cpp stats.cpp
		fileResource.cpp ptcapi.cpp stochastic.cpp
		hcshader.cpp quadrics.cpp subdivision.cpp random.cpp subdivisionCreator.cpp
		implicitSurface.cpp raytracer.cpp surface.cpp
		init.cpp refCounter.cpp texmake.cpp irradiance.cpp remoteChannel.cpp
		texture3d.cpp rendererClipping.cpp texture.cpp
		rendererContext.cpp	trace.cpp renderer.cpp rendererDeclarations.cpp xform.cpp
		rendererDisplay.cpp zbuffer.cpp deepImage.cpp stochasticDraw.cpp vdb_read.cpp gi.cpp ss.cpp)
		
set   (HEADERS  atomic.h            object.h           scriptFunctions.h
                attributes.h        objectMisc.h       scriptOpcodes.h
                brickmap.h          occlusion.h        shadeop.h
                bsplinePatchgrid.h  options.h          shaderFunctions.h
                bundles.h           patches.h          shader.h
                cubemap.h           patchgrid.h        shaderOpcodes.h
                curves.h            patchUtils.h       shaderPl.h
                debug.h             photon.h           shading.h
                deepImage.h         photonMap.h        show.h
                delayed.h           pl.h               slcode.h
                depository.h        pointCloud.h       stats.h
                displayChannel.h    pointHierarchy2.h  stochastic.h
                dlobject.h          pointHierarchy.h   stochasticPoint.h
                dlo.h               points.h           stochasticPrimitives.h
                dso.h               polygons.h         stochasticQuad.h
                dsply.h             ptcapi.h           subdivisionCreator.h
                error.h             quadrics.h         subdivisionData.h
                fileResource.h      random.h           subdivision.h
                giFunctions.h       ray.h              surface.h
                giOpcodes.h         raytracer.h        texmake.h
                global_illum.h      refCounter.h       texture3d.h
                harmonics.h         remoteChannel.h    texture.h
                hcshader.h          rendererc.h        tiff.h
                implicit.h          rendererContext.h  userAttributes.h
                implicitSurface.h   renderer.h         variable.h
                irradiance.h        resource.h         vdb_read.h
                linsys.h            reyes.h            xform.h
                map.h               rib.h              zbuffer.h
                memory.h            ribOut.h           zbufferPoint.h
                netFileMapping.h    ri_config.h        zbufferQuad.h
                noise.h             ri.h
                noiseTables.h       riInterface.h      ptcommon.h)

		
set   (INCS ${Boost_INCLUDE_DIRS} ${OPENEXR_INCLUDE_DIR} ${OPENVDB_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../ ./)

BISON_TARGET(DsoParser dso.y ${CMAKE_CURRENT_BINARY_DIR}/dso.cpp COMPILE_FLAGS -pdso)
BISON_TARGET(RibParser rib.y ${CMAKE_CURRENT_BINARY_DIR}/rib.cpp COMPILE_FLAGS -prib)
BISON_TARGET(SdrParser sdr.y ${CMAKE_CURRENT_BINARY_DIR}/sdr.cpp COMPILE_FLAGS -psdr)
BISON_TARGET(VarParser variable.y ${CMAKE_CURRENT_BINARY_DIR}/variable.cpp COMPILE_FLAGS -pvariable)
BISON_TARGET(IfExprParser ifexpr.y ${CMAKE_CURRENT_BINARY_DIR}/ifexpr.cpp COMPILE_FLAGS -pifexpr)

FLEX_TARGET(DsoFlex dso.l ${CMAKE_CURRENT_BINARY_DIR}/lex.dso.cpp COMPILE_FLAGS "-i -Pdso")
FLEX_TARGET(RibFlex rib.l ${CMAKE_CURRENT_BINARY_DIR}/lex.rib.cpp COMPILE_FLAGS "-i -Prib")
FLEX_TARGET(SdrFlex sdr.l ${CMAKE_CURRENT_BINARY_DIR}/lex.sdr.cpp COMPILE_FLAGS "-i -Psdr")
FLEX_TARGET(VarFlex variable.l ${CMAKE_CURRENT_BINARY_DIR}/lex.variable.cpp COMPILE_FLAGS "-i -Pvariable")
FLEX_TARGET(IfExprFlex ifexpr.l ${CMAKE_CURRENT_BINARY_DIR}/lex.ifexpr.cpp COMPILE_FLAGS "-i -Pifexpr")

SET_SOURCE_FILES_PROPERTIES(${BISON_DsoParser_OUTPUTS} PROPERTIES OBJECT_DEPENDS ${FLEX_DsoFlex_OUTPUTS})
SET_SOURCE_FILES_PROPERTIES(${BISON_RibParser_OUTPUTS} PROPERTIES OBJECT_DEPENDS ${FLEX_RibFlex_OUTPUTS})
SET_SOURCE_FILES_PROPERTIES(${BISON_SdrParser_OUTPUTS} PROPERTIES OBJECT_DEPENDS ${FLEX_SdrFlex_OUTPUTS})
SET_SOURCE_FILES_PROPERTIES(${BISON_VarParser_OUTPUTS} PROPERTIES OBJECT_DEPENDS ${FLEX_VarFlex_OUTPUTS})
SET_SOURCE_FILES_PROPERTIES(${BISON_IfExprParser_OUTPUTS} PROPERTIES OBJECT_DEPENDS ${FLEX_IfExprFlex_OUTPUTS})

if (USE_VDB)
    add_definitions(-DUSE_VDB)
    set (LINK_VDB  ${TBB_LIBRARIES} ${OPENVDB_LIBRARIES}  ${Boost_LIBRARIES})
endif()

if (USE_ALEMBIC)
    add_definitions(-DUSE_ALEMBIC)
    set (LINK_ABC alembic ${ALEMBIC_LIBRARIES})
endif()

SET (exrlibs -Wl,--whole-archive ${OPENEXR_LIBRARIES} -Wl,--no-whole-archive)

include_directories(${INCS})
add_library(core SHARED ${SOURCES} ${HEADERS} ${BISON_DsoParser_OUTPUTS}
            ${BISON_RibParser_OUTPUTS} ${BISON_SdrParser_OUTPUTS}
            ${BISON_VarParser_OUTPUTS} ${BISON_IfExprParser_OUTPUTS}
            )
            
target_link_libraries(core ${exrlibs} ${TIFF_LIBRARIES} ${PNG_LIBRARY} ${LINK_VDB} ${LINK_ABC} z)
install(TARGETS core DESTINATION ${CMAKE_BINARY_DIR}/pixie/lib)
if (UNIX)
install(CODE "EXECUTE_PROCESS (COMMAND cp -L ${TBB_INSTALL_DIR}/lib/libtbb*.so* ${CMAKE_BINARY_DIR}/pixie/lib)")
install(CODE "EXECUTE_PROCESS (COMMAND cp -L ${OPENEXR_ROOT}/lib/lib*.so* ${CMAKE_BINARY_DIR}/pixie/lib)")
endif()
