set   (SOURCES sdr.cpp)
		
set   (HEADERS  sdr.h)

		
set   (INCS ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../ ./)

BISON_TARGET(SdrParser sdr.y ${CMAKE_CURRENT_BINARY_DIR}/sdr.cpp COMPILE_FLAGS -psdr)

FLEX_TARGET(SdrFlex sdr.l ${CMAKE_CURRENT_BINARY_DIR}/lex.sdr.cpp COMPILE_FLAGS "-i -Psdr")

SET_SOURCE_FILES_PROPERTIES(${BISON_SdrParser_OUTPUTS} PROPERTIES OBJECT_DEPENDS ${FLEX_SdrFlex_OUTPUTS})


include_directories(${INCS})

add_library( sdr SHARED ${SOURCES} ${HEADERS} ${BISON_SdrParser_OUTPUTS} )
            
target_link_libraries(sdr z)
install(TARGETS sdr DESTINATION ${CMAKE_BINARY_DIR}/pixie/lib)
