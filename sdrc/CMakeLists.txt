set (SOURCES dso.cpp opcodes.cpp sdr.cpp expression.cpp sdrc.cpp)
set (SOURCESC pp1.c pp2.c pp3.c pp4.c pp5.c pp6.c pp7.c pp8.c)
set (INCS ../ ../core ./)

BISON_TARGET(DsoParser ../core/dso.y ${CMAKE_CURRENT_BINARY_DIR}/dso.cpp COMPILE_FLAGS -pdso)
BISON_TARGET(SlParser sl.y ${CMAKE_CURRENT_BINARY_DIR}/sl.cpp COMPILE_FLAGS -psl)
FLEX_TARGET(DsoFlex ../core/dso.l ${CMAKE_CURRENT_BINARY_DIR}/lex.dso.cpp COMPILE_FLAGS "-i -Pdso")
FLEX_TARGET(SlFlex sl.l ${CMAKE_CURRENT_BINARY_DIR}/lex.sl.cpp COMPILE_FLAGS "-i -Psl")

SET_SOURCE_FILES_PROPERTIES(${BISON_DsoParser_OUTPUTS} PROPERTIES OBJECT_DEPENDS ${FLEX_DsoFlex_OUTPUTS})
SET_SOURCE_FILES_PROPERTIES(${BISON_SlParser_OUTPUTS} PROPERTIES OBJECT_DEPENDS ${FLEX_SlFlex_OUTPUTS})

include_directories(${INCS} ${CMAKE_CURRENT_BINARY_DIR})
add_executable(sdrc ${SOURCES} ${SOURCESC} ${BISON_DsoParser_OUTPUTS} ${BISON_SlParser_OUTPUTS})
target_link_libraries(sdrc common dl pthread stdc++)

install(TARGETS sdrc DESTINATION ${CMAKE_BINARY_DIR}/pixie/bin)
