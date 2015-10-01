set (SOURCES openexr.cpp)
set (INCS ../../ ../../core ./ ${OPENEXR_INCLUDE_DIR})

include_directories(${INCS})
add_library(exr MODULE ${SOURCES})
target_link_libraries(exr common ${X11_LIBRARIES} pthread)

install(TARGETS exr DESTINATION ${CMAKE_BINARY_DIR}/pixie/displays)
