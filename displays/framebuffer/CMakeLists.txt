set (SOURCES fbx.cpp framebuffer.cpp)
set (INCS ../../ ../../core ./)

include_directories(${INCS})
add_library(framebuffer MODULE ${SOURCES} ${SOURCESC})
target_link_libraries(framebuffer common ${X11_LIBRARIES} pthread)

install(TARGETS framebuffer DESTINATION ${CMAKE_BINARY_DIR}/pixie/displays)
