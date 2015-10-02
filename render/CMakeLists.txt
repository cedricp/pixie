set (SOURCES pixie.cpp)

SET(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

include_directories(../)
add_executable(pixie_render ${SOURCES})
target_link_libraries(pixie_render core common dl pthread stdc++)

install(TARGETS pixie_render DESTINATION ${CMAKE_BINARY_DIR}/pixie/bin)

