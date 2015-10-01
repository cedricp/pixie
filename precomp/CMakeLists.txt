set (SOURCES precomp.cpp)

SET(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

include_directories(../)
add_executable(precomp ${SOURCES})
target_link_libraries(precomp common dl pthread stdc++)

