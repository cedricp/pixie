set (SOURCES ptcfilter.cpp)

SET(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

include_directories(../)
add_executable(ptcfilter ${SOURCES})
target_link_libraries(ptcfilter core common dl pthread stdc++)

install(TARGETS ptcfilter DESTINATION ${CMAKE_BINARY_DIR}/pixie/bin)

