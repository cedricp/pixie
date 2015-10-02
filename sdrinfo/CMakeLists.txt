set (SOURCES sdrinfo.cpp)

SET(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

include_directories(../)
add_executable(sdrinfo ${SOURCES})
target_link_libraries(sdrinfo sdr common dl pthread stdc++)

install(TARGETS sdrinfo DESTINATION ${CMAKE_BINARY_DIR}/pixie/bin)

