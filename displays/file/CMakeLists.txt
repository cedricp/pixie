set (SOURCES file.cpp file_png.cpp file_tiff)
set (INCS ../../ ../../core ./)

include_directories(${INCS})
add_library(file MODULE ${SOURCES} ${SOURCESC})
target_link_libraries(file common pthread)

install(TARGETS file DESTINATION ${CMAKE_BINARY_DIR}/pixie/displays)
