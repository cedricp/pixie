
set (HEADERS algebra.h comments.h global.h os.h simd.h align.h containers.h mathSpec.h polynomial.h)
set (SOURCES algebra.cpp os.cpp)
set (INCS ${CMAKE_CURRENT_SOURCE_DIR}/../)

include_directories(${INCS})
add_library(common STATIC ${SOURCES})

