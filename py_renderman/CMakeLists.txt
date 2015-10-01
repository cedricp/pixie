INCLUDE(${SWIG_USE_FILE})

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH} ../core ./)

set(SWIG_HEADERS
    ../core/ri.h
)

set(SWIG_MODULE_ri_EXTRA_DEPS ${SWIG_HEADERS})
SWIG_ADD_MODULE(ri python pyri.i)
SWIG_LINK_LIBRARIES(ri ${PYTHON_LIBRARIES} core common)

install(DIRECTORY py_ri_tests DESTINATION ${CMAKE_BINARY_DIR}/pixie/examples)
install(TARGETS _ri DESTINATION ${CMAKE_BINARY_DIR}/pixie/python)
install(FILES ${CMAKE_BINARY_DIR}/py_renderman/ri.py DESTINATION ${CMAKE_BINARY_DIR}/pixie/python)

