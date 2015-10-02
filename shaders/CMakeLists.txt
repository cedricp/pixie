file (GLOB SHADERS *.sl)

install(FILES ${SHADERS} DESTINATION ${CMAKE_BINARY_DIR}/pixie/shaders)

MACRO(SDR_TARGET Name Input Output)
 SET(SDR_TARGET_outputs "${Output}")
 ADD_CUSTOM_COMMAND(OUTPUT ${Output}
      COMMAND "${CMAKE_BINARY_DIR}/sdrc/sdrc"
      ARGS -o ${Output} ${Input}
      DEPENDS ${Input}
      COMMENT "[SDR][${Name}] Building shader with sdrc"
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
 SET(SDRC_${Name}_OUTPUTS ${SDR_TARGET_outputs})
ENDMACRO(SDR_TARGET)

foreach (SOURCE ${SHADERS})
    get_filename_component(SDRBASENAME ${SOURCE} NAME_WE)
    set(OUTPUT ${CMAKE_BINARY_DIR}/${SDRBASENAME}.sdr)
    SDR_TARGET(${SDRBASENAME} ${SOURCE} ${OUTPUT})
    list(APPEND SDRS ${OUTPUT})
endforeach()

add_custom_target(deps ALL DEPENDS ${SDRS} sdrc)
install(FILES ${SDRS} DESTINATION ${CMAKE_BINARY_DIR}/pixie/shaders)
