macro(generate_datatype_headers TARGET)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  message("Generating datatype headers for ${TARGET}...")
  message("YAML path: ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}.yml")
  message("Output path: ${CMAKE_BINARY_DIR}/include/${TARGET}.hpp")
  message("Template directory: ${CMAKE_INSTALL_PREFIX}/bin/templates")

  add_custom_command(
    COMMENT "Generating datatype headers from YAML .... "
    OUTPUT "${CMAKE_BINARY_DIR}/include/${TARGET}.hpp"
    COMMAND mkdir -p ${CMAKE_BINARY_DIR}/include/
    COMMAND dtgen "${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}.yml" "${CMAKE_BINARY_DIR}/include/${TARGET}.hpp"
    COMMENT "Generating datatype headers from YAML"
    MAIN_DEPENDENCY "${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}.yml"
    DEPENDS dtgen
  )

  add_custom_target(
    ${TARGET} ALL
    DEPENDS "${CMAKE_BINARY_DIR}/include/${TARGET}.hpp"
  )

  install(DIRECTORY ${CMAKE_BINARY_DIR}/include/
    DESTINATION include/
  )
  
  if(NOT EXISTS "${CMAKE_BINARY_DIR}/include/${TARGET}.hpp")
    message(FATAL_ERROR "Failed to generate datatype headers from YAML")
  endif()
endmacro()