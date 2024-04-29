macro(generate_datatype_headers TARGET)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  set(YAML_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}.yaml")
  set(OUTPUT_PATH "${CMAKE_BINARY_DIR}/include/${TARGET}.hpp")
  set(TEMPLATE_DIR "${CMAKE_INSTALL_PREFIX}/bin/templates")

  message("Generating datatype headers for ${TARGET}...")
  message("YAML path: ${YAML_PATH}")
  message("Output path: ${OUTPUT_PATH}")
  message("Template directory: ${TEMPLATE_DIR}")

  # Prepare target directories
  add_custom_target(
    ${TARGET}_prepare ALL
    COMMAND ${CMAKE_COMMAND} -E rm -f ${OUTPUT_PATH}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include
    COMMENT "Preparing target directories")

  # Main target to generate headers
  add_custom_command(
    OUTPUT ${OUTPUT_PATH}
    COMMAND dtgen ${YAML_PATH} ${OUTPUT_PATH}
    DEPENDS ${YAML_PATH} ${TARGET}_prepare
    COMMENT "Generating header file from YAML")

  add_custom_target(
    ${TARGET} ALL
    DEPENDS ${OUTPUT_PATH}
    COMMENT "Building header file")

  # Installation rule
  install(FILES ${OUTPUT_PATH}
          DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
endmacro()
