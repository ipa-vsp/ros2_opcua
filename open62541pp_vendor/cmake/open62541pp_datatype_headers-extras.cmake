macro(generate_datatype_headers TARGET)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  message("Generating datatype headers for ${TARGET}...")
  message("YAML path: ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}.yml")
  message("Output path: ${CMAKE_BINARY_DIR}/include/${TARGET}.hpp")
  message("Template directory: ${CMAKE_INSTALL_PREFIX}/bin/templates")

  add_custom_target(
    ${TARGET}_prepare ALL
    COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/include/${PROJECT_NAME}/${TARGET}.hpp
    COMMAND rm -rf ${CMAKE_BINARY_DIR}/include/
    COMMAND mkdir -p ${CMAKE_BINARY_DIR}/include
    #COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}/
  )
	message("Creating ${TARGET}_prepare target")
  
endmacro()