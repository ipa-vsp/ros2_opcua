macro(generate_datatype_headers TARGET)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  message("Generating datatype headers for ${TARGET}...")
  message("YAML path: ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}.yaml")
  message("Output path: ${CMAKE_BINARY_DIR}/include/${TARGET}.hpp")
  message("Template directory: ${CMAKE_INSTALL_PREFIX}/bin/templates")

  add_custom_target(
    ${TARGET}_prepare ALL
    COMMAND rm -rf ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/include/${PROJECT_NAME}/${TARGET}.hpp
    COMMAND rm -rf ${CMAKE_BINARY_DIR}/include/
    COMMAND mkdir -p ${CMAKE_BINARY_DIR}/include
    #COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/${TARGET}/
  )

  message("Prepare Target")

  add_custom_target(
        ${TARGET} ALL
        DEPENDS ${TARGET}_prepare
  )
  
  message("Build Header file()")

  add_custom_command(
    TARGET ${TARGET} POST_BUILD
    COMMAND dtgen ${CMAKE_CURRENT_SOURCE_DIR}/config/${TARGET}.yaml ${CMAKE_BINARY_DIR}/include/${TARGET}.hpp
  )

  if(NOT EXISTS ${CMAKE_BINARY_DIR}/include/${TARGET}.hpp)
    message(FATAL_ERROR "Failed to generate datatype headers from YAML")
  endif()

  install(FILES ${CMAKE_BINARY_DIR}/include/${TARGET}.hpp 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
  
endmacro()