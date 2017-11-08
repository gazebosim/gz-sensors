find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/ignition.in
                 ${CMAKE_BINARY_DIR}/ignition.dox @ONLY)

  # The ogre doxygen file is quite large, so we are using a compressed version.
  execute_process(COMMAND ${CMAKE_COMMAND} -E tar xvf
    ${CMAKE_SOURCE_DIR}/doc/ogre_1-9.tag.xml.tgz
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/doc)

  add_custom_target(doc
    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/ignition.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}

    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/ignition_logo.svg
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMENT "Generating API documentation with Doxygen" VERBATIM)

  install(FILES ${CMAKE_BINARY_DIR}/doc/${PROJECT_NAME_LOWER}.tag.xml 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ignition/${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}_${PROJECT_MINOR_VERSION})
endif()
