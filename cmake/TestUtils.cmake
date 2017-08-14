#################################################
macro (ign_build_tests)
  # Find the Python interpreter for running the
  # check_test_ran.py script
  find_package(PythonInterp QUIET)

  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    if(USE_LOW_MEMORY_TESTS)
      add_definitions(-DUSE_LOW_MEMORY_TESTS=1)
    endif(USE_LOW_MEMORY_TESTS)
    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file})

    target_link_libraries(${BINARY_NAME}
      ${PROJECT_LIBRARY_TARGET_NAME}
      ${PROJECT_LIBRARY_TARGET_NAME}_headers
      gtest
      gtest_main)

    if (UNIX)
      target_link_libraries(${BINARY_NAME}
         pthread)
    endif()

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
      --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    target_include_directories(${BINARY_NAME}
      PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
             $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR_FULL}>)

    set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

    # set c++ standard used by target
    set_property(TARGET ${BINARY_NAME} PROPERTY CXX_STANDARD 11)

    if(PYTHONINTERP_FOUND)
      # Check that the test produced a result and create a failure if it didn't.
      # Guards against crashed and timed out tests.
      add_test(check_${BINARY_NAME} ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
        ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
    endif()
  endforeach()
endmacro()
