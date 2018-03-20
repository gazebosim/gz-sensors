ign_add_executable(VALGRIND_manager_memcheck manager_memcheck.cc)
target_link_libraries(VALGRIND_manager_memcheck
  ${PROJECT_LIBRARY_TARGET_NAME}
  ${IGNITION-TRANSPORT_LIBRARIES})
