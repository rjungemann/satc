cmake_minimum_required(VERSION 3.15)

project(satc LANGUAGES C)

option(SATC_BUILD_TESTS "Build the satc test executable" ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(satc INTERFACE)
add_library(satc::satc ALIAS satc)

target_include_directories(satc
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(satc INTERFACE c_std_99)

if(UNIX AND NOT APPLE)
  target_link_libraries(satc INTERFACE m)
endif()

install(TARGETS satc EXPORT satcTargets)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/satc.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  EXPORT satcTargets
  FILE satcTargets.cmake
  NAMESPACE satc::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/satc
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/satcConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/satcConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/satc
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/satcConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/satcConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/satcConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/satc
)

if(SATC_BUILD_TESTS)
  add_executable(satc-test satc-test.c)
  target_link_libraries(satc-test PRIVATE satc)
  set_target_properties(satc-test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)

  if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(satc-test PRIVATE -Wall -O3)
  endif()

  include(CTest)
  if(BUILD_TESTING)
    add_test(NAME satc-test COMMAND satc-test)
  endif()
endif()