cmake_minimum_required(VERSION 3.16)
project(pa_firmware VERSION 1.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Two builds from one source tree.
#
# Host build (the default): the portable core plus its unit tests. No board, no
# cross-compiler, no vendor HAL. It also installs the core as a library, which
# is how ros2_ws/src/arm_interface gets the wire protocol: the bridge speaks
# the link by compiling the same encoder the firmware runs, rather than by
# keeping a second implementation in step with it.
#
# Cross build (-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake): the same core
# plus everything under src/platform, linked into an STM32H743 image.
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  if(CMAKE_CROSSCOMPILING)
    set(CMAKE_BUILD_TYPE Release)
  else()
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
  endif()
endif()

# The portable core: identical sources in both builds.
file(GLOB_RECURSE PA_CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
list(FILTER PA_CORE_SOURCES EXCLUDE REGEX ".*/src/platform/.*")
list(FILTER PA_CORE_SOURCES EXCLUDE REGEX ".*/src/main\\.cpp$")

add_library(pa_core STATIC ${PA_CORE_SOURCES})
add_library(pa_core::pa_core ALIAS pa_core)
target_include_directories(pa_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(pa_core PUBLIC cxx_std_17)

if(CMAKE_CROSSCOMPILING)
  # -------------------------------------------------------------------------
  # Target build: STM32H743VIT6 image.
  #
  # -Wconversion is off here. Register-level code casts between widths by
  # nature and the noise would bury the warnings worth reading; the portable
  # core is still compiled with it in the host build, which is where the
  # arithmetic actually lives.
  # -------------------------------------------------------------------------
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)

  file(GLOB_RECURSE PA_PLATFORM_SOURCES CONFIGURE_DEPENDS src/platform/*.cpp)

  add_executable(pa_firmware.elf ${PA_PLATFORM_SOURCES} src/main.cpp)
  target_include_directories(pa_firmware.elf PRIVATE include src)
  target_link_libraries(pa_firmware.elf PRIVATE pa_core)

  set(PA_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32h743xi.ld)
  target_link_options(pa_firmware.elf PRIVATE
    -T${PA_LINKER_SCRIPT}
    -nostartfiles
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,-Map=pa_firmware.map
    -Wl,--print-memory-usage
  )
  target_link_libraries(pa_firmware.elf PRIVATE m)
  set_target_properties(pa_firmware.elf PROPERTIES LINK_DEPENDS ${PA_LINKER_SCRIPT})

  add_custom_command(TARGET pa_firmware.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:pa_firmware.elf> pa_firmware.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:pa_firmware.elf> pa_firmware.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:pa_firmware.elf>
    COMMENT "Generating pa_firmware.bin / .hex and reporting size"
  )

else()
  # -------------------------------------------------------------------------
  # Host build: unit tests and the installable core.
  # -------------------------------------------------------------------------
  if(MSVC)
    target_compile_options(pa_core PRIVATE /W4)
  else()
    target_compile_options(pa_core PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
  endif()

  option(PA_FIRMWARE_BUILD_TESTS "Build the host unit tests" ON)

  if(PA_FIRMWARE_BUILD_TESTS)
    enable_testing()
    file(GLOB PA_TEST_SOURCES CONFIGURE_DEPENDS test/test_*.cpp)
    foreach(test_src ${PA_TEST_SOURCES})
      get_filename_component(test_name ${test_src} NAME_WE)
      add_executable(${test_name} ${test_src})
      target_link_libraries(${test_name} PRIVATE pa_core)
      target_include_directories(${test_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/test
        ${CMAKE_CURRENT_SOURCE_DIR}/../test_support)
      if(NOT MSVC)
        target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
      endif()
      add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
  endif()

  # -------------------------------------------------------------------------
  # Install, so the ROS 2 bridge can find_package(pa_core) and link the same
  # protocol implementation the board runs.
  # -------------------------------------------------------------------------
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  install(TARGETS pa_core
    EXPORT pa_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  install(EXPORT pa_coreTargets
    FILE pa_coreTargets.cmake
    NAMESPACE pa_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pa_core
  )
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/pa_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
  )
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/pa_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/pa_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pa_core
  )
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/pa_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/pa_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pa_core
  )
endif()
