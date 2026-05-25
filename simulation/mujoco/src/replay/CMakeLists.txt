find_path(MCAP_INCLUDE_DIR
  NAMES mcap/reader.hpp
  PATHS
    /opt/ros/humble/include/mcap_vendor
)
find_library(MCAP_LIBRARY
  NAMES mcap
  PATHS
    /opt/ros/humble/lib
    ${SIMULATION_MUJOCO_DIR}/../../lib
)
if(NOT MCAP_INCLUDE_DIR OR NOT MCAP_LIBRARY)
  message(FATAL_ERROR "Failed to find MCAP headers/library for mujoco_log_replay")
endif()

add_library(mujoco_log_replay_lib STATIC)
target_sources(mujoco_log_replay_lib PRIVATE src/mujoco_log_replay.cc)
target_include_directories(mujoco_log_replay_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${SIMULATION_MUJOCO_DIR}/../../core/include>
    $<BUILD_INTERFACE:${SIMULATION_MUJOCO_DIR}/../../core/include/data>
    $<BUILD_INTERFACE:${MCAP_INCLUDE_DIR}>
)
target_link_libraries(mujoco_log_replay_lib
  PUBLIC
    lcm
    ${MCAP_LIBRARY}
    yaml-cpp::yaml-cpp
)

add_executable(mujoco_log_replay)
target_sources(mujoco_log_replay PRIVATE src/mujoco_log_replay_main.cc)
target_link_libraries(mujoco_log_replay PRIVATE mujoco_log_replay_lib)
set_target_properties(mujoco_log_replay PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

install(TARGETS mujoco_log_replay
  RUNTIME DESTINATION bin
)

if(BUILD_TESTS)
  add_googletest(test_mujoco_log_replay
    SRCS
      tests/test_mujoco_log_replay.cc
    DEPS
      GTest::GTest
      GTest::Main
      pthread
      mujoco_log_replay_lib
  )
endif()
