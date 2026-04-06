cmake_minimum_required(VERSION 3.0.0)

project(viri)

# Standard catkin setup with message generation
find_package(catkin REQUIRED
  COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rosgraph_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Declare message files
add_message_files(
  FILES
  RLAction.msg
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Catkin package declaration
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    message_runtime
  DEPENDS
    Eigen3
)

include_directories(${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architecture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
add_compile_options(-O3)

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g")

# Gazebo RL Training Nodes (using standard CMake, not catkin_simple)
add_executable(gazebo_rl_agent_node src/training/gazebo_rl_agent_node.cpp)
add_dependencies(gazebo_rl_agent_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(gazebo_rl_agent_node ${catkin_LIBRARIES} Eigen3::Eigen stdc++fs)

add_executable(state_monitor_node src/training/state_monitor_node.cpp)
add_dependencies(state_monitor_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(state_monitor_node ${catkin_LIBRARIES} stdc++fs)

# Python Script Installation
catkin_install_python(PROGRAMS
    scripts/yolo_stereo_node.py
    scripts/target_manager.py
    scripts/test_action_publisher.py
    scripts/thrust_diagnostic.py
    scripts/test_manual_control.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install targets
install(TARGETS gazebo_rl_agent_node state_monitor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Old targets (commenting out for now - use only if needed)
# If you need the other targets, uncomment and add back catkin_simple support