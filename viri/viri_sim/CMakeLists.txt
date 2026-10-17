cmake_minimum_required(VERSION 3.0.2)
project(viri_sim)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Must match flightlib's flags: mismatched Eigen alignment/vectorization between translation
# units sharing flightlib types causes heap corruption at runtime.
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  set(ARCH_FLAGS "-Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
  set(ARCH_FLAGS "-Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else()
  set(ARCH_FLAGS "-march=native")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  flightlib
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Flightmare stereo renderer
add_executable(flightmare_renderer_node
  src/flightmare_renderer.cpp
  src/flightmare_renderer_node.cpp
)
target_link_libraries(flightmare_renderer_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)

# Odometry noise injection
add_executable(odom_noise_injector_node src/odom_noise_injector_node.cpp)
target_link_libraries(odom_noise_injector_node ${catkin_LIBRARIES})

# Synthetic YOLO training data generation
add_executable(data_generation
  src/data_generation/main.cpp
  src/data_generation/SensorPublisher.cpp
  src/data_generation/EnvironmentManager.cpp
  src/data_generation/TrajectoryGenerator.cpp
)
target_link_libraries(data_generation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs zmq zmqpp)

add_executable(camera_recorder src/data_generation/camera_recorder.cpp)
target_link_libraries(camera_recorder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)

catkin_install_python(PROGRAMS
  scripts/target_manager.py
  scripts/camera_tf_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS flightmare_renderer_node odom_noise_injector_node data_generation camera_recorder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
