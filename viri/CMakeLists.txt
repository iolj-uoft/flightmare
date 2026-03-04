cmake_minimum_required(VERSION 3.0.0)

project(viri)

find_package(catkin_simple REQUIRED)

find_package(OpenCV REQUIRED)

catkin_simple()

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
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
# otherwise double free or corruption (out) error when running racing or motion_planning example
add_compile_options(-O3)

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g")

#pilot

cs_add_library(flight_pilot
    src/pilot/flight_pilot.cpp
)

target_link_libraries(flight_pilot
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  stdc++fs
)

cs_add_executable(flight_pilot_node
   src/pilot/flight_pilot_node.cpp
)

target_link_libraries(flight_pilot_node
  flight_pilot
  ${OpenCV_LIBRARIES}
  stdc++fs
)

# racing
catkin_package(
LIBRARIES
CATKIN_DEPENDS
)

cs_add_executable(racing
    src/racing/racing.cpp
)

target_link_libraries(racing
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
stdc++fs
zmq
zmqpp
)

# camera
cs_add_executable(data_generation
    src/data_generation/main.cpp
    src/data_generation/SensorPublisher.cpp
    src/data_generation/EnvironmentManager.cpp
    src/data_generation/TrajectoryGenerator.cpp
)

target_link_libraries(data_generation
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
stdc++fs
zmq
zmqpp
)

# camera recorder
cs_add_executable(camera_recorder 
    src/data_generation/camera_recorder.cpp)
    
# interceptor

cs_add_library(interceptor
    src/interceptor/interceptor.cpp
)

target_link_libraries(interceptor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  stdc++fs
)

cs_add_executable(interceptor_node
   src/interceptor/interceptor_node.cpp
)

target_link_libraries(interceptor_node
  interceptor
  ${OpenCV_LIBRARIES}
  stdc++fs
)


# Finish
cs_install()
cs_export()
