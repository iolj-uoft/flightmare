# Setup catkin simple
find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
  cv_bridge 
  image_transport 
  sensor_msgs
)

catkin_simple()

add_definitions(-std=c++17)

# Library and Executables
cs_add_library(${PROJECT_NAME} ${FLIGHTLIB_SOURCES})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${BLAS_LIBRARIES}
  ${LAPACK_LIBRARIES}
  ${LAPACKE_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
  zmq
  zmqpp
  stdc++fs
)

# Build tests
if(BUILD_TESTS)
  catkin_add_gtest(flightlib_tests ${FLIGHTLIB_TEST_SOURCES})
  target_link_libraries(flightlib_tests ${PROJECT_NAME} gtest gtest_main)
endif()

cs_add_executable(camera_example ~/Desktop/catkin_ws/src/flightmare/flightros/src/camera/camera.cpp)

# Link it against your flightlib library and ROS dependencies
target_link_libraries(camera_example 
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES}
)

# Finish catkin simple
cs_install()
cs_export()
