cmake_minimum_required(VERSION 3.0.2)
project(viri_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs vision_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

# ROS-free core library
add_library(${PROJECT_NAME} src/stereo_triangulator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS})

# Node
add_executable(depth_estimation_node src/depth_estimation_node.cpp)
target_include_directories(depth_estimation_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(depth_estimation_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(depth_estimation_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_stereo_triangulator test/test_stereo_triangulator.cpp)
  target_link_libraries(test_stereo_triangulator ${PROJECT_NAME})
endif()

catkin_install_python(PROGRAMS scripts/yolo_stereo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME} depth_estimation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
