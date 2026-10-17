cmake_minimum_required(VERSION 3.0.2)
project(viri_estimation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  viri_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

# ROS-free core library
add_library(${PROJECT_NAME}
  src/frame_utils.cpp
  src/relative_ltvkf.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS})

# Node
add_executable(relative_ltvkf_node src/relative_ltvkf_node.cpp)
target_include_directories(relative_ltvkf_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(relative_ltvkf_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(relative_ltvkf_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_relative_ltvkf test/test_relative_ltvkf.cpp)
  target_link_libraries(test_relative_ltvkf ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME} relative_ltvkf_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
