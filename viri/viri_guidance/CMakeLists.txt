cmake_minimum_required(VERSION 3.0.2)
project(viri_guidance)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  std_srvs
  quadrotor_msgs
  viri_msgs
  viri_estimation
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
)

# ROS-free guidance laws
add_library(${PROJECT_NAME}
  src/attitude_mapping.cpp
  src/pro_nav.cpp
  src/hybrid_pro_nav.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS})

# Node
add_executable(guidance_node src/guidance_node.cpp)
target_include_directories(guidance_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(guidance_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(guidance_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_guidance test/test_guidance.cpp)
  target_link_libraries(test_guidance ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME} guidance_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
