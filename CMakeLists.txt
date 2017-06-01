cmake_minimum_required(VERSION 2.8.3)
project(learning_egraphs2d)

add_definitions(-std=c++11)
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

find_package(catkin REQUIRED COMPONENTS
  sbpl_collision_checking
  sbpl_pr2_robot_model
  sbpl_kdl_robot_model
  smpl
)

find_package(sbpl REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES learning_egraphs2d
  CATKIN_DEPENDS sbpl_collision_checking sbpl_pr2_robot_model smpl
                 sbpl_kdl_robot_model
  DEPENDS sbpl
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${sbpl_INCLUDE_DIRS}
)

add_executable(gridworld
    src/gridworld.cpp
    src/utils.cpp
    src/grid_collision_checker_2d.cpp
    src/shape.cpp
    src/random_shape_generator.cpp
    )

target_link_libraries(gridworld
   ${catkin_LIBRARIES}
   ${sbpl_LIBRARIES}
 )

#############
## Testing ##
#############

add_executable( test_random_shape_generator
   test/test_random_shape_generator.cpp
   src/random_shape_generator.cpp
   src/utils.cpp
   src/grid_collision_checker_2d.cpp
   src/shape.cpp
   )

target_link_libraries( test_random_shape_generator
   ${catkin_LIBRARIES} )

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_learning_egraphs2d.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
