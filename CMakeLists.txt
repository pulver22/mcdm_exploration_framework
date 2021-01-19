cmake_minimum_required(VERSION 2.8.3)
project(next_best_sense)

find_package(catkin REQUIRED COMPONENTS
  # mfc: we will record using stats_pub
  #record_ros
  actionlib
  nav_msgs
  geometry_msgs
  roscpp
  tf
  costmap_2d
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_cv
  cv_bridge
  move_base_msgs
  rfid_node
  rfid_grid_map
  strands_navigation_msgs
  topological_navigation
  bayesian_topological_localisation

)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES next_best_sense
  CATKIN_DEPENDS move_base_msgs actionlib nav_msgs roscpp geometry_msgs 
  # mfc: we will record using stats_pub
  # record_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(next_best_sense pose.cpp map.cpp newray.cpp mcdmfunction.cpp
map.cpp evaluationrecords.cpp Criteria/sensingtimecriterion.cpp
Criteria/informationgaincriterion.cpp Criteria/mcdmweightreader.cpp Criteria/traveldistancecriterion.cpp
Criteria/batterystatuscriterion.cpp Criteria/RFIDCriterion.cpp
Criteria/weightmatrix.cpp Criteria/criterioncomparator.cpp Criteria/criterion.cpp
PathFinding/astar.cpp PathFinding/node.cpp RFIDGridmap.cpp utils.cpp)

#add_executable(mcdm_online_exploration_ros main.cpp )
#target_link_libraries(mcdm_online_exploration_ros next_best_sense ${catkin_LIBRARIES})

add_executable(pure_navigation pure_navigation.cpp)
target_link_libraries(pure_navigation next_best_sense ${catkin_LIBRARIES})

add_executable(pure_navigation_two_readers pure_navigation_two_readers.cpp)
target_link_libraries(pure_navigation_two_readers next_best_sense ${catkin_LIBRARIES})

#add_executable(mcdm_slim mcdm_slim.cpp)
#target_link_libraries(mcdm_slim next_best_sense ${catkin_LIBRARIES})

# add_executable(random_walk random_walk.cpp)
# target_link_libraries(random_walk next_best_sense ${catkin_LIBRARIES})

# add_executable(random_frontier random_frontier.cpp)
# target_link_libraries(random_frontier next_best_sense ${catkin_LIBRARIES})
## Declare a C++ executable
add_executable(
   path_len_node
   path_len.cpp
   )

 target_link_libraries(
   path_len_node
   ${catkin_LIBRARIES}
 )

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -g  -ftest-coverage -fprofile-arcs")

## Declare a cpp executable
# add_executable(next_best_sense_node src/next_best_sense_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(next_best_sense_node next_best_sense_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(next_best_sense_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS next_best_sense next_best_sense_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_next_best_sense.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
