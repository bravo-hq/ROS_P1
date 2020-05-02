cmake_minimum_required(VERSION 2.8.3)
project(lla2enu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geodesy sensor_msgs tf nav_msgs geometry_msgs 
                  message_generation dynamic_reconfigure )

add_message_files(
 FILES
 dist.msg
 )
 
add_service_files(
  FILES
  calculation.srv
)

generate_messages(
   DEPENDENCIES
  std_msgs
  geometry_msgs
 )

 
 generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
  
)





## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sub_front src/sub_front.cpp)
target_link_libraries(sub_front ${catkin_LIBRARIES})

add_executable(TF src/TF.cpp)
target_link_libraries(TF ${catkin_LIBRARIES})

add_executable(Service_client src/Service_client.cpp)
add_dependencies(Service_client ${PROJECT_NAME}_gencfg)
target_link_libraries(Service_client ${catkin_LIBRARIES})

add_executable(Service_Server src/Service_Server.cpp)
target_link_libraries(Service_Server ${catkin_LIBRARIES})
add_dependencies(Service_Server ${catkin_EXPORTED_TARGETS})

