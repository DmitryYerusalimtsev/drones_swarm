cmake_minimum_required(VERSION 3.5)
project(drone)

find_package(ament_cmake REQUIRED)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

include(r2r_cargo.cmake)

# put ros package dependencies here.
r2r_cargo(
            std_srvs
            rcl
            rcl_action
            drone_msgs
         )

# install binaries
if(WIN32)
  set(SUFFIX ".exe")
else()
  set(SUFFIX "")
endif()

install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/colcon/drone${SUFFIX}
  ${CMAKE_SOURCE_DIR}/target/colcon/motor${SUFFIX}
  DESTINATION lib/${PROJECT_NAME}
)

# we need this for ros/colcon
ament_package()