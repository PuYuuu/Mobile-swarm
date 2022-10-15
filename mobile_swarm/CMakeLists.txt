cmake_minimum_required(VERSION 3.0.2)
project(mobile_swarm)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")

# ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  nav_msgs
  std_msgs
  cv_bridge
  rosconsole
  sensor_msgs
  geometry_msgs
)
include_directories( ${catkin_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# source
include_directories(${PROJECT_SOURCE_DIR}/include)

catkin_package()

add_executable(robo_serial src/robo_serial.cpp)
target_link_libraries(robo_serial ${catkin_LIBRARIES})

add_executable(obs_detection src/obs_detection.cpp)
target_link_libraries(obs_detection ${catkin_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(mobile_swarm 
  src/mobile_swarm.cpp
  src/agent.cpp
  src/visualize.cpp)
target_link_libraries(mobile_swarm 
  ${catkin_LIBRARIES} ${Pangolin_LIBRARIES}  ${OpenCV_LIBS})

  