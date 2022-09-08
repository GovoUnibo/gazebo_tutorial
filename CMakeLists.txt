cmake_minimum_required(VERSION 3.0.2)
project(gazebo_tutorials)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


include_directories(
  include
)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

# add_library(HelloWorld src/hello_world_plugin.cpp)
# target_link_libraries(HelloWorld ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(shapes src/shapes.cpp)
target_link_libraries(shapes
  ${GAZEBO_LIBRARIES}
)


add_library(cable src/WorldCablePlugin.cpp  include/WorldCablePlugin.hpp
                  src/sdf_builder.cpp       include/sdf_builder.hpp
                  src/sdf_sphere.cpp       include/sdf_sphere.hpp
                  )
target_link_libraries(cable ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)


