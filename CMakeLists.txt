cmake_minimum_required(VERSION 3.5.1)
project(colpa)
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

#================================================================================
# Compiler settings
#================================================================================
# Enforce the use of C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wuninitialized -Wpedantic")

add_compile_options(-Wall -Wextra -Wpedantic)

#================================================================================
# Link Libraries
#================================================================================

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/datastructures/Graph.cpp
  src/datastructures/Queue.cpp
  src/datastructures/State.cpp
  src/event/Event.cpp
  src/event/ShortestPathEvent.cpp
  src/event/ConstantDepthEvent.cpp
  src/sampler/HaltonSequence.cpp
  src/COLPA.cpp)
  #src/LCOLPA.cpp)

#================================================================================
# Example Scripts
#================================================================================

# Correctness Test Script
add_executable(test_COLPA examples/test_COLPA.cpp)
target_link_libraries(test_COLPA ${PROJECT_NAME} ${OpenCV_LIBS})
install(TARGETS test_COLPA RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(test_LCOLPA examples/test_LCOLPA.cpp)
#target_link_libraries(test_LCOLPA ${PROJECT_NAME} ${OpenCV_LIBS})
#install(TARGETS test_LCOLPA RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(test_inc examples/test_inc.cpp)
#target_link_libraries(test_inc ${PROJECT_NAME} ${OpenCV_LIBS})
#install(TARGETS test_inc RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#================================================================================
# Installation
#================================================================================

install(TARGETS colpa EXPORT
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/ DESTINATION include)

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/colpa")

#================================================================================
# Formatting
#================================================================================
include(ClangFormat)

clang_format_setup(VERSION 3.8)

if (CLANG_FORMAT_EXECUTABLE)
  FILE(GLOB_RECURSE colpa_headers "include/*.hpp")
  FILE(GLOB_RECURSE colpa_srcs "include/*.cpp")
  clang_format_add_sources(${colpa_headers})
  clang_format_add_sources(${colpa_srcs})

  clang_format_add_targets()
endif()

#================================================================================
# IDE Comfort
#================================================================================

# This custom target doesn't affect building the project but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE LibFiles "include/*.hpp")
add_custom_target(headers SOURCES ${LibFiles})
