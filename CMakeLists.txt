cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(hand_tracking)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED)

# Use glob to get the list of all source files.
file(GLOB_RECURSE SOURCES "src/*.cpp")
# We don't really need to include header and resource files to build
# file(GLOB_RECURSE HEADERS "include/*.hpp")
include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})
target_link_libraries(${CMAKE_PROJECT_NAME} ${PCL_LIBRARIES} ${BOOST_LIBRARIES})