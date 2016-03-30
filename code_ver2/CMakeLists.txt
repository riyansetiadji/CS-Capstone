cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(HAND_OBJ_TRACKER)
find_package(PCL 1.7 REQUIRED)
set(SUBSYS_NAME hand_obj_tracker)
#############################
set(SRCS 
    src/kinectTest.cpp
    src/skinFilter.cpp
    src/objTracker.cpp
)

set(INCS 
    include/pcl/apps/${SUBSYS_NAME}/skinFilter.h
    include/pcl/apps/${SUBSYS_NAME}/objTracker.h
)

set(IMPL_INCS 
    include/pcl/apps/${SUBSYS_NAME}/impl/skinFilter.hpp
    include/pcl/apps/${SUBSYS_NAME}/impl/objTracker.hpp
)

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(EXE_NAME pcl_${SUBSYS_NAME})

add_executable(${EXE_NAME} ${SRCS} ${INCS} ${IMPL_INCS})
target_link_libraries(${EXE_NAME} ${PCL_LIBRARIES})



