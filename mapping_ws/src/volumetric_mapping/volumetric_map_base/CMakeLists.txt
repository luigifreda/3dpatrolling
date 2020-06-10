cmake_minimum_required(VERSION 2.8.3)
project(volumetric_map_base)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CUSTOM_C_FLAGS  "-O3 -DNDEBUG -march=native") 
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${CUSTOM_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_C_FLAGS}")
endif()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/world_base.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
