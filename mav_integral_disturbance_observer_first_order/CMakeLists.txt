cmake_minimum_required(VERSION 2.8.3)
project(mav_integral_disturbance_observer_first_order)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(Integral_DO_first_orderLibrary
  src/Integral_disturbance_observer_first_order.cpp
)

############
# BINARIES #
############

#No binaries, only libraries in this package!

##########
# EXPORT #
##########
cs_install()
cs_export()
