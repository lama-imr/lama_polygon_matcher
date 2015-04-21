cmake_minimum_required(VERSION 2.8.3)
project(csm_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

# From original csm source, to generate config.h.
subdirs(csm/sm/lib/json-c)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS csm/sm csm/sm/lib
  LIBRARIES csm
  #  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS libgsl
)

###########
## Build ##
###########

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  csm/sm
  csm/sm/lib
  )

## Declare a cpp library
# Adapted from csm/sm/CMakeLists.txt in the original sources.
set(csm_sources 
  csm/sm/csm/clustering.c
  csm/sm/csm/gpm/gpm.c
  csm/sm/csm/icp/icp.c
  csm/sm/csm/icp/icp_corr_dumb.c
  csm/sm/csm/icp/icp_corr_tricks.c
  csm/sm/csm/icp/icp_covariance.c
  csm/sm/csm/icp/icp_loop.c
  csm/sm/csm/icp/icp_outliers.c
  csm/sm/csm/icp/icp_debug.c
  csm/sm/csm/mbicp/mbicp_interface.c      
  csm/sm/csm/mbicp/MbICP.c      
  csm/sm/csm/mbicp/calcul.c     
  csm/sm/csm/mbicp/percolate.c  
  csm/sm/csm/mbicp/sp_matrix.c
  csm/sm/csm/hsm/hsm.c
  csm/sm/csm/hsm/hsm_interface.c
  csm/sm/csm/json_journal.c
  csm/sm/csm/laser_data.c
  csm/sm/csm/laser_data_carmen.c
  csm/sm/csm/laser_data_fisher.c
  csm/sm/csm/laser_data_json.c
  csm/sm/csm/laser_data_load.c
  csm/sm/csm/laser_data_drawing.c
  csm/sm/csm/laser_data_bbox.c
  csm/sm/csm/logging.c
  csm/sm/csm/math_utils.c
  csm/sm/csm/math_utils_gsl.c
  csm/sm/csm/orientation.c
  csm/sm/csm/sm_options.c
  csm/sm/csm/utils.c
  csm/sm/lib/egsl/egsl.c
  csm/sm/lib/egsl/egsl_conversions.c
  csm/sm/lib/egsl/egsl_misc.c
  csm/sm/lib/egsl/egsl_ops.c
  csm/sm/lib/gpc/gpc.c
  csm/sm/lib/gpc/gpc_utils.c
  csm/sm/lib/json-c/arraylist.c
  csm/sm/lib/json-c/debug.c
  csm/sm/lib/json-c/JSON_checker.c
  csm/sm/lib/json-c/json_more_utils.c
  csm/sm/lib/json-c/json_object.c
  csm/sm/lib/json-c/json_tokener.c
  csm/sm/lib/json-c/json_util.c
  csm/sm/lib/json-c/linkhash.c
  csm/sm/lib/json-c/printbuf.c
  csm/sm/lib/options/options.c
  csm/sm/lib/options/options_interface.c
)
add_library(csm ${csm_sources})
target_link_libraries(csm ${gsl_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS csm
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY csm/sm
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY csm/sm/lib
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_csm_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
