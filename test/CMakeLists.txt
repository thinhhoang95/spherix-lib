cmake_minimum_required(VERSION 3.0)

project(spherix)



set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3" CACHE STRING "Eigen3 include directory")

set(SPHERIX_LIB_DIR "/usr/lib" CACHE STRING "Spherix library search directory")

set(SPHERIX_HEADERS_DIR ${CMAKE_SOURCE_DIR}/include CACHE STRING "Spherix library headers")



# set(CMAKE_BUILD_TYPE Release)



file(GLOB sources_h "*.h")

file(GLOB sources_c "*.cpp")

SET(TARGET_SRC ${sources_c} ${sources_h})



# Eigen3 Headers (note Eigen3 do not have cc files)

include_directories("${EIGEN3_INCLUDE_DIR}") #eigen3



add_library(spherix_lib SHARED IMPORTED)

set_target_properties(spherix_lib PROPERTIES IMPORTED_LOCATION ${SPHERIX_LIB_DIR}/libspherix_lib.so) # Linux
# set_target_properties(spherix_lib PROPERTIES IMPORTED_IMPLIB "${SPHERIX_LIB_DIR}/spherix_lib.lib") # Windows



ADD_EXECUTABLE(TestSampleTraj ${TARGET_SRC})



target_link_libraries(TestSampleTraj spherix_lib)

target_include_directories(TestSampleTraj PUBLIC ${CMAKE_SOURCE_DIR}/../include) #spherix_lib headers