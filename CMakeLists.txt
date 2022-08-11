cmake_minimum_required(VERSION 3.0)
project(spherix)

set(INSTALL_PATH "usr/lib" CACHE STRING "Default installation path")
set(EIGEN3_INCLUDE_DIR "usr/eigen3" CACHE STRING "Eigen3 include directory")

# set(CMAKE_BUILD_TYPE Release)

file(GLOB sources_h "src/*.h")
file(GLOB sources_c "src/*.cc")
SET(LIBRARY_SRC ${sources_c} ${sources_h})

# Eigen3 Headers (note Eigen3 do not have cc files)

include_directories("${EIGEN3_INCLUDE_DIR}")

add_library(spherix_lib SHARED ${LIBRARY_SRC}) # Building SO/DLL
# add_library(spherix_lib STATIC ${LIBRARY_SRC}) # Building A/LIB
set_target_properties(spherix_lib PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(spherix_lib PUBLIC include)

install(TARGETS spherix_lib DESTINATION ${INSTALL_PATH})

set(CPACK_PACKAGE_NAME "spherix_lib")
set(CPACK_PACKAGE_VERSION "1.0.0")

set(CPACK_MONOLITHIC_INSTALL 1)
include(CPack)