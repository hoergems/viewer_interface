cmake_minimum_required (VERSION 2.6)

project (viewer_interface)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -w")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -w")
SET(BOOST_MIN_VERSION "1.54.0")
SET(PYTHON_MIN_VERSION "2.7.0")
include(FindPkgConfig)

############################# LOAD URDF #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(URDF urdfdom)
    if(NOT URDF_FOUND)
       message(FATAL_ERROR "URDF package could not be found")
    endif()
    include_directories(${URDF_INCLUDE_DIRS})
    link_directories(${URDF_LIBRARY_DIRS})
    
    pkg_check_modules(URDFHEADERS urdfdom_headers) 
    if(NOT URDFHEADERS_FOUND)
       message(FATAL_ERROR "URDFHEADERS package could not be found")
    endif() 
    include_directories(${URDFHEADERS_INCLUDE_DIRS})
endif()

############################# LOAD PYTHON #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(PYTHON python-2.7)
    if(NOT PYTHON_FOUND)
       message(FATAL_ERROR "Python could not be found")
    endif()
    include_directories(${PYTHON_INCLUDE_DIRS})
    link_directories(${PYTHON_LIBRARY_DIRS})    
endif()


############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS python system thread timer)
if (NOT Boost_FOUND)
    set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIRS}")
    IF ( NOT Boost_INCLUDE_DIRS )
        message(FATAL_ERROR "Please point the environment variable BOOST_INCLUDE_DIRS to the include directory of Boost")
    ENDIF()
endif ()
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

############################# LOAD OPENRAVE #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(OPENRAVE openrave0.9-core)
    if(NOT OPENRAVE_FOUND)
       message(FATAL_ERROR "OpenRAVE 0.9 could not be found")
    endif()
    include_directories(${OPENRAVE_INCLUDE_DIRS})
    link_directories(${OPENRAVE_LIBRARY_DIRS})    
endif()

############################# LOAD EIGEN #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN eigen3)
    if(NOT EIGEN_FOUND)
       message(FATAL_ERROR "EIGEN could not be found")
    endif()
    include_directories(${EIGEN_INCLUDE_DIRS}) 
    message("Eigen include dirs: ${EIGEN_INCLUDE_DIRS}")
    message("Eigen library dirs: ${EIGEN_LIBRARY_DIRS}")   
endif()

add_library(viewer_interface SHARED
            viewer_interface.cpp
            viewer.cpp
            urdf_loader.cpp)

target_link_libraries (viewer_interface
                       ${URDF_LIBRARIES}
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}
                       ${OPENRAVE_LIBRARIES})


set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/viewer_interface.pc.in")
set(pkg_conf_file_out "${CMAKE_CURRENT_BINARY_DIR}/viewer_interface.pc")
configure_file("${pkg_conf_file_in}" "${pkg_conf_file_out}" @ONLY)

install(TARGETS viewer_interface DESTINATION lib)
install(FILES urdf_loader.hpp viewer_interface.hpp viewer.hpp DESTINATION include/viewer_interface)
install(FILES "${pkg_conf_file_out}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)


