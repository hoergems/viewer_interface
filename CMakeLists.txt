cmake_minimum_required (VERSION 2.6)

project (viewer_interface)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -w")
set(CMAKE_CXX_FLAGS_RELEASE "-g -std=c++11 -O3 -w")

#SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${viewer_interface_SOURCE_DIR})
#SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${viewer_interface_SOURCE_DIR}/../../../python/)
SET(BOOST_MIN_VERSION "1.54.0")
SET(PYTHON_MIN_VERSION "2.7.0")
include(FindPkgConfig)

####### USER SPECIFIED VARIABLES #########
set(USE_URDF True)
##########################################

##############################
#Try to load URDF
##############################
if (USE_URDF)       
    find_package(urdf REQUIRED)
    if (urdf_FOUND)
        message("-- Found URDF package: ${urdf_INCLUDE_DIRS}")
        set(VIEWER_SRC "viewer_interface.cpp")
        set(URDF_SRC "urdf_loader.cpp")
        add_definitions(-DUSE_URDF)              
    endif()
endif()
##############################

############################# LOAD PYTHON #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(PYTHON python-2.7)
    if(NOT PYTHON_FOUND)
       message(FATAL_ERROR "Python could not be found")
    endif()
    include_directories(${PYTHON_INCLUDE_DIRS})    
endif()


############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS python system thread timer)
if (NOT Boost_FOUND)
    set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIRS}")
    IF ( NOT Boost_INCLUDE_DIRS )
        message(FATAL_ERROR "Please point the environment variable BOOST_INCLUDE_DIRS to the include directory of Boost")
    ENDIF()
endif (NOT Boost_FOUND)
include_directories(${Boost_INCLUDE_DIRS})

############################# LOAD OPENRAVE #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(OPENRAVE openrave0.9-core)
    if(NOT OPENRAVE_FOUND)
       message(FATAL_ERROR "OpenRAVE 0.9 could not be found")
    endif()
    include_directories(${OPENRAVE_INCLUDE_DIRS})    
endif()

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
link_directories(${OPENRAVE_LIBRARY_DIRS}
                 ${PYTHON_LIBRARY_DIRS}                 
                 ${Boost_LIBRARY_DIRS}                 
                 ${URDF_LIBRARY_DIRS})

add_library(viewer_interface SHARED
            viewer_interface.cpp
            viewer.cpp
            urdf_loader.cpp)

target_link_libraries (viewer_interface
                       ${urdf_LIBRARIES}
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}
                       ${OPENRAVE_LIBRARIES})

message("WHAT ${CMAKE_INSTALL_INCLUDEDIR}")

set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/viewer_interface.pc.in")
set(pkg_conf_file_out "${CMAKE_CURRENT_BINARY_DIR}/viewer_interface.pc")
configure_file("${pkg_conf_file_in}" "${pkg_conf_file_out}" @ONLY)

message ("WHA ${CMAKE_INSTALL_PREFIX}" )
install(TARGETS viewer_interface DESTINATION lib)
install(FILES urdf_loader.hpp viewer_interface.hpp viewer.hpp DESTINATION include/viewer_interface)
install(FILES "${pkg_conf_file_out}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)


