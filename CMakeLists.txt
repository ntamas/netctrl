cmake_minimum_required(VERSION 2.8.12)
cmake_policy(SET CMP0003 NEW)

#####################################################################
# General project properties
#####################################################################

project(netctrl C CXX)

#####################################################################
# Build options
#####################################################################

# Whether we prefer static to shared libraries
option(PREFER_STATIC_LIBRARIES
       "Decides whether we prefer static libraries to shared ones when they both exist"
	   OFF)

#####################################################################
# Version information
#####################################################################

set(NETCTRL_VERSION_MAJOR 0)
set(NETCTRL_VERSION_MINOR 2)
set(NETCTRL_VERSION_PATCH 0)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/netctrl/version.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/include/netctrl/version.h)

#####################################################################
# Tweaking CMake
#####################################################################

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/etc/cmake;${CMAKE_CURRENT_SOURCE_DIR}/vendor/igraphpp/etc/cmake;${CMAKE_MODULE_PATH})
if(PREFER_STATIC_LIBRARIES)
	set(CMAKE_FIND_LIBRARY_SUFFIXES .a;.lib;${CMAKE_FIND_LIBRARY_SUFFIXES})
	set(CMAKE_C_FLAGS "-static-libgcc ${CMAKE_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "-static-libgcc ${CMAKE_CXX_FLAGS}")
endif(PREFER_STATIC_LIBRARIES)

#####################################################################
# Find dependencies
#####################################################################

find_package(igraph REQUIRED)

if(PREFER_STATIC_LIBRARIES)
	find_package(ZLIB REQUIRED)
	find_package(LibXml2 REQUIRED)
	set(igraph_LIBRARIES ${igraph_LIBRARIES} ${LIBXML2_LIBRARIES} ${ZLIB_LIBRARIES})
endif(PREFER_STATIC_LIBRARIES)

#####################################################################
# Compiler flags for different build configurations
#####################################################################

set(CMAKE_C_FLAGS   "${CMAKE_ARCH_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_ARCH_FLAGS} -Wall")
set(CMAKE_C_FLAGS_DEBUG   "${CMAKE_ARCH_FLAGS} -O0 -g")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_ARCH_FLAGS} -O0 -g")
set(CMAKE_C_FLAGS_RELEASE   "${CMAKE_ARCH_FLAGS} -O3 -DNDEBUG -Wall -s")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_ARCH_FLAGS} -O3 -DNDEBUG -Wall -s")
set(CMAKE_C_FLAGS_PROFILING "${CMAKE_ARCH_FLAGS} -pg")
set(CMAKE_CXX_FLAGS_PROFILING "${CMAKE_ARCH_FLAGS} -pg")

include_directories(${igraph_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_BINARY_DIR}/include
					${CMAKE_CURRENT_SOURCE_DIR}/vendor/igraphpp/include)

#####################################################################
# Set up CPack
#####################################################################

set(CPACK_PACKAGE_VERSION_MAJOR ${NETCTRL_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${NETCTRL_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${NETCTRL_VERSION_PATCH})
set(CPACK_GENERATOR "TGZ")
set(CPACK_STRIP_FILES "bin/netctrl")
include(CPack)

install(FILES README.rst
        PERMISSIONS OWNER_READ OWNER_WRITE
                    GROUP_READ WORLD_READ
        DESTINATION .)

#####################################################################
# Process subdirectories
#####################################################################

add_subdirectory(src)
add_subdirectory(vendor/igraphpp/src)
