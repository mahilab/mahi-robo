# mahi::robo CMakeLists.txt
# Evan Pezent (epezent@rice.edu)
# Updated: 3/2020

cmake_minimum_required(VERSION 3.13.0)

#===============================================================================
# USER OPTIONS
#===============================================================================

option(MAHI_ROBO_EXAMPLES "Turn ON to build example executable(s)" ON)

#===============================================================================
# FRONT MATTER
#===============================================================================

# create the project
project(mahi-robo VERSION 1.0.0 LANGUAGES C CXX)

# tell user they can't have shared version if they try
if (BUILD_SHARED_LIBS)
    message(FATAL_ERROR "mahi::robo does not support shared libaries")
endif()

# add ./cmake to CMake module path so our .cmake files can be found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(GNUInstallDirs) # defines conventional GNU isntallation directories

# Enable IDE folders and set them for predefined CMake projects
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMake")

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3") # ALL WARNINGS
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /MP")    # MULTICORE BUILDS
endif()

#===============================================================================
# 3RD PARTY DEPENDENCIES
#===============================================================================

set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/Eigen3/include)

#===============================================================================
# MAHI UTIL
#===============================================================================

include(FetchContent) 
FetchContent_Declare(mahi-util GIT_REPOSITORY https://github.com/mahilab/mahi-util.git) 
FetchContent_MakeAvailable(mahi-util)

#===============================================================================
# CREATE LIBRARY
#===============================================================================

add_library(robo "")
add_library(mahi::robo ALIAS robo)
set_target_properties(robo PROPERTIES DEBUG_POSTFIX -d)
target_compile_features(robo PUBLIC cxx_std_11)
install(TARGETS robo EXPORT mahi-robo-targets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
set_target_properties(robo PROPERTIES OUTPUT_NAME "mahi-robo")

# defines
target_compile_definitions(robo PUBLIC MAHI_ROBO) # for compatibility checks

# add source files
add_subdirectory(src/mahi/robo)

# add include files
file(GLOB_RECURSE MAHI_ROBO_HEADERS "include/*.hpp" "include/*.h" "include/*.inl")
target_sources(robo PRIVATE ${MAHI_ROBO_HEADERS}) # for intellisense
target_include_directories(robo
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${EIGEN_INCLUDE_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# link libraries
target_link_libraries(robo PUBLIC mahi::util)

#===============================================================================
# WINDOWS ONLY
#===============================================================================

if(WIN32)

target_compile_definitions(robo
    PUBLIC
        -D_CRT_SECURE_NO_WARNINGS          # remove secure warnings (e.g sprinft_s)
        -DNOMINMAX                         # remove min/max macros
        -D_WINSOCK_DEPRECATED_NO_WARNINGS  # remove winsock deprecated warnings
) 

endif(WIN32)

#===============================================================================
# EXAMPLE EXECUTABLES
#===============================================================================

if(MAHI_ROBO_EXAMPLES)
    message("Building mahi::robo examples")
    add_subdirectory(examples)
endif()

#===============================================================================
# INSTALL
#===============================================================================

if (MSVC)
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(MAHI_INSTALL_POSTFIX "") # 64 bit Windows is default
    else()
        set(MAHI_INSTALL_POSTFIX "-x86")
    endif()
elseif(NI_LRT AND CMAKE_SYSTEM_PROCESSOR MATCHES ARM)
    set(MAHI_INSTALL_POSTFIX "-nilrt-arm")
elseif(NI_LRT AND CMAKE_SYSTEM_PROCESSOR MATCHES ARM)
    set(MAHI_INSTALL_POSTFIX "-nilrt-x64")
endif()

set(CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}${MAHI_INSTALL_POSTFIX}")

# install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY ${EIGEN_INCLUDE_DIR}/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# set where we want to install our config
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/mahi-robo)

# export the targets to a script
install(EXPORT mahi-robo-targets
	FILE
		mahi-robo-targets.cmake
	NAMESPACE
		mahi::
	DESTINATION
		${INSTALL_CONFIGDIR}
)

# include helper functions for creating config files that can be included by other projects to find and use a package
include(CMakePackageConfigHelpers)

# generate a package configuration file and a package version file
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/mahi-robo-config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/mahi-robo-config.cmake
	INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/mahi-robo-config-version.cmake
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY AnyNewerVersion
)

# install the config and configversion modules
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/mahi-robo-config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/mahi-robo-config-version.cmake
	DESTINATION ${INSTALL_CONFIGDIR}
)

# export from the build tree
export(EXPORT mahi-robo-targets
	   NAMESPACE mahi::
	   FILE ${CMAKE_CURRENT_BINARY_DIR}/mahi-robo-targets.cmake)