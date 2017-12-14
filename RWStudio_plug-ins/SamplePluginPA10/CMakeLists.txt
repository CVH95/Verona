#####################################################
# Template for building RobWork dependent projects
# - the src should contain code for putting into libraries 
# - the plugins contain code for multiple RobWorkStudio plugins
# - the test for testing 
# - the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this
# file for adding additional dependencies to external projects  
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project.
PROJECT(RoVi1PluginPA10)

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Set the level of verbosity when running make
set(CMAKE_VERBOSE_MAKEFILE False)

# Now set the RW/RWS root (edit this if necessary)
SET(RW_ROOT $ENV{RW_ROOT})
SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})

# Set build type to release
IF( NOT CMAKE_BUILD_TYPE )
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
MESSAGE("-- RobWork root: " ${RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake) 
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Use RobWorkStudio
MESSAGE("-- RobWorkStudio root: " ${RWSTUDIO_ROOT})
SET(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
FIND_PACKAGE(RobWorkStudio REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

# Use OpenCV 3
find_package(OpenCV 3 REQUIRED)

# Set the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)

# If we want to use ui files add them here
SET(UIS_FILES src/SamplePlugin.ui )
QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})

#Need to add the current binary dir to the include directory because the generated source files are placed here
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# MOC
QT5_WRAP_CPP(MocSrcFiles src/SamplePlugin.hpp)

# Rcc the files:
QT5_ADD_RESOURCES(RccSrcFiles src/resources.qrc)

# Add the source files to build
SET(SrcFiles src/SamplePlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(${PROJECT_NAME} MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${OpenCV_LIBS})

