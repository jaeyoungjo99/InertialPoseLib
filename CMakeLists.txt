cmake_minimum_required(VERSION 2.8.4)
# Specifies the minimum version of CMake required to build this project.

project(InertialPoseLib)
# Defines the name of the project.

# Version information
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
if (PROJECT_VERSION_PATCH GREATER 0)
  set(PROJECT_VERSION "${PROJECT_VERSION}.${PROJECT_VERSION_PATCH}")
endif()
# Sets the version information for the project. The version is composed of major, minor, and patch numbers.

# The library version tracks the numbering given by libtool in the
# autoconf set up.
set(LIBVERSION_API 1)
set(LIBVERSION_BUILD 1.0.0)
# Sets the library version information, which may be used for shared library versioning.

string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
# Converts the project name to lowercase and uppercase strings for potential use in the configuration.

# Set the default "real" precision
set(INERTIALPOSELIB_PRECISION 2 CACHE STRING
  "Precision: 1 = float, 2 = double, 3 = extended, 4 = quadruple, 5 = variable")
set_property(CACHE INERTIALPOSELIB_PRECISION PROPERTY STRINGS 1 2 3 4 5)
# Defines a cache variable for the precision setting of the library, with a default value of 2 (double precision).

set(LIBNAME InertialPose)
# Sets the library name.

include_directories(
  ./include/
)
# Specifies the directories to be included during the compilation process.

add_library(libInertialPose src/LinearAlgebra.cpp
                            src/PoseEstimation.cpp)
# Adds a library target named libInertialPose, composed of the specified source files.