# This file is used to specify the build process of the package

# Project Declaration
cmake_minimum_required(VERSION 3.8)
project(localisation_aruco_marker)

# Compiler Options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # enable all warnings
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED) # loads 'ament_cmake' package - required for building ROS2 packages
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Executable Target
add_executable(localisation_aruco_marker src/localisation_aruco_marker.cpp) # Compile .cpp doc 
# to create and  produce executable localisation_aruco_marker
target_include_directories(localisation_aruco_marker PUBLIC # Include directories for the target
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(localisation_aruco_marker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Installation
install(TARGETS localisation_aruco_marker
  DESTINATION lib/${PROJECT_NAME}) # specify the destination for the executable

# If testing enabled:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # find package, set up linting
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE) # skip cpplint checks
  ament_lint_auto_find_test_dependencies()
endif()

# Export Package
ament_package()
