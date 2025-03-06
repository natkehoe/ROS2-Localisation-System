# This file is used to specify the build process of the package

#----- PROJECT DECLARATION -----#
cmake_minimum_required(VERSION 3.8)
project(localisation_aruco_marker)

#----- COMPILER OPTIONS -----#
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # enable all warnings
endif()

#----- FIND DEPENDENCIES -----#
# This is required for building the package. i.e. if we are publishing message 
#   of type 'std_msgs/String', we need to make sure we declare a dependency on 
#   'std_msgs' on ROS2 package.
find_package(ament_cmake REQUIRED) # loads 'ament_cmake' package - required for building ROS2 packages
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(roscpp REQUIRED)
find_package(rospy REQUIRED)
find_package(std_msgs REQUIRED)

# Define a CMake variable named dependencies, that lists all ROS2 packages and other dependencies 
#   the project requires
set(dependencies
  rclcpp
  std_msgs
)


## Include specified directories to list of paths the compiler uses to search for header files.
## Important for C++ projects where you have custom header files.
# include_directories() # already done? target_include_directories()

#----- EXECUTABLE TARGET -----#
add_executable(localisation_aruco_marker src/localisation_aruco_marker.cpp) # Compile .cpp doc 
# to create and  produce executable localisation_aruco_marker
target_include_directories(localisation_aruco_marker PUBLIC # Include directories for the target
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(localisation_aruco_marker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#----- INSTALLATION -----#
# Install buile CPP programs to project install directory
install(TARGETS localisation_aruco_marker
  DESTINATION lib/${PROJECT_NAME}) # specify the destination for the executable

install(
  DIRECTORY include/
  DESTINATION include
)

# Install cpp executables
#install(
#  TARGETS
#  minimal_cpp_publisher
#  minimal_cpp_subscriber
#  DESTINATION lib/${PROJECT_NAME}
#)

# install python modules for import
ament_python_install_package(${PROJECT_NAME})

# Add this section to install Python scripts
install(
  PROGRAMS
  ROS2-Localisation-System/localisation_aruco_marker.py
  DESTINATION lib/${PROJECT_NAME}
)


#----- TESTING -----#
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

#----- EXPORT PACKAGE -----#
# Export include directories of package so that they can be easily included
# by other packages that depend on this package
ament_export_include_directories(include)

# Generate and install necessary CMake and environment hooks that allow
# other packages to find and use package
ament_package()
