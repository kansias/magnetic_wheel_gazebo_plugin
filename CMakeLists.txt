cmake_minimum_required(VERSION 3.8)
project(magnetic_wheel_gazebo_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
add_library(${PROJECT_NAME} SHARED
  src/MagneticWheelPlugin.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  gazebo_dev
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(TARGETS ${PROJECT_NAME} # TARGETS is the target name of the library or executable to install
  ARCHIVE DESTINATION lib # ARCHIVE is the destination for static libraries ex) lib/libmy_lib.a
  LIBRARY DESTINATION share/${PROJECT_NAME} # LIBRARY is the destination for shared libraries ex) share/my_project/my_lib.so
  RUNTIME DESTINATION bin # RUNTIME is the destination for executables ex) bin/my_executable
  
)

  

ament_package()
