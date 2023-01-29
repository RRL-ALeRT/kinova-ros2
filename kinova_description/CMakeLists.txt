cmake_minimum_required(VERSION 3.5)
project(kinova_description)

set(CMAKE_CXX_STANDARD 14)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes
   DESTINATION share/${PROJECT_NAME}
  #  FILES_MATCHING PATTERN "*.dae"
 )

ament_package()
