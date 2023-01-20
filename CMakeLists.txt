cmake_minimum_required(VERSION 3.8)
project(haruto_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(include)

add_executable(startup src/startup.cpp src/mecanum.cpp src/differential.cpp)

target_link_libraries (startup Eigen3::Eigen)

install(TARGETS
startup
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
