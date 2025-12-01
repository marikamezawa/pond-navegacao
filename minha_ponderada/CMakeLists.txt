cmake_minimum_required(VERSION 3.8)
project(minha_ponderada)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cg_interfaces REQUIRED)

include_directories(include)

# UM ÚNICO executável com ambas as partes
add_executable(main_node
    src/main.cpp
    src/map_loader.cpp
    src/pathfinder.cpp
    src/robot_mover.cpp
    src/mapper.cpp
)

ament_target_dependencies(main_node
    rclcpp
    std_msgs
    cg_interfaces
)

install(TARGETS
  main_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()