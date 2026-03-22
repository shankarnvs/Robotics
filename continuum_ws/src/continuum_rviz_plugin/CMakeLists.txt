cmake_minimum_required(VERSION 3.8)
project(continuum_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)   # ✅ ADD THIS
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(continuum_msgs REQUIRED)   # ✅ ADD THIS


# Enable Qt MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include headers
include_directories(
  include
)

# ✅ CREATE TARGET FIRST
add_library(${PROJECT_NAME} SHARED
  src/continuum_panel.cpp
  include/continuum_rviz_plugin/continuum_panel.hpp
)

# ✅ THEN link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  pluginlib
  rclcpp
  continuum_msgs  
)

# ✅ LINK Qt ONCE (AFTER TARGET)
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# Export plugin
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install library
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin description
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
