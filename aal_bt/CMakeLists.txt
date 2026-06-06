cmake_minimum_required(VERSION 3.14.4)
project(aal_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(aal_msgs REQUIRED)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_action
    behaviortree_cpp
    aal_msgs)

include_directories(include)

add_library(aal_bt_nodes SHARED
    src/ensure_configuration.cpp
    src/register_nodes.cpp
)
target_compile_definitions(aal_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(aal_bt_nodes ${THIS_PACKAGE_DEPS})
target_include_directories(aal_bt_nodes PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS aal_bt_nodes
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS aal_bt_nodes
    LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
    ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
    RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(DIRECTORY include/
    DESTINATION include
)

install(FILES bts/groot_palette.xml
    DESTINATION share/${PROJECT_NAME}/bts
)

add_executable(export_node_models tools/export_node_models.cpp)
ament_target_dependencies(export_node_models behaviortree_cpp)
target_link_libraries(export_node_models aal_bt_nodes)
target_include_directories(export_node_models PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(TARGETS export_node_models
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ensure_configuration
      test/test_ensure_configuration.cpp)
  ament_target_dependencies(test_ensure_configuration
      rclcpp
      rclcpp_action
      behaviortree_cpp
      aal_msgs)
  target_link_libraries(test_ensure_configuration aal_bt_nodes)
  target_include_directories(test_ensure_configuration PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_export_include_directories(include)
ament_package()
