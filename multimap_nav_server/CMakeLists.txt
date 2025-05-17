cmake_minimum_required(VERSION 3.8)
project(multimap_nav_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(multimap_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(SQLite3 REQUIRED)

add_library(move_base_action_client src/move_base_action_client.cpp)
ament_target_dependencies(move_base_action_client rclcpp_action behaviortree_cpp_v3 geometry_msgs nav2_msgs multimap_msgs)
target_include_directories(move_base_action_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# add_library(sql_query_maker src/sql_query_maker.cpp)
# ament_target_dependencies(sql_query_maker SQLite3 behaviortree_cpp_v3)
# target_include_directories(sql_query_maker PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# add_library(portal_service_client src/portal_service_client.cpp)
# add_dependencies(portal_service_client ${catkin_EXPORTED_TARGETS})
#
# add_library(portal_vicinity_checker src/portal_vicinity_checker.cpp)
# add_dependencies(portal_vicinity_checker ${catkin_EXPORTED_TARGETS})
#
# add_library(multimap_nav_action_server src/multimap_nav_action_server.cpp)
# add_dependencies(multimap_nav_action_server ${catkin_EXPORTED_TARGETS})
# target_link_libraries(multimap_nav_action_server
#     move_base_action_client
#     sql_query_maker
#     portal_service_client
#     portal_vicinity_checker
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES})
#
# add_executable(multimap_nav_server src/main.cpp)
# add_dependencies(multimap_nav_server ${MULTIMAP_NAV_LIBS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(multimap_nav_server
#     multimap_nav_action_server
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES})

# target_include_directories(my_executable PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
