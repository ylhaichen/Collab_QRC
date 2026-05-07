cmake_minimum_required(VERSION 3.16)
project(team_pose_graph_optimizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

get_filename_component(_repo_root "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(_local_gtsam_prefix "${_repo_root}/.local_deps/gtsam_humble/extract/opt/ros/humble")
if(EXISTS "${_local_gtsam_prefix}/lib/cmake/GTSAM/GTSAMConfig.cmake")
  list(PREPEND CMAKE_PREFIX_PATH "${_local_gtsam_prefix}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(GTSAM QUIET CONFIG)

add_executable(team_pose_graph_optimizer_node src/team_pose_graph_optimizer_node.cpp)
ament_target_dependencies(team_pose_graph_optimizer_node rclcpp std_msgs)

if(GTSAM_FOUND AND TARGET gtsam)
  target_compile_definitions(team_pose_graph_optimizer_node PRIVATE HAVE_GTSAM_CPP=1)
  target_link_options(team_pose_graph_optimizer_node PRIVATE "-Wl,--no-as-needed")
  target_link_libraries(team_pose_graph_optimizer_node gtsam)
  if(TARGET metis-gtsam)
    target_link_libraries(team_pose_graph_optimizer_node metis-gtsam)
  endif()
  get_target_property(_gtsam_location gtsam IMPORTED_LOCATION_NONE)
  if(_gtsam_location)
    get_filename_component(_gtsam_lib_dir "${_gtsam_location}" DIRECTORY)
    set_target_properties(team_pose_graph_optimizer_node PROPERTIES
      BUILD_RPATH "${_gtsam_lib_dir}"
      INSTALL_RPATH "${_gtsam_lib_dir}")
  endif()
else()
  target_compile_definitions(team_pose_graph_optimizer_node PRIVATE HAVE_GTSAM_CPP=0)
endif()

install(TARGETS team_pose_graph_optimizer_node DESTINATION lib/${PROJECT_NAME})

ament_package()
