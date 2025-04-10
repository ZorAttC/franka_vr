cmake_minimum_required(VERSION 3.8)
project(franka_vr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    trajectory_msgs)
# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)  # 添加geometry_msgs依赖
find_package(moveit_servo REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()
# 生成服务接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetStepSize.srv"
  "srv/SetTargetPose.srv"  # 添加新服务
  DEPENDENCIES std_msgs geometry_msgs
)
# 获取生成接口的目标
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Executable vr controller (velocity controller)
add_executable(demo_franka_vr_vel src/franka_vr_vel.cpp)
target_include_directories(demo_franka_vr_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(demo_franka_vr_vel 
  moveit_servo::moveit_servo_lib_cpp
  ${cpp_typesupport_target}  # 链接生成的服务接口
)
ament_target_dependencies(demo_franka_vr_vel ${THIS_PACKAGE_INCLUDE_DEPENDS})


install(TARGETS
demo_franka_vr_vel
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
    config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
)
# 声明这是一个ROS 2包
ament_package()