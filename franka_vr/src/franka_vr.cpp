// control pose traj
#include <atomic>
#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>
#include "franka_vr/srv/set_step_size.hpp"
using namespace moveit_servo;

namespace
{
constexpr auto K_BASE_FRAME = "panda_link0";
constexpr auto K_TIP_FRAME = "panda_link8";
}  // namespace

static Eigen::Vector3d linear_step_size{ 0.00, 0.00, 0.00 };
static Eigen::AngleAxisd x_step_size(0.00, Eigen::Vector3d::UnitX());
static Eigen::AngleAxisd y_step_size(0.00, Eigen::Vector3d::UnitY());
static Eigen::AngleAxisd z_step_size(0.00, Eigen::Vector3d::UnitZ());

void set_step_size_callback(const rclcpp::Node::SharedPtr& node,
                            const std::shared_ptr<franka_vr::srv::SetStepSize::Request> request,
                            std::shared_ptr<franka_vr::srv::SetStepSize::Response> response)
{
  try
  {
    // 更新线性步长
    linear_step_size = Eigen::Vector3d{ request->linear_x, request->linear_y, request->linear_z };

    // 更新角步长
    x_step_size = Eigen::AngleAxisd(request->angular_x, Eigen::Vector3d::UnitX());
    y_step_size = Eigen::AngleAxisd(request->angular_y, Eigen::Vector3d::UnitY());
    z_step_size = Eigen::AngleAxisd(request->angular_z, Eigen::Vector3d::UnitZ());

    // 设置成功响应
    response->success = true;
    response->message = "Step sizes updated successfully";

    RCLCPP_INFO(node->get_logger(), "Updated step sizes: linear[%.3f, %.3f, %.3f], angular[%.3f, %.3f, %.3f]",
                linear_step_size.x(), linear_step_size.y(), linear_step_size.z(), x_step_size.angle(),
                y_step_size.angle(), z_step_size.angle());
  }
  catch (const std::exception& e)
  {
    // 设置失败响应
    response->success = false;
    response->message = std::string("Failed to update step sizes: ") + e.what();
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // 创建节点
  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");
  moveit::setNodeLoggerName(demo_node->get_name());

  // 创建服务
  auto service = demo_node->create_service<franka_vr::srv::SetStepSize>(
      "set_step_size", std::bind(&set_step_size_callback, demo_node, std::placeholders::_1, std::placeholders::_2));

  // 获取servo参数
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  // 创建发布者
  auto trajectory_outgoing_cmd_pub = demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());

  // 创建Servo对象
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  // 获取当前位姿的辅助函数
  const auto get_current_pose = [](const std::string& target_frame, const moveit::core::RobotStatePtr& robot_state) {
    return robot_state->getGlobalLinkTransform(target_frame);
  };

  // 等待规划场景加载（仅用于演示）
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // 获取机器人状态和关节模型组
  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  // 设置servo命令类型
  servo.setCommandType(CommandType::POSE);

  // 动态更新的目标位姿
  PoseCommand target_pose;
  target_pose.frame_id = K_BASE_FRAME;
  target_pose.pose = get_current_pose(K_TIP_FRAME, robot_state);

  // 创建命令队列并初始化当前状态
  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState(true /* wait for updated state */);
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

  std::mutex pose_guard;

  // 位姿跟踪线程
  auto pose_tracker = [&]() {
    KinematicState joint_state;
    rclcpp::WallRate tracking_rate(1 / servo_params.publish_period);
    std::deque<KinematicState> joint_cmd_rolling_window;
    KinematicState current_state = servo.getCurrentRobotState(true /* wait for updated state */);
    updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

    while (rclcpp::ok())
    {
      {
        std::lock_guard<std::mutex> pguard(pose_guard);
        joint_state = servo.getNextJointState(robot_state, target_pose);
      
      StatusCode status = servo.getStatus();

      if (status != StatusCode::INVALID)
      {
        updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
        if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
        {
          trajectory_outgoing_cmd_pub->publish(msg.value());
        }
        if (!joint_cmd_rolling_window.empty())
        {
          robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
          robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
        }
      }
      }
      tracking_rate.sleep();
    }
  };

  std::thread tracker_thread(pose_tracker);
  tracker_thread.detach();

  // 主循环发送命令
  rclcpp::WallRate command_rate(50);
  RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());

  while (rclcpp::ok())
  {
    {
      std::lock_guard<std::mutex> pguard(pose_guard);
      target_pose.pose = get_current_pose(K_TIP_FRAME, robot_state);
      target_pose.pose.translate(linear_step_size);
      target_pose.pose.rotate(x_step_size);
      target_pose.pose.rotate(y_step_size);
      target_pose.pose.rotate(z_step_size);
    }
    rclcpp::spin_some(demo_node);
    command_rate.sleep();
  }

  if (tracker_thread.joinable())
    tracker_thread.join();

  rclcpp::shutdown();
  return 0;
}