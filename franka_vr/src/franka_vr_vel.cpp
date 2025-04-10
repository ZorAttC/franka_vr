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
#include "franka_vr/srv/set_target_pose.hpp"
#include <Eigen/Geometry>

using namespace moveit_servo;

namespace
{
constexpr auto K_BASE_FRAME = "fr3_link0";
constexpr auto K_TIP_FRAME = "fr3_hand";
constexpr double POSITION_TOLERANCE = 0.002;     // 位置容差（米）
constexpr double ORIENTATION_TOLERANCE = 0.05;  // 角度容差（弧度）
}  // namespace

static Eigen::Vector3d linear_step_size{ 3.0, 3.0, 3.0 };
static Eigen::AngleAxisd x_step_size(3.14, Eigen::Vector3d::UnitX());
static Eigen::AngleAxisd y_step_size(3.14, Eigen::Vector3d::UnitY());
static Eigen::AngleAxisd z_step_size(3.14, Eigen::Vector3d::UnitZ());
static geometry_msgs::msg::Pose target_pose;
static bool target_set = false;

void set_step_size_callback(const rclcpp::Node::SharedPtr& node,
                            const std::shared_ptr<franka_vr::srv::SetStepSize::Request> request,
                            std::shared_ptr<franka_vr::srv::SetStepSize::Response> response)
{
  try
  {
    linear_step_size = Eigen::Vector3d{ request->linear_x, request->linear_y, request->linear_z };
    x_step_size = Eigen::AngleAxisd(request->angular_x, Eigen::Vector3d::UnitX());
    y_step_size = Eigen::AngleAxisd(request->angular_y, Eigen::Vector3d::UnitY());
    z_step_size = Eigen::AngleAxisd(request->angular_z, Eigen::Vector3d::UnitZ());
    response->success = true;
    response->message = "Step sizes updated successfully";
    RCLCPP_INFO(node->get_logger(), "Updated step sizes: linear[%.3f, %.3f, %.3f], angular[%.3f, %.3f, %.3f]",
                linear_step_size.x(), linear_step_size.y(), linear_step_size.z(), x_step_size.angle(),
                y_step_size.angle(), z_step_size.angle());
  }
  catch (const std::exception& e)
  {
    response->success = false;
    response->message = std::string("Failed to update step sizes: ") + e.what();
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
  }
}

void set_target_pose_callback(const rclcpp::Node::SharedPtr& node,
                              const std::shared_ptr<franka_vr::srv::SetTargetPose::Request> request,
                              std::shared_ptr<franka_vr::srv::SetTargetPose::Response> response)
{
  target_pose = request->target_pose;
  target_set = true;
  response->success = true;
  response->message = "Target pose set successfully";
  RCLCPP_INFO(node->get_logger(), "Target pose set: [x: %.3f, y: %.3f, z: %.3f]", target_pose.position.x,
              target_pose.position.y, target_pose.position.z);
}

Eigen::Vector3d compute_position_error(const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& target)
{
  return Eigen::Vector3d(target.position.x - current.position.x, target.position.y - current.position.y,
                         target.position.z - current.position.z);
}

double compute_orientation_error(const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& target)
{
  tf2::Quaternion q_current, q_target;
  tf2::fromMsg(current.orientation, q_current);
  tf2::fromMsg(target.orientation, q_target);
  return q_current.angleShortestPath(q_target);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");
  moveit::setNodeLoggerName(demo_node->get_name());

  auto step_size_service = demo_node->create_service<franka_vr::srv::SetStepSize>(
      "set_step_size", std::bind(&set_step_size_callback, demo_node, std::placeholders::_1, std::placeholders::_2));
  auto target_pose_service = demo_node->create_service<franka_vr::srv::SetTargetPose>(
      "set_target_pose", std::bind(&set_target_pose_callback, demo_node, std::placeholders::_1, std::placeholders::_2));

  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  auto trajectory_outgoing_cmd_pub = demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  const auto get_current_pose = [](const std::string& target_frame, const moveit::core::RobotStatePtr& robot_state) {
    return robot_state->getGlobalLinkTransform(target_frame);
  };

  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  servo.setCommandType(CommandType::TWIST);

  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState(true);
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

  std::mutex pose_guard;
  auto pose_tracker = [&]() {
    KinematicState joint_state;
    rclcpp::WallRate tracking_rate(1 / servo_params.publish_period);
    std::deque<KinematicState> joint_cmd_rolling_window;
    KinematicState current_state = servo.getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

    while (rclcpp::ok())
    {
      if (target_set)
      {
        std::lock_guard<std::mutex> pguard(pose_guard);
        geometry_msgs::msg::Pose current_pose = tf2::toMsg(get_current_pose(K_TIP_FRAME, robot_state));
        Eigen::Vector3d pos_error = compute_position_error(current_pose, target_pose);
        double ori_error = compute_orientation_error(current_pose, target_pose);

        if (pos_error.norm() < POSITION_TOLERANCE && ori_error < ORIENTATION_TOLERANCE)
        {
          RCLCPP_INFO(demo_node->get_logger(), "Reached target pose within tolerance");
          target_set = false;
          continue;
        }

        double max_linear_speed = linear_step_size.norm() > 0 ? linear_step_size.norm() : 0.05;
        double max_angular_speed = std::max({ x_step_size.angle(), y_step_size.angle(), z_step_size.angle() }) > 0 ?
                                       std::max({ x_step_size.angle(), y_step_size.angle(), z_step_size.angle() }) :
                                       0.4;
                              
        Eigen::Vector3d linear_vel = pos_error.normalized() * max_linear_speed * pos_error.norm();

        // 计算角速度
        tf2::Quaternion q_current, q_target;
        tf2::fromMsg(current_pose.orientation, q_current);
        tf2::fromMsg(target_pose.orientation, q_target);
        tf2::Quaternion q_diff = q_target * q_current.inverse();
        tf2::Vector3 angular_vel_axis = q_diff.getAxis();  // 使用tf2::Vector3
        double angular_vel_magnitude = q_diff.getAngle();
        Eigen::Vector3d angular_vel =
            Eigen::Vector3d(angular_vel_axis.x(), angular_vel_axis.y(), angular_vel_axis.z()) * max_angular_speed *
            angular_vel_magnitude;  // 使用angular_vel_magnitude

        TwistCommand target_twist{ K_BASE_FRAME,
                                   { linear_vel.x(), linear_vel.y(), linear_vel.z(), angular_vel.x(), angular_vel.y(),
                                     angular_vel.z() } };

        joint_state = servo.getNextJointState(robot_state, target_twist);
        StatusCode status = servo.getStatus();

        if (status != StatusCode::INVALID)
        {
          updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency,
                              demo_node->now());
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

  rclcpp::WallRate command_rate(70);
  RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());

  while (rclcpp::ok())
  {
    rclcpp::spin_some(demo_node);
    command_rate.sleep();
  }

  if (tracker_thread.joinable())
    tracker_thread.join();

  rclcpp::shutdown();
  return 0;
}