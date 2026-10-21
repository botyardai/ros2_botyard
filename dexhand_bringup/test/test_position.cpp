#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath> // For std::fabs
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

std::vector<double> desired_goals = {0, -1, 1, 0.0};
unsigned int ct_goals_reached = 0;

void common_goal_response(
        rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr future)
{
    RCLCPP_DEBUG(node->get_logger(), "common_goal_response time: %f", rclcpp::Clock().now().seconds());
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        common_goal_accepted = false;
        RCLCPP_ERROR(node->get_logger(), "Goal rejected");
    }
    else
    {
        common_goal_accepted = true;
        RCLCPP_INFO(node->get_logger(), "Goal accepted");
    }
}

void common_result_response(
        const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult &result)
{
    RCLCPP_INFO(node->get_logger(), "common_result_response time: %f", rclcpp::Clock(RCL_ROS_TIME).now().seconds());
    common_resultcode = result.code;
    common_action_result_code = result.result->error_code;
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(), "Action goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node->get_logger(), "Action goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node->get_logger(), "Action goal was canceled");
            break;
        default:
            RCLCPP_INFO(node->get_logger(), "Action goal: Unknown result code");
            break;
    }
}

void common_feedback(
        rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
        const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
    RCLCPP_INFO(node->get_logger(), "feedback->desired: positions: %f, velocities: %f",
                feedback->desired.positions.at(0), feedback->desired.velocities.at(0));

    RCLCPP_INFO(node->get_logger(), "feedback->actual: positions: %f",
                feedback->actual.positions.at(0));

    if (ct_goals_reached < desired_goals.size())
    {
        if (std::fabs(feedback->actual.positions[0] - desired_goals.at(ct_goals_reached)) < 0.1)
        {
            RCLCPP_INFO(node->get_logger(), "Goal # %d: %f reached", ct_goals_reached, desired_goals.at(ct_goals_reached));
            ct_goals_reached++;
            if (ct_goals_reached < desired_goals.size())
            {
                RCLCPP_INFO(node->get_logger(), "Next goal # %d: %f", ct_goals_reached, desired_goals.at(ct_goals_reached));
            }
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("trajectory_test_node");
    RCLCPP_INFO(node->get_logger(), "Node created");

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client =
            rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
                    node->get_node_base_interface(),
                    node->get_node_graph_interface(),
                    node->get_node_logging_interface(),
                    node->get_node_waitables_interface(),
                    "/hand_controller/follow_joint_trajectory");

    while (!action_client->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(node->get_logger(), "Trying to connect to the server again");
    }
    RCLCPP_INFO(node->get_logger(), "Connected to action server");

    std::vector<std::string> joint_names = {
            "lh_THJ4", "lh_THJ3", "lh_THJ2", "lh_THJ1", "lh_FFJ4", "lh_FFJ3", "lh_FFJ2", "lh_FFJ1",
            "lh_MFJ4", "lh_MFJ3", "lh_MFJ2", "lh_MFJ1", "lh_RFJ4", "lh_RFJ3", "lh_RFJ2", "lh_RFJ1",
            "lh_LFJ5", "lh_LFJ4", "lh_LFJ3", "lh_LFJ2", "lh_LFJ1",
            "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1", "rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1",
            "rh_MFJ4", "rh_MFJ3", "rh_MFJ2", "rh_MFJ1", "rh_RFJ4", "rh_RFJ3", "rh_RFJ2", "rh_RFJ1",
            "rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ2", "rh_LFJ1"
    };

    control_msgs::action::FollowJointTrajectory_Goal goal_msg;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
    goal_msg.trajectory.joint_names = joint_names;
    while (rclcpp::ok())
    {
        goal_msg.trajectory.header.stamp = rclcpp::Time();
        for (double desired_goal : desired_goals)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions.resize(joint_names.size(), desired_goal); // 设置所有关节的目标位置
            point.velocities.resize(joint_names.size(), 0.0);         // 可选：设置速度为零
            point.accelerations.resize(joint_names.size(), 0.0);      // 可选：设置加速度为零
            // 确保时间戳是严格递增的
            point.time_from_start = rclcpp::Duration::from_seconds(ct_goals_reached * 1.0);
            goal_msg.trajectory.points.push_back(point);
            ct_goals_reached++;
        }
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
        opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
        opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
        opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);
        auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);
        if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Send goal call failed");
            return 1;
        }
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
            return 1;
        }
        auto result_future = action_client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Get result call failed");
            return 1;
        }
        if (desired_goals.size() != ct_goals_reached)
        {
            RCLCPP_ERROR(node->get_logger(), "Not all goals were reached");
            return -1;
        }
        sleep(1);
    }
    rclcpp::shutdown();
    return 0;
}