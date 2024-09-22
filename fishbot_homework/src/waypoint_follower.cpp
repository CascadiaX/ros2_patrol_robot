#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <nav2_msgs/action/follow_waypoints.hpp>

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

class FollowWaypointsClient : public rclcpp::Node
{
public:
    using NavigationActionClient = rclcpp_action::Client<FollowWaypoints>;
    // 定义导航动作客户端类型
    using NavigationActionGoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>; // 定义导航动作目标句柄类型

    FollowWaypointsClient() : Node("follow_waypoints_client")
    {
        action_client_ = rclcpp_action::create_client<FollowWaypoints>(this,
                                                                       "follow_waypoints");
    }

    void sendGoals()
    {
        // 等待导航动作服务器上线,等待时间为5s
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(get_logger(), "等待Action服务上线。");
        }
        // 设置导航目标点集
        auto goals_msg = FollowWaypoints::Goal();

        geometry_msgs::msg::PoseStamped pose1;
        pose1.header.frame_id = "map";
        pose1.pose.position.x = 0.0f;
        pose1.pose.position.y = 0.0f;
        pose1.pose.orientation.w = 1.0f;
        goals_msg.poses.push_back(pose1);

        geometry_msgs::msg::PoseStamped pose2;
        pose2.header.frame_id = "map";
        pose2.pose.position.x = 2.0f;
        pose2.pose.position.y = 0.0f;
        pose2.pose.orientation.w = 1.0f;
        goals_msg.poses.push_back(pose2);

        geometry_msgs::msg::PoseStamped pose3;
        pose3.header.frame_id = "map";
        pose3.pose.position.x = 2.0f;
        pose3.pose.position.y = 2.0f;
        pose3.pose.orientation.w = 1.0f;
        goals_msg.poses.push_back(pose3);

        auto send_goal_options =
            rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

        // 设置请求目标结果回调函数
        send_goal_options.goal_response_callback =
            [this](NavigationActionGoalHandle::SharedPtr goal_handle)
        {
            if (goal_handle)
            {
                RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
            }
        };
        // 设置移动过程反馈回调函数
        send_goal_options.feedback_callback =
            [this](NavigationActionGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
        {
            (void)goal_handle; // 假装调用,避免warning:unused
            RCLCPP_INFO(this->get_logger(), "当前目标编号:%u", feedback->current_waypoint);
        };

        // 设置执行结果回调函数
        send_goal_options.result_callback =
            [this](const NavigationActionGoalHandle::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "所有目标点处理成功!");
            }
            else if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_WARN(this->get_logger(), "导航被取消。");
            }
            else if (result.code == rclcpp_action::ResultCode::ABORTED)
            {
                RCLCPP_ERROR(this->get_logger(), "导航任务被中止。");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "导航任务结果无效。");
            }
        };

        action_client_->async_send_goal(goals_msg, send_goal_options); // 发送导航目标点
    }

    NavigationActionClient::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowWaypointsClient>();
    node->sendGoals();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
