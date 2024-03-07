#ifndef _DECISION_NODE_H
#define _DECISION_NODE_H

#include "./sentry_decision_v2/structs.h"
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <auto_aim_interfaces/msg/uart.hpp>
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <fstream>
#include <regex>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "Json/json.h"

namespace sentry
{
    using namespace std::chrono_literals;
    class RobotDecisionNode : public rclcpp::Node
    {
    public:
        RobotDecisionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~RobotDecisionNode();

    private:
        void initParam();
        void initSubscribers();
        void initPublishers();
        void init();
        void uartSubCallback(const auto_aim_interfaces::msg::Uart::SharedPtr msg);
        bool readJsonFile();
        void run();
        void nav2GoalStatusCallBack(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
        void nav2FeedBackCallBack(const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg);
        int calcCurrentWayPoint(Eigen::Vector2d &pos);
        std::shared_ptr<Decision_Warp> makeDecision();
        std::shared_ptr<Way_Point> getWay_PointByID(int id);
        bool executeMission(Mission &mission);
        Mission parseMission(std::string &str);

    private:
        std::shared_ptr<rclcpp::Rate> local_rate_;
        Blackboard blackboard;
        std::deque<Mission> missionList;
        std::string decisions_file, waypoints_file;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_;
        rclcpp::Subscription<auto_aim_interfaces::msg::Uart>::SharedPtr uart_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mark_pub_;
        rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
            nav_to_pose_feedback_sub_;
        rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
            nav_to_pose_goal_status_sub_;
        std::shared_ptr<std::thread> main_thread_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_poses_action_client_;
        std::vector<std::shared_ptr<Way_Point>> waypoints;
        std::vector<std::shared_ptr<Decision_Warp>> decisions;

        std::shared_ptr<Decision_Warp> past_decision = nullptr;
        Mission last_Mission;
    };
} // namespace sentry

#endif // _DECISION_NODE_H