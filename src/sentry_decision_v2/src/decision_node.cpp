#include <cstdio>
#include "../include/decision_node.h"

namespace sentry
{
  RobotDecisionNode::RobotDecisionNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("robot_decision_node", options)
  {
    RCLCPP_INFO(
        this->get_logger(),
        "RobotDecision node...starting");
    initParam();
    initSubscribers();
    initPublishers();
    readJsonFile();
    init();
  }

  RobotDecisionNode::~RobotDecisionNode()
  {
  }

  void RobotDecisionNode::initParam()
  {
    double local_rate;
    this->declare_parameter<double>("local_rate", 20.0);
    this->declare_parameter<std::string>("waypoints_file", "default_waypoints.json");
    this->declare_parameter<std::string>("decisions_file", "default_decisions.json");
    this->get_parameter("local_rate", local_rate);
    local_rate_ = std::make_shared<rclcpp::Rate>(local_rate);
    this->get_parameter("waypoints_file", waypoints_file);
    this->get_parameter("decisions_file", decisions_file);
  }

  void RobotDecisionNode::initSubscribers()
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    uart_sub_ = this->create_subscription<auto_aim_interfaces::msg::Uart>("/uart", rclcpp::QoS(10).best_effort().keep_last(1), std::bind(&RobotDecisionNode::uartSubCallback, this, std::placeholders::_1));
    this->nav_to_pose_feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
        "navigate_through_poses/_action/feedback",
        rclcpp::QoS(10).best_effort().keep_last(1),
        std::bind(&RobotDecisionNode::nav2FeedBackCallBack, this, std::placeholders::_1));
    this->nav_to_pose_goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "navigate_through_poses/_action/status",
        rclcpp::QoS(10).best_effort().keep_last(1),
        std::bind(&RobotDecisionNode::nav2GoalStatusCallBack, this, std::placeholders::_1));
  }

  void RobotDecisionNode::initPublishers()
  {
    mark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("decision_mark", rclcpp::QoS(10).transient_local().keep_last(1));
  }

  void RobotDecisionNode::init()
  {
    RCLCPP_INFO(
        this->get_logger(),
        "Starting action_client");
    this->nav_to_poses_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_through_poses");
    if (!this->nav_to_poses_action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Action server not available after waiting");
      return;
    }
    else
    {
      this->nav_to_poses_action_client_->async_cancel_all_goals();
    }
    main_thread_ = std::make_shared<std::thread>(std::bind(&RobotDecisionNode::run, this));
  }

  void RobotDecisionNode::uartSubCallback(const auto_aim_interfaces::msg::Uart::SharedPtr msg)
  {
    blackboard.update(msg->sentry_hp, msg->outpost_hp, msg->remain_time, msg->game_stage);
  }

  bool RobotDecisionNode::readJsonFile()
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("sentry_decision_v2");
    Json::Reader jsonReader;
    Json::Value jsonValue;
    std::ifstream jsonFile_waypoints(package_share_directory + "/JsonFile/" + waypoints_file);
    if (!jsonReader.parse(jsonFile_waypoints, jsonValue, true))
    {
      RCLCPP_ERROR(this->get_logger(), "JSON FILE I/O ERR !");
      jsonFile_waypoints.close();
      return false;
    }
    Json::Value arrayValue = jsonValue["data"];
    for (int i = 0; i < int(arrayValue.size()); ++i)
    {
      std::shared_ptr<Way_Point> temp(new Way_Point);
      temp->id = arrayValue[i]["id"].asInt();
      temp->name = arrayValue[i]["name"].asCString();
      temp->x = arrayValue[i]["x"].asDouble();
      temp->y = arrayValue[i]["y"].asDouble();
      temp->w = arrayValue[i]["w"].asDouble();
      waypoints.emplace_back(temp);
    }
    arrayValue.clear();
    jsonValue.clear();
    jsonFile_waypoints.close();
    std::ifstream jsonFile_decisions(package_share_directory + "/JsonFile/" + decisions_file);
    if (!jsonReader.parse(jsonFile_decisions, jsonValue, true))
    {
      RCLCPP_ERROR(this->get_logger(), "JSON FILE I/O ERR !");
      jsonFile_decisions.close();
      return false;
    }
    arrayValue = jsonValue["data"];
    for (int i = 0; i < int(arrayValue.size()); ++i)
    {
      std::shared_ptr<Decision_Warp> temp(new Decision_Warp);
      temp->id = arrayValue[i]["id"].asInt();
      temp->group = arrayValue[i]["group"].asCString();
      temp->name = arrayValue[i]["name"].asCString();
      temp->waypointID = arrayValue[i]["wayPointID"].asInt();
      temp->weight = arrayValue[i]["weight"].asInt();
      temp->start_time = arrayValue[i]["start_time"].asInt();
      temp->end_time = arrayValue[i]["end_time"].asInt();
      temp->minHP = arrayValue[i]["minHP"].asInt();
      temp->maxHP = arrayValue[i]["maxHP"].asInt();
      temp->outpost_minHP = arrayValue[i]["out_post_HP_min"].asInt();
      Json::Value actions_array = arrayValue[i]["actions"];
      for (int j = 0; j < int(actions_array.size()); ++j)
      {
        temp->actions.emplace_back(actions_array[j].asCString());
      }
      decisions.emplace_back(temp);
    }
    jsonFile_decisions.close();
    return true;
  }

  void RobotDecisionNode::run()
  {
    while (rclcpp::ok())
    {
      local_rate_->sleep();
      if (!this->blackboard.checkAvilable())
        continue;
      if (this->blackboard.getStage() != GAME_STAGE_START)
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "NOT RECIVE START SIGINAL");
        continue;
      }
      blackboard.setTime(this->get_clock()->now().nanoseconds() / 1e9);
      geometry_msgs::msg::TransformStamped::SharedPtr transformStamped = nullptr;
      try
      {
        transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>(this->tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero));
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "Cannot get transform ! TransformException: %s",
            ex.what());
        continue;
      }
      if (transformStamped == nullptr)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to get transformStamped");
        continue;
      }
      Eigen::Vector2d now_pose;
      now_pose << transformStamped->transform.translation.x, transformStamped->transform.translation.y;
      blackboard.updatePos(now_pose);
      blackboard.setWayPointID(calcCurrentWayPoint(now_pose));
      std::shared_ptr<Decision_Warp> currentDecision = makeDecision();
      if (currentDecision == nullptr)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to make Decision! Check your profile");
        continue;
      }
      if (blackboard.checkMissionsComplete())
      {
        RCLCPP_INFO(this->get_logger(), "No decision executing, excute next!");
        past_decision = currentDecision;
        for (auto &it : currentDecision->actions)
        {
          Mission mission = parseMission(it);
          blackboard.insertMission(mission);
        }
      }
      else
      {
        if (past_decision->weight < currentDecision->weight ||
            blackboard.getMission().status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
            blackboard.getMission().status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
            blackboard.getMission().status == action_msgs::msg::GoalStatus::STATUS_UNKNOWN)
        {
          RCLCPP_INFO(this->get_logger(), "Decision override ,Mission changed!");
          past_decision = currentDecision;
          blackboard.cleanUpMissions();
        }
        else if (blackboard.getMission().status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED && blackboard.checkWaitingComplete())
        {
          RCLCPP_INFO(this->get_logger(), "Mission complete, pop!");
          last_Mission = blackboard.getMission();
          blackboard.popMission();
        }
        if (blackboard.getMission().status != action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        {
          if (!blackboard.checkMissionsComplete() && blackboard.checkWaitingComplete())
          {
            RCLCPP_INFO(this->get_logger(), "No mission executing, execute next!");
            Mission &current_mission = blackboard.getMission();
            executeMission(current_mission);
          }
        }
      }
    }
  }

  int RobotDecisionNode::calcCurrentWayPoint(Eigen::Vector2d &pos)
  {
    int nearestWayPointID = -1;
    double min_distance = INFINITY;
    for (auto &it : waypoints)
    {
      double temp_distance = (pos - Eigen::Vector2d(it->x, it->y)).norm();
      if (temp_distance < min_distance)
      {
        min_distance = temp_distance;
        nearestWayPointID = it->id;
      }
    }
    if (min_distance > MAX_WAYPOINT_DISTANCE)
      nearestWayPointID = -1;
    return nearestWayPointID;
  }

  std::shared_ptr<Decision_Warp> RobotDecisionNode::makeDecision()
  {
    std::vector<std::shared_ptr<Decision_Warp>> local_decisions;
    for (auto &it : decisions)
    {
      if (blackboard.compare(*it))
        local_decisions.emplace_back(it);
    }
    std::sort(local_decisions.begin(), local_decisions.end(), [](std::shared_ptr<sentry::Decision_Warp> A, std::shared_ptr<sentry::Decision_Warp> B)
              { return A->weight > B->weight; });
    if (local_decisions.empty())
      return nullptr;
    return local_decisions[0];
  }

  void RobotDecisionNode::nav2FeedBackCallBack(const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg)
  {
    RCLCPP_DEBUG(
        this->get_logger(),
        "Receive Nav2FeedBack: Distance Remainimg: %f Current Pose: x=%lf , y=%lf , z=%lf Time Remaining: %d in frame %s",
        msg->feedback.distance_remaining, msg->feedback.current_pose.pose.position.x, msg->feedback.current_pose.pose.position.y, msg->feedback.current_pose.pose.position.z, msg->feedback.estimated_time_remaining.sec, msg->feedback.current_pose.header.frame_id.c_str());
  }

  void RobotDecisionNode::nav2GoalStatusCallBack(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
      RCLCPP_INFO(
          this->get_logger(),
          "Nav2StatusCallBack Status: %d",
          msg->status_list.back().status);
      blackboard.setMissionStatus(msg->status_list.back().status);
  }

  std::shared_ptr<Way_Point> RobotDecisionNode::getWay_PointByID(int id)
  {
    for (auto &it : this->waypoints)
    {
      if (it->id == id)
      {
        return it;
      }
    }
    return nullptr;
  }

  bool RobotDecisionNode::executeMission(Mission &mission)
  {
    if (mission.name == MISSION_TYPE_WAIT)
    {
      blackboard.setWaitTime(mission.time_stay);
    }
    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    if (this->nav_to_poses_action_client_->wait_for_action_server(std::chrono::microseconds(100)))
    {
      auto pose = geometry_msgs::msg::PoseStamped();
      pose.header.stamp = this->get_clock()->now();
      pose.header.frame_id = "map";
      pose.pose.position.x = mission.move_taget.x;
      pose.pose.position.y = mission.move_taget.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(mission.move_taget.w);
      auto goal = nav2_msgs::action::NavigateToPose::Goal();
      goal.pose = pose;
      auto future_goal_handle = nav_to_poses_action_client_->async_send_goal(goal, send_goal_options);
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Action server still not available !");
      return false;
    }
    mission.starting_time = blackboard.getTime();
    return true;
  }

  Mission RobotDecisionNode::parseMission(std::string &str)
  {
    Mission mission;
    std::regex reg("_");
    std::vector<std::string> mission_params(std::sregex_token_iterator(str.begin(), str.end(), reg, -1),
                                            std::sregex_token_iterator());
    if (!mission_params.empty())
    {
      mission.name = mission_params[0];
      if (mission.name == MISSION_TYPE_MOVE)
      {
        mission.move_taget = *getWay_PointByID(std::stoi(mission_params[1]));
      }
      else if (mission.name == MISSION_TYPE_WAIT)
      {
        if (last_Mission.move_taget.name.empty())
        {
          Way_Point temp_waypoint;
          temp_waypoint.id = -1;
          temp_waypoint.name = "origin";
          double w;
          Eigen::Vector2d pos;
          blackboard.getPose(pos, w);
          temp_waypoint.x = pos[0];
          temp_waypoint.y = pos[1];
          temp_waypoint.w = w;
          mission.move_taget = temp_waypoint;
        }
        else
        {
          mission.move_taget = last_Mission.move_taget;
        }
        mission.time_stay = std::stod(mission_params[1]);
      }
    }
    return mission;
  }
} // namespace sentry

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto my_node = std::make_shared<sentry::RobotDecisionNode>();
  rclcpp::spin(my_node);
  rclcpp::shutdown();
  return 0;
}
