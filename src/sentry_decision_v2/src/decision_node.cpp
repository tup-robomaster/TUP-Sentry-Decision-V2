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
    if (!readJsonFile())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to read JsonFile, please check and relaunch the node.");
      return;
    }
    init();
  }

  RobotDecisionNode::~RobotDecisionNode()
  {
    main_thread_->join();
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
    this->uart_sub_ = this->create_subscription<auto_aim_interfaces::msg::Uart>(
        "/uart",
        rclcpp::SensorDataQoS(),
        std::bind(&RobotDecisionNode::uartSubCallback, this, std::placeholders::_1));
    this->nav_to_pose_feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
        "navigate_to_pose/_action/feedback",
        rclcpp::SensorDataQoS(),
        std::bind(&RobotDecisionNode::nav2FeedBackCallBack, this, std::placeholders::_1));
    this->nav_to_pose_goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "navigate_to_pose/_action/status",
        rclcpp::SensorDataQoS(),
        std::bind(&RobotDecisionNode::nav2GoalStatusCallBack, this, std::placeholders::_1));
  }

  void RobotDecisionNode::initPublishers()
  {
    mark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("decision_mark", rclcpp::SensorDataQoS());
  }

  void RobotDecisionNode::init()
  {
    RCLCPP_INFO(
        this->get_logger(),
        "Starting action_client");
    this->nav_to_poses_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    if (!this->nav_to_poses_action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Action server not available after waiting");
    }
    else
    {
      this->nav_to_poses_action_client_->async_cancel_all_goals();
    }
    main_thread_ = std::make_shared<std::thread>(std::bind(&RobotDecisionNode::run, this));
  }

  void RobotDecisionNode::uartSubCallback(const auto_aim_interfaces::msg::Uart::SharedPtr msg)
  {
    blackboard.update(msg->sentry_hp,    // 哨兵血量
                      msg->outpost_hp,   // 己方前哨站血量
                      msg->remain_time,  // 比赛剩余时间
                      msg->game_stage,   // 比赛进行阶段
                      msg->cmd_keyboard, // 云台手指令
                      msg->x, msg->y);   // 云台手指令坐标
  }

  bool RobotDecisionNode::readJsonFile()
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("sentry_decision_v2");
    Json::Reader jsonReader;
    Json::Value jsonValue;
    std::ifstream jsonFile_waypoints(package_share_directory + "/JsonFiles/" + waypoints_file);
    if (!jsonReader.parse(jsonFile_waypoints, jsonValue, true))
    {
      RCLCPP_ERROR(this->get_logger(), "JSON FILE I/O ERR !");
      jsonFile_waypoints.close();
      return false;
    }
    Json::Value arrayValue = jsonValue["data"];
    waypoints.reserve(arrayValue.size());
    for (int i = 0; i < int(arrayValue.size()); ++i)
    {
      std::shared_ptr<Way_Point> temp(new Way_Point);
      temp->id = arrayValue[i]["id"].asInt();         // 路径点ID
      temp->name = arrayValue[i]["name"].asCString(); // 路径点名
      temp->x = arrayValue[i]["x"].asDouble();        // 路径点坐标X
      temp->y = arrayValue[i]["y"].asDouble();        // 路径点坐标Y
      temp->w = arrayValue[i]["w"].asDouble();        // 路径点朝向
      waypoints.emplace_back(temp);
    }
    arrayValue.clear();
    jsonValue.clear();
    jsonFile_waypoints.close();
    std::ifstream jsonFile_decisions(package_share_directory + "/JsonFiles/" + decisions_file);
    if (!jsonReader.parse(jsonFile_decisions, jsonValue, true))
    {
      RCLCPP_ERROR(this->get_logger(), "JSON FILE I/O ERR !");
      jsonFile_decisions.close();
      return false;
    }
    arrayValue = jsonValue["data"];
    decisions.reserve(arrayValue.size());
    for (int i = 0; i < int(arrayValue.size()); ++i)
    {
      std::shared_ptr<Decision_Warp> temp(new Decision_Warp);
      temp->id = arrayValue[i]["id"].asInt();                         // 决策ID
      temp->group = arrayValue[i]["group"].asCString();               // 决策组别
      temp->name = arrayValue[i]["name"].asCString();                 // 决策名
      temp->waypointID = arrayValue[i]["wayPointID"].asInt();         // 决策所属路径点ID
      temp->weight = arrayValue[i]["weight"].asInt();                 // 决策权重
      temp->start_time = arrayValue[i]["start_time"].asInt();         // 决策作用时间起始点
      temp->end_time = arrayValue[i]["end_time"].asInt();             // 决策作用时间终止点
      temp->minHP = arrayValue[i]["minHP"].asInt();                   // 决策作用血量最低点
      temp->maxHP = arrayValue[i]["maxHP"].asInt();                   // 决策作用血量最高点
      temp->outpost_minHP = arrayValue[i]["out_post_HP_min"].asInt(); // 决策作用时最小前哨站血量
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

  // 决策循环方法，负责周期性地进行决策
  void RobotDecisionNode::run()
  {
    // 主循环，只要ROS节点处于运行状态就一直执行
    while (rclcpp::ok())
    {
      // 休眠，控制决策循环的频率
      local_rate_->sleep();
      // 检查UART消息是否可用，若不可用则记录警告
      if (!this->blackboard.checkAvilable())
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "UartMsg Not Avilable Now!");
        continue;
      }
      // 检查游戏阶段是否为开始状态，若不是则记录警告
      if (this->blackboard.getStage() != GAME_STAGE_START)
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "NOT RECIVE START SIGINAL");
        continue;
      }
      // 更新黑板中的时间信息
      blackboard.setTime(this->get_clock()->now().nanoseconds() / 1e9);
      geometry_msgs::msg::TransformStamped::SharedPtr transformStamped = nullptr;
      // 尝试获取base_link到map坐标系的变换
      try
      {
        transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>(this->tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero));
      }
      catch (tf2::TransformException &ex)
      {
        // 记录变换获取失败的错误
        RCLCPP_ERROR(
            this->get_logger(),
            "Cannot get transform ! TransformException: %s",
            ex.what());
        continue;
      }
      // 若变换为空，则记录错误并继续下一次循环
      if (transformStamped == nullptr)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to get transformStamped");
        continue;
      }
      // 获取当前机器人的位置
      Eigen::Vector2d now_pose;
      now_pose << transformStamped->transform.translation.x, transformStamped->transform.translation.y;
      blackboard.updatePos(now_pose);
      // 计算机器人当前所在的路标点
      blackboard.setWayPointID(calcCurrentWayPoint(now_pose));
      // 制定决策并获取当前最优决策
      std::shared_ptr<Decision_Warp> currentDecision = nullptr;
      if (blackboard.checkCmd_keyboard())
      {
        Decision_Warp temp;
        temp.actions.push_back("move_" + blackboard.getCmd_X() + "_" + blackboard.getCmd_Y());
        temp.end_time = -1;
        temp.group = "CMD";
        temp.start_time = -1;
        temp.end_time = -1;
        temp.id = -1;
        temp.maxHP = -1;
        temp.minHP = -1;
        temp.outpost_minHP = 300;
        temp.name = "cmd";
        temp.waypointID = -1;
        temp.weight = 1000;
        currentDecision = std::make_shared<Decision_Warp>(temp);
      }
      else
      {
        currentDecision = makeDecision();
      }
      // 若无法制定决策，则记录错误并继续下一次循环
      if (currentDecision == nullptr)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to make Decision! Check your profile");
        continue;
      }
      // 检查任务是否全部完成
      if (blackboard.checkMissionsComplete())
      {
        // 若无正在执行的决策，则执行下一个决策
        RCLCPP_INFO(this->get_logger(), "No decision executing, excute next!");
        RCLCPP_INFO(this->get_logger(), "Select Decision: %d", currentDecision->id);
        past_decision = currentDecision;
        // 遍历当前决策的所有动作，将它们添加到任务队列中
        for (auto &it : currentDecision->actions)
        {
          Mission mission = parseMission(it);
          blackboard.insertMission(mission);
        }
      }
      else
      {
        // 若当前决策的权重大于过去决策的权重，或者当前任务执行失败或已取消，则覆盖任务队列
        if (past_decision->weight < currentDecision->weight ||
            blackboard.getMission().status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
            blackboard.getMission().status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
            blackboard.getMission().status == action_msgs::msg::GoalStatus::STATUS_UNKNOWN)
        {
          RCLCPP_INFO(this->get_logger(), "Decision override ,Mission changed!");
          past_decision = currentDecision;
          blackboard.cleanUpMissions();
        }
        // 若当前任务已成功且等待任务完成，则移除队首任务
        else if (blackboard.checkWaitingComplete() && blackboard.getMission().status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
        {
          RCLCPP_INFO(this->get_logger(), "Mission complete, pop!");
          last_Mission = blackboard.getMission();
          blackboard.popMission();
        }
        // 如果当前任务状态不是执行中，则检查任务队列是否有任务待执行
        if (blackboard.getMission().status != action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        {
          // 如果任务队列不为空且队首任务已经等待完成，则执行下一个任务
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
    std::sort(local_decisions.begin(), local_decisions.end(), [](const std::shared_ptr<sentry::Decision_Warp> &A, const std::shared_ptr<sentry::Decision_Warp> &B)
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
        msg->feedback.distance_remaining,
        msg->feedback.current_pose.pose.position.x,
        msg->feedback.current_pose.pose.position.y,
        msg->feedback.current_pose.pose.position.z,
        msg->feedback.estimated_time_remaining.sec,
        msg->feedback.current_pose.header.frame_id.c_str());
  }

  void RobotDecisionNode::nav2GoalStatusCallBack(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    RCLCPP_INFO(
        this->get_logger(),
        "Nav2StatusCallBack Status: %d",
        msg->status_list.back().status);
    if (blackboard.getMission().name != MISSION_TYPE_WAIT)
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
    mission.starting_time = blackboard.getTime();
    if (mission.name == MISSION_TYPE_WAIT)
    {
      blackboard.setWaitTime(mission.time_stay);
      return true;
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
        if(mission_params.size() == 2)
          mission.move_taget = *getWay_PointByID(std::stoi(mission_params[1]));
        if(mission_params.size() == 3)
        {
          sentry::Way_Point temp;
          temp.id = -1;
          temp.name = "cmd";
          temp.w = 0.0;
          temp.x = mission_params[1];
          temp.y = mission_params[2];
          mission.move_taget = temp;
        }
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sentry::RobotDecisionNode)
