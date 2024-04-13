#ifndef _BLACKBOARD_H
#define _BLACKBOARD_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <queue>
#include <mutex>
#include <map>
#include <nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp>

#define MAX_WAYPOINT_DISTANCE double(1.0)
#define STAGE_FULL_TIME int(420)
#define GAME_STAGE_START int(4)

#define MISSION_TYPE_MOVE std::string("move")
#define MISSION_TYPE_WAIT std::string("wait")

namespace sentry
{
    struct Way_Point
    {
        double x = 0.;
        double y = 0.;
        double w = 0.;
        int id = -1;
        std::string name;
    };

    struct Decision_Warp
    {
        int id = -1;
        int waypointID = -1;
        int weight = 0;
        int start_time = -1;
        int end_time = 420;
        int minHP = -1;
        int maxHP = -1;
        int outpost_minHP = -1;
        std::string name;
        std::string group;
        std::vector<std::string> actions;
    };

    struct Mission
    {
        int status;
        std::string name;
        bool if_spin = false;
        Way_Point move_taget;
        double time_stay = 1000;
        double starting_time = -1;
    };

    struct Blackboard
    {
    private:
        std::mutex mutex;
        int _hp = -1;
        Eigen::Vector2d _pos = Eigen::Vector2d::Zero();
        double theta = 0.0;
        int current_waypointID = -1;
        int _oupost_hp_remaining = -1;
        int _time_remaining = -1;
        int _stage = -1;
        double current_time = -1;
        std::queue<Mission> _missions;
        double wait_timeout_time = 0;
        int _commd_keyboard = -1;
        int _x, _y;

    public:
        void update(int hp, int oupost_hp_remaining, int time_remaining, int stage, int commd_keyboard, int x, int y)
        {
            std::lock_guard<std::mutex> lck(mutex);
            _hp = hp;
            _oupost_hp_remaining = oupost_hp_remaining;
            _time_remaining = time_remaining;
            _stage = stage;
            _commd_keyboard = commd_keyboard;
            _x = x;
            _y = y;
        }

        bool checkAvilable()
        {
            std::lock_guard<std::mutex> lck(mutex);
            return _hp != -1 && _oupost_hp_remaining != -1 && _time_remaining != -1 && _stage != -1;
        }

        void updatePos(Eigen::Vector2d &pos)
        {
            std::lock_guard<std::mutex> lck(mutex);
            _pos = pos;
        }

        int getStage()
        {
            std::lock_guard<std::mutex> lck(mutex);
            return _stage;
        }

        void setTime(double t)
        {
            std::lock_guard<std::mutex> lck(mutex);
            current_time = t;
        }

        double getTime()
        {
            std::lock_guard<std::mutex> lck(mutex);
            return current_time;
        }

        void setWayPointID(int id)
        {
            std::lock_guard<std::mutex> lck(mutex);
            current_waypointID = id;
        }

        int getWayPointID()
        {
            std::lock_guard<std::mutex> lck(mutex);
            return current_waypointID;
        }

        bool compare(const Decision_Warp d)
        {
            std::lock_guard<std::mutex> lck(mutex);
            return !((d.waypointID != -1 && d.waypointID != current_waypointID) ||
                     (d.start_time != -1 && STAGE_FULL_TIME - _time_remaining < d.start_time) ||
                     (d.end_time != -1 && STAGE_FULL_TIME - _time_remaining >= d.end_time) ||
                     (d.minHP != -1 && _hp < d.minHP) ||
                     (d.maxHP != -1 && _hp >= d.maxHP) ||
                     (d.outpost_minHP != -1 && _oupost_hp_remaining < d.outpost_minHP));
        }

        bool checkMissionsComplete()
        {
            std::lock_guard<std::mutex> lck(mutex);
            return _missions.empty();
        }

        bool checkWaitingComplete()
        {
            std::lock_guard<std::mutex> lck(mutex);
            bool check = wait_timeout_time - current_time < 1e-6;
            if (_missions.front().name == MISSION_TYPE_WAIT)
            {
                if (check)
                    _missions.front().status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
                else
                    _missions.front().status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
            }
            return check;
        }

        void cleanUpMissions()
        {
            std::lock_guard<std::mutex> lck(mutex);
            while (_missions.size() > 0)
            {
                _missions.pop();
            }
            wait_timeout_time = 0;
        }

        void setMissionStatus(int _status)
        {
            std::lock_guard<std::mutex> lck(mutex);
            _missions.front().status = _status;
        }

        Mission &getMission()
        {
            std::lock_guard<std::mutex> lck(mutex);
            return _missions.front();
        }

        void popMission()
        {
            std::lock_guard<std::mutex> lck(mutex);
            _missions.pop();
        }

        bool isMissionEmpty()
        {
            std::lock_guard<std::mutex> lck(mutex);
            return _missions.empty();
        }

        void insertMission(Mission &m)
        {
            std::lock_guard<std::mutex> lck(mutex);
            _missions.emplace(m);
        }

        void setWaitTime(double sec)
        {
            std::lock_guard<std::mutex> lck(mutex);
            wait_timeout_time = current_time + sec;
        }

        double getWaitTimeoutTime()
        {
            std::lock_guard<std::mutex> lck(mutex);
            return wait_timeout_time;
        }

        void getPose(Eigen::Vector2d &pos, double &w)
        {
            std::lock_guard<std::mutex> lck(mutex);
            pos = _pos;
            w = theta;
        }
    };
} // namespace sentry

#endif // _BLACKBOARD_H