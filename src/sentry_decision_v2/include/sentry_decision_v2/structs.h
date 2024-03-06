#ifndef _BLACKBOARD_H
#define _BLACKBOARD_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <queue>
#include <mutex>
#include <map>

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
        std::string name;
        bool status;
        bool if_spin = false;
        Way_Point move_taget;
        double timeout = 10000;
        double time_stay = 1000;
        double starting_time = -1;
    };

    struct Blackboard
    {
        std::mutex mutex;
        int _hp = -1;
        Eigen::Vector2d _pos = Eigen::Vector2d::Zero();
        int _oupost_hp_remaining = -1;
        int _time_remaining = -1;
        int _stage = -1;
        std::map<Mission, int> _missions;
    };
} // namespace sentry

#endif // _BLACKBOARD_H