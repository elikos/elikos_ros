//
// Created by andre on 16/07/15.
// Total ripoff from rpg_vikit
// https://github.com/uzh-rpg/rpg_vikit/blob/master/vikit_ros/include/vikit/params_helper.h

#ifndef elikos_main_PARAMS_HELPER_H
#define elikos_main_PARAMS_HELPER_H

#include <string>
#include <ros/ros.h>

namespace elikos {
    inline
    bool hasParam(const std::string& name)
    {
        return ros::param::has(name);
    }

    template<typename T>
    T getParam(const std::string& name, const T& defaultValue)
    {
        T v;
        if(ros::param::get(name, v))
        {
            ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
            return v;
        }
        else
            ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
        return defaultValue;
    }

    template<typename T>
    T getParam(const std::string& name)
    {
        T v;
        if(ros::param::get(name, v))
        {
            ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
            return v;
        }
        else
            ROS_ERROR_STREAM("Cannot find value for parameter: " << name);
        return T();
    }
} // namespace elikos

#endif //elikos_main_PARAMS_HELPER_H
