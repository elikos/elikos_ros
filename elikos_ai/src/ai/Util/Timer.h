//
// Created by olivier on 16/07/16.
//

#ifndef UTIL_TIMER_H
#define UTIL_TIMER_H

#include <ros/time.h>

namespace util
{

class Timer
{
public:
    Timer() = default;
    ~Timer() = default;

    void start();
    void pause();
    void reset();

    inline bool isStarted() const;
    inline bool isPaused() const;


    ros::Duration getDuration();

    double getElapsedS();
    double getElapsedMS();

private:
    ros::Time startTime_;
    ros::Time pauseTime_;

    bool isPaused_{ true };
};

inline bool Timer::isStarted() const
{
    return !isPaused_;
}

inline bool Timer::isPaused() const
{
    return isPaused_;
}

}

#endif //ELIKOS_AI_TIMER_H
