//
// Created by olivier on 16/07/16.
//

#include "Timer.h"

namespace util
{

void Timer::start()
{
    startTime_ = ros::Time::now();
    isPaused_ = false;
}

void Timer::pause()
{
    pauseTime_ = ros::Time::now();
    isPaused_ = true;
}

void Timer::reset()
{
    startTime_ = ros::Time::now();
    pauseTime_ = ros::Time::now();
}

ros::Duration Timer::getDuration()
{
    ros::Time now;
    if (!isPaused_) {
        now = ros::Time::now();
    } else {
        now = pauseTime_;
    }
    return now - startTime_;
}

double Timer::getElapsedS()
{
    ros::Duration test = getDuration();
    double test2 = test.toSec();
    return test2;
    //getDuration().toSec();
}

double Timer::getElapsedMS()
{
    return getElapsedS() / 1000.0;
}

}

