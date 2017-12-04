#ifndef ELIKOS_SIM_BOUNDEDPID_H
#define ELIKOS_SIM_BOUNDEDPID_H

#include <ros/ros.h>
#include <control_toolbox/pid.h>


class BoundedPID : public control_toolbox::Pid
{
    public:
        BoundedPID(double minCmd, double maxCmd, double p, double i, double d, double i_max, double i_min, bool antiwindup);
        ~BoundedPID();

        double computeCommand(double error, ros::Duration dt);

    private:
        double minCmd_;
        double maxCmd_;

};

#endif // ELIKOS_SIM_BOUNDEDPID_H