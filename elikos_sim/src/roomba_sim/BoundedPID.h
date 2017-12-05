#ifndef ELIKOS_SIM_BOUNDEDPID_H
#define ELIKOS_SIM_BOUNDEDPID_H

/**
 * \file BoundedPID.h
 * \brief BoundedPID class declaration
 * \author christophebedard
 */

#include <ros/ros.h>
#include <control_toolbox/pid.h>

/** \class BoundedPID
 * \brief class which represents a bounded (min/max) PID.
 */
class BoundedPID : public control_toolbox::Pid
{
    public:
        /**
         * \brief BoundedPID constructor.
         *
         * \param minCmd : minimum value for command.
         * \param maxCmd : maximum value for command.
         * \param p : proportional gain.
         * \param i : integral gain.
         * \param d : derivative gain.
         * \param i_max : max integral windup.
         * \param i_min : min integral windup.
         * \param antiwindup : (see documentation for control_toolbox::Pid).
         */
        BoundedPID(double minCmd, double maxCmd, double p, double i, double d, double i_max, double i_min, bool antiwindup);
        
        /**
         * \brief BoundedPID destructor.
         */
        ~BoundedPID();

        /**
         * \brief BoundedPID constructor.
         *
         * \param error : error since last call (error = target - current).
         * \param dt : change in time since last call.
         *
         * \return value for next command
         */
        double computeCommand(double error, ros::Duration dt);

    private:
        double minCmd_; /**< minimum command */
        double maxCmd_; /**< maximum command */

};

#endif // ELIKOS_SIM_BOUNDEDPID_H