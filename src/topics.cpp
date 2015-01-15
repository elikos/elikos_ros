/*
 * topics.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: Myriam Claveau-Mallet
 *      Brief: This file contains all topics definitions and functions to create publishers
 *             for these topics.
 */

#include <ros/ros.h>
#include <vector>
#include <string>

///
/// Define topics to avoid confusion in topics denomination
///
enum topics
{
    pos_robot_1 = 0,
    pos_robot_2,
    pos_robot_3,
    pos_robot_4,
    pos_robot_5,
    pos_robot_6,
    pos_robot_7,
    pos_robot_8,
    pos_robot_9,
    pos_robot_10,
    NB_TOPICS
};

///
/// Provide access to enum names
///
static const std::vector<sdt::string> TOPICS_NAMES(
    {
        pos_robot_1,
        pos_robot_2,
        pos_robot_3,
        pos_robot_4,
        pos_robot_5,
        pos_robot_6,
        pos_robot_7,
        pos_robot_8,
        pos_robot_9,
        pos_robot_10
    }
);