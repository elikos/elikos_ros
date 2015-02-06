/*
 * defines.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: Myriam Claveau-Mallet
 *      Brief: This file contains all topics definitions and functions to create publishers
 *             for these topics, and other static or const variables and enum.
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
    robotsim_robot_markers,
    robotsim_mav_marker,
    robotsim_setpoint_marker,
    robotsim_arena_marker,
    mavros_setpoint_local_position,
    NB_TOPICS
};

///
/// Provide access to enum names
///

//const string arr[] = {16,2,77,29};
//vector<int> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );

//static const std::vector<std::string> TOPICS_NAMES =
const std::string arr[] = // TOPICS_NAMES
{
	"pos_robot_1",
	"pos_robot_2",
	"pos_robot_3",
	"pos_robot_4",
	"pos_robot_5",
	"pos_robot_6",
	"pos_robot_7",
	"pos_robot_8",
	"pos_robot_9",
	"pos_robot_10",
	"robotsim/robot_markers",
	"robotsim/mav_marker",
	"robotsim/setpoint_marker",
	"robotsim/arena_marker",
	"mavros/setpoint/local_position"
};
static const std::vector<std::string> TOPICS_NAMES
(
    arr, arr + sizeof(arr) / sizeof(arr[0])
);


