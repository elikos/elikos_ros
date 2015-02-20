/*
 * defines.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: Myriam Claveau-Mallet
 *      Brief: This file contains all topics definitions and functions to create publishers
 *             for these topics, and other static or const variables and enum. Also are
 *             included commun includes and pre-compiler defines.
 */


#ifndef _DEFINES_CPP_
#define _DEFINES_CPP_


/* *************************************************************************************************
 * ***           INCLUDES
 * *************************************************************************************************
 */


#include <ros/ros.h>
#include <vector>
#include <string>


/* *************************************************************************************************
 * ***           DEFINES
 * *************************************************************************************************
 */

#ifndef PI
#define PI 3.14159265
#endif


/* *************************************************************************************************
 * ***           STATIC VARIABLES
 * *************************************************************************************************
 */


/*
 * VERY IMPORTANT NOTE REGARDING TOPICS
 *
 * For every new topic included in the following lists, one should ABSOLUTLY
 *
 *     (1) include new topic in the enum,
 *     (2) include new topic string in the std::vector TOPICS_NAMES,
 *     (3) make sure both enum and vector entry have the SAME POSITION.
 *
 * This is because each enum value represents an index in TOPICS_NAMES vector.
 */

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
    camera_image_raw,
    NB_TOPICS
};

///
/// Provide access to enum names
///
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
    "camera/image_raw"
};
static const std::vector<std::string> TOPICS_NAMES
(
    arr, arr + sizeof(arr) / sizeof(arr[0])
);


#endif // _DEFINES_CPP_
