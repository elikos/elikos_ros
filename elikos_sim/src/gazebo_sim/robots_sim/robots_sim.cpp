#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>
#include "Obstacle.h"
#include <vector>

const std::string SERVICE_NAME = "/gazebo/set_model_state";

int main(int argc, char* argv[])
{
  ros::init( argc, argv, "robots_sim" );

  ros::NodeHandle n;

  int updateRate = 30;

  ros::ServiceClient client;
  client = n.serviceClient<gazebo_msgs::SetModelState>(SERVICE_NAME);

  int speed;
  n.getParam("/"+ros::this_node::getName()+"/speed", speed);

  std::vector<Robot> targets;

  bool red_1;
  n.getParam("/"+ros::this_node::getName()+"/red_1", red_1);
  if(red_1)
  {
    Robot target1 = Robot("target1", 0, 2, speed, updateRate, client);
    targets.emplace_back(target1);
  } 
  
  bool green_1;
  n.getParam("/"+ros::this_node::getName()+"/green_1", green_1);
  if(green_1)
  {
    Robot target2 = Robot("target2", 1, 1.73, speed, updateRate, client);
    targets.emplace_back(target2);
  } 
  
  bool red_2;
  n.getParam("/"+ros::this_node::getName()+"/red_2", red_2);
  if(red_2)
  {
    Robot target3 = Robot("target3", 1.73, 1, speed, updateRate, client);
    targets.emplace_back(target3);
  } 
  
  bool green_2;
  n.getParam("/"+ros::this_node::getName()+"/green_2", green_2);
  if(green_2)
  {
    Robot target4 = Robot("target4", 1.73, -1, speed, updateRate, client);
    targets.emplace_back(target4);
  } 
  
  bool red_3;
  n.getParam("/"+ros::this_node::getName()+"/red_3", red_3);
  if(red_3)
  {
    Robot target5 = Robot("target5", 1, -1.73, speed, updateRate, client);
    targets.emplace_back(target5);
  } 
  
  bool green_3;
  n.getParam("/"+ros::this_node::getName()+"/green_3", green_3);
  if(green_3)
  {
    Robot target6 = Robot("target6", 0, -2, speed, updateRate, client);
    targets.emplace_back(target6);
  } 
  
  bool red_4;
  n.getParam("/"+ros::this_node::getName()+"/red_4", red_4);
  if(red_4)
  {
    Robot target7 = Robot("target7", -1, -1.73, speed, updateRate, client);
    targets.emplace_back(target7);
  } 
  
  bool green_4;
  n.getParam("/"+ros::this_node::getName()+"/green_4", green_4);
  if(green_4)
  {
    Robot target8 = Robot("target8", -1.73, -1, speed, updateRate, client);
    targets.emplace_back(target8);
  } 
  
  bool red_5;
  n.getParam("/"+ros::this_node::getName()+"/red_5", red_5);
  if(red_5)
  {
    Robot target9 = Robot("target9", -1.73, 1, speed, updateRate, client);
    targets.emplace_back(target9);
  } 
  
  bool green_5;
  n.getParam("/"+ros::this_node::getName()+"/green_5", green_5);
  if(green_5)
  {
    Robot target10 = Robot("target10", -1, 1.73, speed, updateRate, client);
    targets.emplace_back(target10);
  } 

  std::vector<Obstacle> obstacles;

  bool obstacle_05;
  n.getParam("/"+ros::this_node::getName()+"/obstacle_05", obstacle_05);
  if(obstacle_05)
  {
    Obstacle obstacle1 = Obstacle("obstacle1", 0, 5, speed, updateRate, client);
    obstacles.emplace_back(obstacle1);
  }
  bool obstacle_10;
  n.getParam("/"+ros::this_node::getName()+"/obstacle_10", obstacle_10);
  if(obstacle_10)
  {
    Obstacle obstacle2 = Obstacle("obstacle2", 5, 0, speed, updateRate, client);
    obstacles.emplace_back(obstacle2);
  }
  bool obstacle_15;
  n.getParam("/"+ros::this_node::getName()+"/obstacle_15", obstacle_15);
  if(obstacle_15)
  {
    Obstacle obstacle3 = Obstacle("obstacle3", 0, -5, speed, updateRate, client);
    obstacles.emplace_back(obstacle3);
  }
  bool obstacle_20;
  n.getParam("/"+ros::this_node::getName()+"/obstacle_20", obstacle_20);
  if(obstacle_20)
  {
    Obstacle obstacle4 = Obstacle("obstacle4", -5, 0, speed, updateRate, client);
    obstacles.emplace_back(obstacle4);
  }

  srand(time(NULL));

  ros::Rate r(updateRate);
  while(ros::ok())
  {
    for( auto &target: targets)
    {
      target.move();
    }

    for( auto &obstacle: obstacles)
    {
      obstacle.move();
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}