#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>
#include "Obstacle.h"

const std::string SERVICE_NAME = "/gazebo/set_model_state";

int main(int argc, char* argv[])
{
  ros::init( argc, argv, "robots_sim" );

  ros::NodeHandle n;

  int updateRate = 30;

  ros::ServiceClient client;
  client = n.serviceClient<gazebo_msgs::SetModelState>(SERVICE_NAME);

  Robot target1 = Robot("target1", 0, 2, 0.25, updateRate, client);
  Robot target2 = Robot("target2", 1, 1.73, 0.25, updateRate, client);
  Robot target3 = Robot("target3", 1.73, 1, 0.25, updateRate, client);
  Robot target4 = Robot("target4", 1.73, -1, 0.25, updateRate, client);
  Robot target5 = Robot("target5", 1, -1.73, 0.25, updateRate, client);
  Robot target6 = Robot("target6", 0, -2, 0.25, updateRate, client);
  Robot target7 = Robot("target7", -1, -1.73, 0.25, updateRate, client);
  Robot target8 = Robot("target8", -1.73, -1, 0.25, updateRate, client);
  Robot target9 = Robot("target9", -1.73, 1, 0.25, updateRate, client);
  Robot target10 = Robot("target10", -1, 1.73, 0.25, updateRate, client);

  Obstacle obstacle1 = Obstacle("obstacle1", 0, 5, 0.25, updateRate, client);
  Obstacle obstacle2 = Obstacle("obstacle2", 5, 0, 0.25, updateRate, client);
  Obstacle obstacle3 = Obstacle("obstacle3", 0, -5, 0.25, updateRate, client);
  Obstacle obstacle4 = Obstacle("obstacle4", -5, 0, 0.25, updateRate, client);

  srand(time(NULL));

  ros::Rate r(updateRate);
  while(ros::ok())
  {
    target1.move();
    target2.move();
    target3.move();
    target4.move();
    target5.move();
    target6.move();
    target7.move();
    target8.move();
    target9.move();
    target10.move();

    obstacle1.move();
    obstacle2.move();
    obstacle3.move();
    obstacle4.move();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}