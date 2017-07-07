#ifndef ROBOT_H
#define ROBOT_H

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

class Robot {
   public:
    Robot(int id, uint8_t color);
    Robot();
    ~Robot();

    /* void setPos(geometry_msgs::PoseStamped pose);
     geometry_msgs::PoseStamped getPos();

     void setFcu(geometry_msgs::PoseStamped pose);
     geometry_msgs::PoseStamped getFcu();
     */
    void setPos(geometry_msgs::Point point);
    geometry_msgs::Point getPos();

    void setColor(uint8_t color);
    uint8_t getColor();

    void setId(uint8_t id) { id = this->id; }
    int getId() { return id; }

    void setTime(ros::Time time);
    ros::Time getTime();

    void setIncertitude(double incertitude);
    double getIncertitude();

    void setSpeed(double speed);
    double getSpeed();

    void setAssigned(bool state) { isAssigned = state; }
    bool getAssigned() { return isAssigned; }

    double updateIncertitude(int dt);

    double getDistanceFrom(geometry_msgs::Point pos);

    bool isNew;

   private:
    // geometry_msgs::PoseStamped poseOrigin;
    // geometry_msgs::PoseStamped fcu;

    geometry_msgs::Point poseOrigin;
    int id;
    uint8_t color;
    ros::Time time;

    // Indique si le robot a déja ete assigne pour literation courante
    bool isAssigned;
    // en m
    double incertitude;

    // en m/s
    double speed;
};

#endif
