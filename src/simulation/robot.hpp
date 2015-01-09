#ifndef __ROBOT_H_
#define __ROBOT_H_

/**
* Abstract base class representing robots in the arena (obstacles and targets)
*/
class Robot{
public:
    Robot(int id);
    virtual ~Robot();

    std::string getName() { return this->name; }
    int getID() { return this->id; }
    tf::Transform getTransform() { return this->transform; }
    virtual visualization_msgs::Marker getVizMarker() = 0;

    virtual void collide() = 0;
    virtual void move(ros::Duration cycleTime) = 0;
private:
    int id;
    std::string name;
    tf::Transform transform;


};

#endif /* __GROUNDROBOT_H_ */