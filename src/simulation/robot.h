#ifndef __ROBOT_H_
#define __ROBOT_H_

class Robot{
public:
    Robot(int id);
    ~Robot();

    std::string getName();
    int getID();
    tf::Transform

    void collide();
    void move(ros::Duration cycleTime);
private:
    int id;
    std::string name;
    tf::Transform transform;


};

#endif /* __GROUNDROBOT_H_ */