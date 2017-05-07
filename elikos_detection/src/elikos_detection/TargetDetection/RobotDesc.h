#ifndef DETECT_ROBOTDESC_H
#define DETECT_ROBOTDESC_H
#define PI   3.1415926535
#include <opencv2/core/core.hpp>
#include "string"


using namespace std;
enum ColorsIndex {WHITE, GREEN, RED};

class RobotDesc {
public:
    //Constructors and destructor
    RobotDesc();

    RobotDesc(int, int, int);

    ~RobotDesc();

    //Setter and getters
    int getID() const;

    void setID(int id);

    double getDirection() const;

    void setDirection(double direction);

    //The window is the area where we can find the robot on the frame
    cv::RotatedRect getWindow() const;

    void setWindow(cv::RotatedRect);

    int getHPos();

    int getVPos();

    void setXPos(int pos);

    void setYPos(int pos);

    void setRadius(int radius);

    void setColor(ColorsIndex);

    int getXPos() const;

    int getYPos() const;

    int getRadius() const;

    ColorsIndex getColor() const;

    cv::Scalar getHSVmin();

    cv::Scalar getHSVmax();

    void setHSVmin(cv::Scalar scalar);

    void setHSVmax(cv::Scalar scalar);

    double getArea();

    void setArea(double area);

    double getDistance(){return distancePrevious;}
    void setDistance(double d){distancePrevious=d;}

    bool getAlreadyFound(){return alreadyFound;}
    void setAlreadyFound(bool a){alreadyFound=a;}

    bool getInsideFOV(){return insideFOV;}
    void setInsideFOV(bool in){insideFOV=in;}

    bool operator==(const RobotDesc& otherDesc) const;
private:
    /////Attributes/////
    //Inside fov(field of view)
    bool insideFOV;
    // center
    int xPos, yPos;
    int radius;
    double area;
    //direction is an angle in radians
    double direction;
    //distance from the previous position on frame. TODO: Could be interesting to choose which robot to keep when there are many with the same ID.
    double distancePrevious;
    //to insure that there is only one robot by id
    bool alreadyFound;
    int id;//Used to compare two RobotDesc
    //window is the area on the frame where the robot was found
    cv::RotatedRect window;
    ColorsIndex color;
    string type;
    cv::Scalar HSVmin, HSVmax;
};
#endif


