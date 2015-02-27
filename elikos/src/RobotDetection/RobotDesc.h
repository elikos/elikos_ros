#include <opencv2/core/core.hpp>
#include "string"

using namespace std;

class RobotDesc
{
public:
    RobotDesc();
    ~RobotDesc();

    int getXPos();
    int getYPos();

    void setXPos(int pos);
    void setYPos(int pos);

    cv::Scalar getHSVmin();
    cv::Scalar getHSVmax();

    void setHSVmin(cv::Scalar scalar);
    void setHSVmax(cv::Scalar scalar);

private:
    int xPos, yPos;
    string type;
    cv::Scalar HSVmin, HSVmax;


    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    // TODO: check this out! Copy constructor and operator= get called for class RobotDesc. It's impossible to make these constructor private and not implement them.
    //RobotDesc& operator= (const RobotDesc&);
    //RobotDesc (const RobotDesc&);
};
