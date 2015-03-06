#include "Detection.h"

namespace elikos_detection
{
    Detection::Detection( ros::NodeHandle *nh ) : nh_(nh),it_(*nh)
    {
        H_MIN = 0;
        H_MAX = 256;
        S_MIN = 0;
        S_MAX = 256;
        V_MIN = 0;
        V_MAX = 256;
    }

    Detection::~Detection()
    {
    }

    void Detection::init()
    {
        setPublishers();
        setSubscribers();
    }

    string Detection::intToString(int number)
    {
        std::stringstream ss;
        ss << number;
        return ss.str();
    }

    void on_trackbar( int, void* )
    {//This function gets called whenever a
        // trackbar position is changed
    }

    void Detection::setupDebug()
    {
        //TODO : Change the hardcoding on the camera number
        try
        {
            capture.open(0);
        }
        catch (int e)
        {
            printf("No cameras detected");
        }

        capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
    }

    void Detection::captureFrame()
    {
        //store image to matrix
        capture.read(currentImage);
        //convert frame from BGR to HSV colorspace
        cvtColor(currentImage,hsv,COLOR_BGR2HSV);
        //printf("I captured an image with my webcam");
    }

    void Detection::createTrackbars()
    {
        //create window for trackbars
        namedWindow(trackbarWindowName,0);
        //create memory to store trackbar name on window
        char TrackbarName[50];
        sprintf( TrackbarName, "H_MIN", H_MIN);
        sprintf( TrackbarName, "H_MAX", H_MAX);
        sprintf( TrackbarName, "S_MIN", S_MIN);
        sprintf( TrackbarName, "S_MAX", S_MAX);
        sprintf( TrackbarName, "V_MIN", V_MIN);
        sprintf( TrackbarName, "V_MAX", V_MAX);
        //create trackbars and insert them into window
        //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
        //the max value the trackbar can move (eg. H_HIGH),
        //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
        //                                  ---->    ---->     ---->
        createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
        createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
        createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
        createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
        createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
        createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
    }


    void Detection::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        nextImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    void Detection::setPublishers()
    {
        //TODO : publish robot info
        if(nh_)
        {
            std::string topicName = TOPICS_NAMES[robotsPos];
            robots_publish = nh_->advertise<elikos_ros::RobotsPos>(topicName,1);

        }
    }

    void Detection::trackFilteredObjects(Mat threshold,Mat HSV, Mat &cameraFeed)
    {

        //int x,y;

        vector<RobotDesc> vecRobot;
        RobotDesc myRobot;

        Mat temp;
        threshold.copyTo(temp);
        //these two vectors needed for output of findContours
        vector< vector<Point> > contours;
        vector<Vec4i> hierarchy;
        //find contours of filtered image using openCV findContours function
        findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
        //use moments method to find our filtered object
        double refArea = 0;
        bool objectFound = false;
        if (hierarchy.size() > 0) {
            int numObjects = hierarchy.size();
            //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
            if(numObjects<MAX_NUM_OBJECTS){
                for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                    Moments moment = moments((cv::Mat)contours[index]);
                    double area = moment.m00;

                    //if the area is less than 20 px by 20px then it is probably just noise
                    //if the area is the same as the 3/2 of the image size, probably just a bad filter
                    //we only want the object with the largest area so we safe a reference area each
                    //iteration and compare it to the area in the next iteration.
                    if(area>MIN_OBJECT_AREA){

                        myRobot.setXPos(moment.m10/area);
                        myRobot.setYPos(moment.m01/area);

                        vecRobot.push_back(myRobot);

                        objectFound = true;

                    }else objectFound = false;


                }
                //let user know you found an object
                if(objectFound ==true) {
                    //draw object location on screen
                    drawObject(vecRobot, cameraFeed);
                    if (vecRobot.size() != 0)
                    {
                        for (int i = 0; i < vecRobot.size(); i++)
                        {
                            elikos_ros::RobotPos pos = vecRobot.at(i).toMsg();
                            robotsPos_msg.robotsPos.push_back(pos);
                        }
                        robots_publish.publish(robotsPos_msg);
                        ROS_INFO_STREAM("I am advertising a robotInfo vector");
                    }
                }

            }else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
        }
    }


    void Detection::trackRobots()
    {
        cvtColor(currentImage,hsv,COLOR_BGR2HSV);
        inRange(hsv,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
        morphOps(threshold);
        imshow(windowName2,threshold);
        trackFilteredObjects(threshold,hsv,currentImage);
    }

    void Detection::setSubscribers()
    {
        //TODO : subscribe to mavros (drone position)
        //TODO : subscribe to camera feed
        std::string robotsPosTopic = TOPICS_NAMES[camera_image_raw];
        image_sub_ = it_.subscribe(robotsPosTopic, 1, &Detection::cameraCallback, this);

    }

    void Detection::drawObject(vector<RobotDesc> vecRobot,Mat &frame){

        for(int i = 0; i<vecRobot.size(); i++)
        {
            cv::circle(frame, cv::Point(vecRobot.at(i).getXPos(), vecRobot.at(i).getYPos()), 10, cv::Scalar(0, 0, 255));
            cv::putText(frame, intToString(vecRobot.at(i).getXPos()) + " , " + intToString(vecRobot.at(i).getYPos()), cv::Point(vecRobot.at(i).getXPos(), vecRobot.at(i).getYPos() + 20), 1, 1, Scalar(0, 255, 0));
        }
    }

    void Detection::morphOps(Mat &thresh)
    {

        //create structuring element that will be used to "dilate" and "erode" image.
        //the element chosen here is a 3px by 3px rectangle

        Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
        //dilate with larger element so make sure object is nicely visible
        Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

        erode(thresh,thresh,erodeElement);
        dilate(thresh,thresh,dilateElement);

        erode(thresh,thresh,erodeElement);
        dilate(thresh,thresh,dilateElement);
    }

    void Detection::showCurrentImage()
    {
        imshow(windowName, currentImage);
    }

}