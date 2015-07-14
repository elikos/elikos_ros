#include <geometry_msgs/PoseStamped.h>
#include "Detection.h"

namespace elikos_detection
{
    Detection::Detection( ros::NodeHandle *nh ) : nh_(nh),it_(*nh)
    {
        // Load parameters
        nh->param<int>("h_min", H_MIN, 0);
        nh->param<int>("h_max", H_MAX, 256);
        nh->param<int>("s_min", S_MIN, 0);
        nh->param<int>("s_max", S_MAX, 256);
        nh->param<int>("v_min", V_MIN, 0);
        nh->param<int>("v_max", V_MAX, 256);
    }

    Detection::~Detection()
    {
    }

    void Detection::init()
    {
        setPublishers();
        setSubscribers();
        initCameraTF();
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
        sprintf( TrackbarName, "H_MIN");
        sprintf( TrackbarName, "H_MAX");
        sprintf( TrackbarName, "S_MIN");
        sprintf( TrackbarName, "S_MAX");
        sprintf( TrackbarName, "V_MIN");
        sprintf( TrackbarName, "V_MAX");
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
        currentImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }

    void Detection::setPublishers()
    {
        //TODO : publish robot info
        if(nh_)
        {
            std::string topicName = TOPIC_NAMES[robotsPos];
            robots_publish = nh_->advertise<elikos_ros::RobotsPos>(topicName,1);

        }
    }

    void Detection::trackFilteredObjects(Mat threshold,Mat HSV, Mat &cameraFeed)
    {

        //int x,y;
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

                vecRobot.clear();

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
        trackFilteredObjects(threshold,hsv,currentImage);
    }

    void Detection::showThreshold()
    {
        imshow(windowName2,threshold);
    }

    void Detection::setSubscribers()
    {
        image_sub_ = it_.subscribe(TOPIC_NAMES[camera_image_raw], 1, &Detection::cameraCallback, this);
    }

    void Detection::drawObject(vector<RobotDesc> vecRobot,Mat &frame){

        for(int i = 0; i<vecRobot.size(); i++)
        {
            cv::circle(frame, cv::Point(vecRobot.at(i).getHPos(), vecRobot.at(i).getVPos()), 10, cv::Scalar(0, 0, 255));
            cv::putText(frame, intToString(vecRobot.at(i).getHPos()) + " , " + intToString(vecRobot.at(i).getVPos()), cv::Point(vecRobot.at(i).getHPos(), vecRobot.at(i).getVPos() + 20), 1, 1, Scalar(0, 255, 0));
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

    void Detection::cannyEdge()
    {
        int edgeThresh = 1;
        int lowThreshold;
        int const max_lowThreshold = 100;
        int ratio = 3;
        int kernel_size = 3;
        Mat cannyEdges, blur;
        //char* window_name = "Edge Map";

        cannyEdges.create( threshold.size(), threshold.type() );
        blur.create(threshold.size(), threshold.type() );

        //GaussianBlur( threshold, blur, Size(3,3),2,2);

        Canny( blur, cannyEdges, lowThreshold, lowThreshold*ratio, kernel_size );

        imshow("Edge Map", cannyEdges);

    }
    void Detection::sendMsg()
    {
        robots_publish.publish(robotsPos_msg);
        //ROS_INFO_STREAM("I am advertising a robotInfo vector");
        vecRobot.clear();
    }


    void Detection::initCameraTF() {
        // Set up the camera's position relative to the fcu
        camera_.setOrigin(tf::Vector3(-0.10, 0, -0.05));
        camera_.setRotation(tf::Quaternion(tf::Vector3(0, 1, 0), PI/2));
    }

    void Detection::computeTargetPosition() {
        // Append the camera frame to the fcu
        tf_broadcaster_.sendTransform(tf::StampedTransform(camera_, ros::Time::now(), "fcu", "camera"));

        // Set yaw and pitch of the target wrt the camera frame
        try {
            getRotationFromImage(turret_rotation_);
        }
        catch (std::out_of_range ex) {
            static uint empty_robot_vector_ex_count = 0;
            empty_robot_vector_ex_count++;
            if (empty_robot_vector_ex_count % 30 == 0) {
                ROS_ERROR("%s", ex.what());
            }
            return;
        }
        turret_.setOrigin(tf::Vector3(0, 0, 0));
        turret_.setRotation(turret_rotation_);

        // Broadcast the turret frame
        tf_broadcaster_.sendTransform(tf::StampedTransform(turret_, ros::Time::now(),"camera", "turret"));

        // Get the world to turret transform
        try {
            tf_listener_.lookupTransform("local_origin", "turret", ros::Time(0), turret_world_);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            return;
        }

        // Get the smallest angle between the turret and the z axis
        //      - First get the vector pointing towards the x axis of the turret
        turret_world_x_ = tf::quatRotate(turret_world_.getRotation(), tf::Vector3(1, 0, 0));

        //      - Then find it's angle with the camera's resting position (x pointing straight down (-z))
        tf::Vector3 zAxis(0, 0, -1);
        double zAxis_turret_angle = zAxis.angle(turret_world_x_);

        // Get distance from turret to target (using angle and altitude)
        double camera_altitude = turret_world_.getOrigin().getZ();
        double distance_from_target = camera_altitude / cos(zAxis_turret_angle);

        // Add the robot transform as child of the turret
        target_robot_.setOrigin(tf::Vector3(distance_from_target, 0, 0));
        target_robot_.setRotation(tf::Quaternion(0, 0, 0, 1));

        tf_broadcaster_.sendTransform(tf::StampedTransform(target_robot_, ros::Time::now(), "turret", "target_robot"));

    }

    void Detection::getRotationFromImage(tf::Quaternion &q) {
        if (vecRobot.empty()) {
            throw std::out_of_range("No object is currently detected.");
        }

        // Set pitch - y axis (image vertical)
        double pitch = ((double)(vecRobot[0].getVPos() - CAM_HEIGHT / 2) / (double)CAM_HEIGHT) * CAMERA_FOV_V;

        // Set yaw - z axis (image horizontal)
        double yaw = -((double)(vecRobot[0].getHPos() - CAM_WIDTH / 2) / (double)CAM_WIDTH) * CAMERA_FOV_H;

        // Set roll, pitch and yaw
        q.setRPY(0, pitch, yaw);
    }
}
