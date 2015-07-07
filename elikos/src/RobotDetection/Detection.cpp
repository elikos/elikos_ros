#include <geometry_msgs/PoseStamped.h>
#include "Detection.h"

namespace elikos_detection
{
    Detection::Detection( ros::NodeHandle *nh ) : nh_(nh),it_(*nh)
    {
        // Load parameters
        nh->param<int>("pre_erosions", PRE_EROSIONS, 5);
        nh->param<int>("dilations", DILATIONS, 5);
        nh->param<int>("post_erosions", POST_EROSIONS, 5);
       /* nh->param<int>("h_min_w", H_MIN_W, 0);
        nh->param<int>("h_max_w", H_MAX_W, 256);
        nh->param<int>("s_min_w", S_MIN_W, 0);
        nh->param<int>("s_max_w", S_MAX_W, 256);
        nh->param<int>("v_min_w", V_MIN_W, 0);
        nh->param<int>("v_max_w", V_MAX_W, 256);
        nh->param<int>("h_min_c", H_MIN_C, 0);
        nh->param<int>("h_max_c", H_MAX_C, 256);
        nh->param<int>("s_min_c", S_MIN_C, 0);
        nh->param<int>("s_max_c", S_MAX_C, 256);
        nh->param<int>("v_min_c", V_MIN_C, 0);
        nh->param<int>("v_max_c", V_MAX_C, 256);*/

        nh->param<int>("h_min_w", H_MIN_W, 0);
        nh->param<int>("h_max_w", H_MAX_W, 52);
        nh->param<int>("s_min_w", S_MIN_W, 0);
        nh->param<int>("s_max_w", S_MAX_W, 149);
        nh->param<int>("v_min_w", V_MIN_W, 140);
        nh->param<int>("v_max_w", V_MAX_W, 256);
        nh->param<int>("h_min_c", H_MIN_C, 39);
        nh->param<int>("h_max_c", H_MAX_C, 248);
        nh->param<int>("s_min_c", S_MIN_C, 50);
        nh->param<int>("s_max_c", S_MAX_C, 256);
        nh->param<int>("v_min_c", V_MIN_C, 30);
        nh->param<int>("v_max_c", V_MAX_C, 256);
        nh->param<int>("pre_blur", PRE_BLUR, 2);
        nh->param<int>("canny_thresh1", CANNY_THRESH1, 1);
        nh->param<int>("canny_thresh1", CANNY_THRESH2, 1);
        nh->param<int>("canny_aperture", CANNY_APERTURE, 3);
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
        namedWindow(shapeDetectTrackbars,0);
        //create trackbars and insert them into window
        //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
        //the max value the trackbar can move (eg. H_HIGH),
        //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
        //                                  ---->    ---->     ---->
        createTrackbar( "PRE EROSIONS", trackbarWindowName, &PRE_EROSIONS, 50, on_trackbar );
        createTrackbar( "DILATIONS", trackbarWindowName, &DILATIONS, 50, on_trackbar );
        createTrackbar( "POST EROSIONS", trackbarWindowName, &POST_EROSIONS, 50, on_trackbar );
        createTrackbar( "H_MIN W", trackbarWindowName, &H_MIN_W, 256, on_trackbar );
        createTrackbar( "H_MAX W", trackbarWindowName, &H_MAX_W, 256, on_trackbar );
        createTrackbar( "S_MIN W", trackbarWindowName, &S_MIN_W, 256, on_trackbar );
        createTrackbar( "S_MAX W", trackbarWindowName, &S_MAX_W, 256, on_trackbar );
        createTrackbar( "V_MIN W", trackbarWindowName, &V_MIN_W, 256, on_trackbar );
        createTrackbar( "V_MAX W", trackbarWindowName, &V_MAX_W, 256, on_trackbar );
        createTrackbar( "H_MIN C", trackbarWindowName, &H_MIN_C, 256, on_trackbar );
        createTrackbar( "H_MAX C", trackbarWindowName, &H_MAX_C, 256, on_trackbar );
        createTrackbar( "S_MIN C", trackbarWindowName, &S_MIN_C, 256, on_trackbar );
        createTrackbar( "S_MAX C", trackbarWindowName, &S_MAX_C, 256, on_trackbar );
        createTrackbar( "V_MIN C", trackbarWindowName, &V_MIN_C, 256, on_trackbar );
        createTrackbar( "V_MAX C", trackbarWindowName, &V_MAX_C, 256, on_trackbar );

        // Create shape detector trackbars
        createTrackbar("Pre Blur", shapeDetectTrackbars, &PRE_BLUR, 50, on_trackbar);
        createTrackbar("Canny Thresh 1", shapeDetectTrackbars, &CANNY_THRESH1, 256, on_trackbar);
        createTrackbar("Canny Thresh 2", shapeDetectTrackbars, &CANNY_THRESH2, 256, on_trackbar);
        createTrackbar("Canny Aperture", shapeDetectTrackbars, &CANNY_APERTURE, 5, on_trackbar);
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


    void Detection::trackBlobs()
    {
        cvtColor(currentImage,hsv,COLOR_BGR2HSV);

        // Find the white in the image (the robot plastic casing)
        inRange(hsv,Scalar(H_MIN_W,S_MIN_W,V_MIN_W),Scalar(H_MAX_W,S_MAX_W,V_MAX_W), threshold_w);

        // Consolidate the white parts into one big blob to delimit the robot
        erode(threshold_w, threshold_w, getStructuringElement(MORPH_ELLIPSE,Size(3,3)), Point(-1,-1), PRE_EROSIONS);
        dilate(threshold_w, threshold_w, getStructuringElement(MORPH_ELLIPSE,Size(3,3)), Point(-1,-1), DILATIONS);
        erode(threshold_w, threshold_w, getStructuringElement(MORPH_ELLIPSE,Size(3,3)), Point(-1,-1), POST_EROSIONS);

        // Crop the rest of the image for color blob filtering
        cvtColor(threshold_w, threshold_w, CV_GRAY2BGR);
        bitwise_and(hsv, threshold_w, cropped_hsv);


        // Filter the cropped image
        inRange(cropped_hsv,Scalar(H_MIN_C,S_MIN_C,V_MIN_C),Scalar(H_MAX_C,S_MAX_C,V_MAX_C), threshold_c);
        morphOps(threshold_c);
        trackFilteredObjects(threshold_c,cropped_hsv,currentImage);
    }

    void Detection::trackShape()
    {
        cvtColor(currentImage, grayscale_image, COLOR_BGR2GRAY);
        BLUR_AMOUNT = PRE_BLUR + 1;
        blur(grayscale_image, grayscale_image, Size(BLUR_AMOUNT, BLUR_AMOUNT), Point(-1,-1));
        Canny(grayscale_image, canny, CANNY_THRESH1, CANNY_THRESH2);

        std::vector<std::vector<Point>> contours;
        findContours(canny, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

        contour_drawings = Mat::zeros(canny.size(), CV_8UC3);
        for (int i = 0; i < contours.size(); i++) {
            drawContours(contour_drawings, contours, i, Scalar(255,255,255), 2, 8);
        }

    }

    void Detection::showThreshold()
    {
        //imshow("White threshold", threshold_w);
        //imshow("Cropped image", cropped_hsv);
        //imshow("Color threshold", threshold_c);
        imshow("Blurred", grayscale_image);
        imshow("Canny Edges", canny);
        imshow("Contours", contour_drawings);
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

        cannyEdges.create( threshold_c.size(), threshold_c.type() );
        blur.create(threshold_c.size(), threshold_c.type() );

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
