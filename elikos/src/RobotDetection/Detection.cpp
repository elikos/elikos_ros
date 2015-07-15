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
        nh->param<int>("poly_area_min", POLY_AREA_MIN, 0);
        nh->param<int>("poly_area_max", POLY_AREA_MAX, 50000);
        nh->param<int>("morph_op", MORPH_OP, 0);
        nh->param<int>("morph_op", MORPH_ELEMENT, 0);
        nh->param<int>("morph_size", MORPH_SIZE, 0);
        nh->param<int>("max_dist", MAX_DIST, 0);

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
        cvtColor(currentImage,hsv_w,COLOR_BGR2HSV);
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
        createTrackbar( "PRE BLUR", trackbarWindowName, &PRE_BLUR, 50, on_trackbar);
        createTrackbar( "PRE EROSIONS", trackbarWindowName, &PRE_EROSIONS, 256, on_trackbar );
        createTrackbar( "DILATIONS", trackbarWindowName, &DILATIONS, 256, on_trackbar );
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
        createTrackbar("Canny Thresh 1", shapeDetectTrackbars, &CANNY_THRESH1, 256, on_trackbar);
        createTrackbar("Canny Thresh 2", shapeDetectTrackbars, &CANNY_THRESH2, 256, on_trackbar);
        createTrackbar("Canny Aperture", shapeDetectTrackbars, &CANNY_APERTURE, 5, on_trackbar);
        createTrackbar("Poly Area Min", shapeDetectTrackbars, &POLY_AREA_MIN, 50000, on_trackbar);
        createTrackbar("Poly Area Max", shapeDetectTrackbars, &POLY_AREA_MAX, 50000, on_trackbar);
        createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", shapeDetectTrackbars, &MORPH_OP, 4, on_trackbar );
        createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", shapeDetectTrackbars, &MORPH_ELEMENT, 2, on_trackbar );
        createTrackbar( "Kernel size:\n 2n +1", shapeDetectTrackbars, &MORPH_SIZE, 21, on_trackbar );
        createTrackbar("Max distance", shapeDetectTrackbars, &MAX_DIST, 50000, on_trackbar);

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

    vector<RobotDesc> Detection::trackFilteredObjects(Mat threshold, Mat &cameraFeed) {

        //int x,y;
        RobotDesc myRobot;
        vector<RobotDesc> vecRobot;
        Mat temp;
        threshold.copyTo(temp);
        //these two vectors needed for output of findContours
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        //find contours of filtered image using openCV findContours function
        findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        //use moments method to find our filtered object
        double refArea = 0;

        if (hierarchy.size() > 0) {
            int numObjects = hierarchy.size();
            //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
            if (numObjects < MAX_NUM_OBJECTS) {

                vecRobot.clear();

                for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                    Moments moment = moments((cv::Mat) contours[index]);
                    double area = moment.m00;

                    //if the area is less than 20 px by 20px then it is probably just noise
                    //if the area is the same as the 3/2 of the image size, probably just a bad filter
                    //we only want the object with the largest area so we safe a reference area each
                    //iteration and compare it to the area in the next iteration.
                    if (area > MIN_OBJECT_AREA) {

                        myRobot.setXPos(moment.m10 / area);
                        myRobot.setYPos(moment.m01 / area);

                        vecRobot.push_back(myRobot);

                    }
                }
            }
            else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
        }
        return vecRobot;
    }

    void Detection::trackBlobs()
    {
        cvtColor(currentImage,hsv_w,COLOR_BGR2HSV);
        cvtColor(currentImage, hsv_c,COLOR_BGR2HSV);


        BLUR_AMOUNT = PRE_BLUR + 1;
        blur(hsv_w, hsv_w, Size(BLUR_AMOUNT, BLUR_AMOUNT), Point(-1,-1));

        blur(hsv_c, hsv_c, Size(BLUR_AMOUNT, BLUR_AMOUNT), Point(-1,-1));

        // Find the white in the image (the robot plastic casing)
        inRange(hsv_w,Scalar(H_MIN_W,S_MIN_W,V_MIN_W),Scalar(H_MAX_W,S_MAX_W,V_MAX_W), threshold_w);

        // Consolidate the white parts into one big blob to delimit the robot
        erode(threshold_w, threshold_w, getStructuringElement(MORPH_ELLIPSE,Size(3,3)), Point(-1,-1), PRE_EROSIONS);
        dilate(threshold_w, threshold_w, getStructuringElement(MORPH_ELLIPSE,Size(3,3)), Point(-1,-1), DILATIONS);
        erode(threshold_w, threshold_w, getStructuringElement(MORPH_ELLIPSE,Size(3,3)), Point(-1,-1), POST_EROSIONS);

        inRange(hsv_c,Scalar(H_MIN_C,S_MIN_C,V_MIN_C),Scalar(H_MAX_C,S_MAX_C,V_MAX_C), threshold_c);

        // Consolidate the white parts into one big blob to delimit the robot
        erode(threshold_c, threshold_c, getStructuringElement(MORPH_ELLIPSE,Size(3,3)), Point(-1,-1), PRE_EROSIONS);
        dilate(threshold_c, threshold_c, getStructuringElement(MORPH_ELLIPSE,Size(3,3)), Point(-1,-1), DILATIONS);
        erode(threshold_c, threshold_c, getStructuringElement(MORPH_ELLIPSE,Size(3,3)), Point(-1,-1), POST_EROSIONS);


        vector<RobotDesc> foundObjects_w = trackFilteredObjects(threshold_w, currentImage);
        vector<RobotDesc> foundObjects_c = trackFilteredObjects(threshold_c, currentImage);
       /* for (int i=0; i<foundObjects_c.size(); i++){
            printf("Found Color Object[%i]:\n", i);
            printf("xPos: %i\n", foundObjects_c[i].getXPos());
            printf("yPos: %i\n", foundObjects_c[i].getYPos());
        }*/
        //drawObject(foundObjects_w, currentImage);
        for (int i=0; i<foundObjects_w.size(); i++){
            printf("Found White Object[%i]:\n", i);
            printf("xPos: %i\n", foundObjects_w[i].getXPos());
            printf("yPos: %i\n", foundObjects_w[i].getYPos());
        }
        //drawObject(foundObjects_c, currentImage);
        foundRobots.clear();
        RobotDesc myRobot;

        bool robotFound=false;
        for (int i=0; i<foundObjects_c.size(); i++){
            for (int j=0; j<foundObjects_w.size(); j++){
                if (abs(foundObjects_c[i].getXPos()-foundObjects_w[j].getXPos())<MAX_DIST&&
                        abs(foundObjects_c[i].getYPos()-foundObjects_w[j].getYPos())<MAX_DIST){
                    myRobot.setXPos((foundObjects_c[i].getXPos()+foundObjects_w[j].getXPos())/2);
                    myRobot.setYPos((foundObjects_c[i].getYPos()+foundObjects_w[j].getYPos())/2);
                    foundRobots.push_back(myRobot);
                    robotFound=true;
                    break;
                }
            }
        }

        //let user know you found an object
        if(robotFound ==true) {
            //draw object location on screen
            drawObject(foundRobots, currentImage);
            if (foundRobots.size() != 0)
            {
                for (int i = 0; i < foundRobots.size(); i++)
                {
                    elikos_ros::RobotPos pos = foundRobots.at(i).toMsg();
                    robotsPos_msg.robotsPos.push_back(pos);
                }
            }
        }
       // morphOps(merged);
/*
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
        //trackFilteredObjects(threshold_c,cropped_hsv,currentImage);
        */
    }

    void Detection::trackShape()
    {
        //cvtColor(currentImage, grayscale_image, COLOR_BGR2GRAY);
        /*int operation = MORPH_OP + 2;
        Mat element = getStructuringElement( MORPH_ELEMENT, Size( 2*MORPH_SIZE + 1, 2*MORPH_SIZE+1 ), Point( MORPH_SIZE, MORPH_SIZE ) );
        morphologyEx(merged, morph_ex, operation, element, Point(-1,-1), 30);
*/
       Canny(merged, canny, CANNY_THRESH1, CANNY_THRESH2);

        std::vector<std::vector<Point>> contours;
        findContours(canny, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

        contour_drawings = Mat::zeros(canny.size(), CV_8UC3);
        std::vector<std::vector<Point>> polygons(contours.size());

        std::vector<std::vector<Point> > contours_poly( contours.size() );
        //std::vector<Rect> boundRect( contours.size() );
        std::vector<Point2f>center( contours.size() );
        std::vector<float>radius( contours.size() );

        for( int i = 0; i < contours.size(); i++ )
        { approxPolyDP( Mat(contours[i]), contours_poly[i], 20, true );
            //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
            minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        }


        /// Draw polygonal contour + bonding rects + circles

        for( int i = 0; i< contours.size(); i++ )
        {
            drawContours( contour_drawings, contours_poly, i, Scalar(255,255,255), 1, 8, vector<Vec4i>(), 0, Point() );
            circle( contour_drawings, center[i], (int)radius[i], Scalar(255,255,255), 2, 8, 0 );
        }

/*
        for (int i = 0; i < contours.size(); i++) {
            if (contourArea(contours[i]) < POLY_AREA_MIN || contourArea(contours[i]) > POLY_AREA_MAX) { // Should be in relation with altitude
                continue;
            }
            approxPolyDP(Mat(contours[i]), polygons[i], 10.0, true);
        }

        for (int i = 0; i < polygons.size(); i++) {
           // drawContours(contour_drawings, polygons, i, Scalar(255,255,255), 2, 8);
            drawContours(contour_drawings, polygons, i, Scalar(255,255,255), 2, 8);
        }
*/
    }

    void Detection::showThreshold()
    {
        imshow("White threshold", threshold_w);
        //imshow("Cropped image", cropped_hsv);
        imshow("Color threshold", threshold_c);
        //imshow("Merged", merged);
        //imshow("Morp", morph_ex);
        //imshow("Blurred", grayscale_image);
        //imshow("Canny Edges", canny);
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

        Mat erodeElement = getStructuringElement( MORPH_ELLIPSE,Size(3,3));
        //dilate with larger element so make sure object is nicely visible
        Mat dilateElement = getStructuringElement( MORPH_ELLIPSE,Size(8,8));
/*
        erode(thresh,thresh,erodeElement);
        dilate(thresh,thresh,dilateElement);

        erode(thresh,thresh,erodeElement);
        dilate(thresh,thresh,dilateElement);
        */
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
        foundRobots.clear();
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
        if (foundRobots.empty()) {
            throw std::out_of_range("No object is currently detected.");
        }

        // Set pitch - y axis (image vertical)
        double pitch = ((double)(foundRobots[0].getVPos() - CAM_HEIGHT / 2) / (double)CAM_HEIGHT) * CAMERA_FOV_V;

        // Set yaw - z axis (image horizontal)
        double yaw = -((double)(foundRobots[0].getHPos() - CAM_WIDTH / 2) / (double)CAM_WIDTH) * CAMERA_FOV_H;

        // Set roll, pitch and yaw
        q.setRPY(0, pitch, yaw);
    }
}
