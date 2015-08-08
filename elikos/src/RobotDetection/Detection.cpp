#include <geometry_msgs/PoseStamped.h>
#include "Detection.h"

namespace elikos_detection {
    Detection::Detection(ros::NodeHandle *nh) : nh_(nh), it_(*nh) {
        // Load parameters
        nh->param<int>("robotDetect/pre_erosions_w", PRE_EROSIONS_W, 12);
        nh->param<int>("robotDetect/dilations_w", DILATIONS_W, 25);
        nh->param<int>("robotDetect/post_erosions_w", POST_EROSIONS_W, 5);
        nh->param<int>("robotDetect/pre_erosions_g", PRE_EROSIONS_G, 3);
        nh->param<int>("robotDetect/dilations_g", DILATIONS_G, 20);
        nh->param<int>("robotDetect/post_erosions_g", POST_EROSIONS_G, 0);
        /*
        nh->param<int>("robotDetect/h_min_w", H_MIN_W, 0);
        nh->param<int>("robotDetect/h_max_w", H_MAX_W, 256);
        nh->param<int>("robotDetect/s_min_w", S_MIN_W, 0);
        nh->param<int>("robotDetect/s_max_w", S_MAX_W, 256);
        nh->param<int>("robotDetect/v_min_w", V_MIN_W, 0);
        nh->param<int>("robotDetect/v_max_w", V_MAX_W, 256);
        nh->param<int>("robotDetect/h_min_c", H_MIN_G, 0);
        nh->param<int>("robotDetect/h_max_c", H_MAX_G, 256);
        nh->param<int>("robotDetect/s_min_c", S_MIN_G, 0);
        nh->param<int>("robotDetect/s_max_c", S_MAX_G, 256);
        nh->param<int>("robotDetect/v_min_c", V_MIN_G, 0);
        nh->param<int>("robotDetect/v_max_c", V_MAX_G, 256);*/

        nh->param<int>("robotDetect/h_min_w", H_MIN_W, 0);
        nh->param<int>("robotDetect/h_max_w", H_MAX_W, 52);
        nh->param<int>("robotDetect/s_min_w", S_MIN_W, 0);
        nh->param<int>("robotDetect/s_max_w", S_MAX_W, 149);
        nh->param<int>("robotDetect/v_min_w", V_MIN_W, 140);
        nh->param<int>("robotDetect/v_max_w", V_MAX_W, 256);
        nh->param<int>("robotDetect/h_min_g", H_MIN_G, 39);
        nh->param<int>("robotDetect/h_max_g", H_MAX_G, 88);
        nh->param<int>("robotDetect/s_min_g", S_MIN_G, 54);
        nh->param<int>("robotDetect/s_max_g", S_MAX_G, 256);
        nh->param<int>("robotDetect/v_min_g", V_MIN_G, 30);
        nh->param<int>("robotDetect/v_max_g", V_MAX_G, 256);
        nh->param<int>("robotDetect/h_min_r", H_MIN_R, 0);
        nh->param<int>("robotDetect/h_max_r", H_MAX_R, 256);
        nh->param<int>("robotDetect/s_min_r", S_MIN_R, 0);
        nh->param<int>("robotDetect/s_max_r", S_MAX_R, 256);
        nh->param<int>("robotDetect/v_min_r", V_MIN_R, 0);
        nh->param<int>("robotDetect/v_max_r", V_MAX_R, 256);
        nh->param<int>("robotDetect/pre_blur", PRE_BLUR, 2);
        nh->param<int>("robotDetect/canny_thresh1", CANNY_THRESH1, 1);
        nh->param<int>("robotDetect/canny_thresh1", CANNY_THRESH2, 1);
        nh->param<int>("robotDetect/canny_aperture", CANNY_APERTURE, 3);
        nh->param<int>("robotDetect/poly_area_min", POLY_AREA_MIN, 0);
        nh->param<int>("robotDetect/poly_area_max", POLY_AREA_MAX, 50000);
        nh->param<int>("robotDetect/morph_op", MORPH_OP, 0);
        nh->param<int>("robotDetect/morph_op", MORPH_ELEMENT, 0);
        nh->param<int>("robotDetect/morph_size", MORPH_SIZE, 0);
        nh->param<int>("robotDetect/max_dist", MAX_DIST, 100);

    }

    Detection::~Detection() {
    }

    void Detection::init() {
        setPublishers();
        setSubscribers();
        initCameraTF();
    }

    string Detection::intToString(int number) {
        std::stringstream ss;
        ss << number;
        return ss.str();
    }

    void on_trackbar(int, void *) {//This function gets called whenever a
        // trackbar position is changed
    }

    void Detection::setupDebug() {
        //TODO : Change the hardcoding on the camera number
        try {
            capture.open(0);
        }
        catch (int e) {
            printf("No cameras detected");
        }

        capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    }

    void Detection::captureFrame() {
        //store image to matrix
        capture.read(currentImage);
        //convert frame from BGR to HSV colorspace
        cvtColor(currentImage, hsv_w, COLOR_BGR2HSV);
        //printf("I captured an image with my webcam");
    }

    void Detection::createTrackbars() {
        //create window for trackbars
        namedWindow(trackbarWindowName, 0);
        namedWindow(HSVTrackbars, 0);
        namedWindow(shapeDetectTrackbars, 0);
        //create trackbars and insert them into window
        //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
        //the max value the trackbar can move (eg. H_HIGH),
        //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
        //                                  ---->    ---->     ---->
        createTrackbar("PRE BLUR", trackbarWindowName, &PRE_BLUR, 50, on_trackbar);
        createTrackbar("PRE EROSIONS W", trackbarWindowName, &PRE_EROSIONS_W, 256, on_trackbar);
        createTrackbar("DILATIONS_W W", trackbarWindowName, &DILATIONS_W, 256, on_trackbar);
        createTrackbar("POST EROSIONS W", trackbarWindowName, &POST_EROSIONS_W, 50, on_trackbar);
        createTrackbar("PRE EROSIONS G", trackbarWindowName, &PRE_EROSIONS_G, 256, on_trackbar);
        createTrackbar("DILATIONS_W G", trackbarWindowName, &DILATIONS_G, 256, on_trackbar);
        createTrackbar("POST EROSIONS G", trackbarWindowName, &POST_EROSIONS_G, 50, on_trackbar);
        createTrackbar("H_MIN W", HSVTrackbars, &H_MIN_W, 256, on_trackbar);
        createTrackbar("H_MAX W", HSVTrackbars, &H_MAX_W, 256, on_trackbar);
        createTrackbar("S_MIN W", HSVTrackbars, &S_MIN_W, 256, on_trackbar);
        createTrackbar("S_MAX W", HSVTrackbars, &S_MAX_W, 256, on_trackbar);
        createTrackbar("V_MIN W", HSVTrackbars, &V_MIN_W, 256, on_trackbar);
        createTrackbar("V_MAX W", HSVTrackbars, &V_MAX_W, 256, on_trackbar);
        createTrackbar("H_MIN G", HSVTrackbars, &H_MIN_G, 256, on_trackbar);
        createTrackbar("H_MAX G", HSVTrackbars, &H_MAX_G, 256, on_trackbar);
        createTrackbar("S_MIN G", HSVTrackbars, &S_MIN_G, 256, on_trackbar);
        createTrackbar("S_MAX G", HSVTrackbars, &S_MAX_G, 256, on_trackbar);
        createTrackbar("V_MIN G", HSVTrackbars, &V_MIN_G, 256, on_trackbar);
        createTrackbar("V_MAX G", HSVTrackbars, &V_MAX_G, 256, on_trackbar);
        createTrackbar("H_MIN R", HSVTrackbars, &H_MIN_R, 256, on_trackbar);
        createTrackbar("H_MAX R", HSVTrackbars, &H_MAX_R, 256, on_trackbar);
        createTrackbar("S_MIN R", HSVTrackbars, &S_MIN_R, 256, on_trackbar);
        createTrackbar("S_MAX R", HSVTrackbars, &S_MAX_R, 256, on_trackbar);
        createTrackbar("V_MIN R", HSVTrackbars, &V_MIN_R, 256, on_trackbar);
        createTrackbar("V_MAX R", HSVTrackbars, &V_MAX_R, 256, on_trackbar);

        // Create shape detector trackbars
        createTrackbar("Canny Thresh 1", shapeDetectTrackbars, &CANNY_THRESH1, 256, on_trackbar);
        createTrackbar("Canny Thresh 2", shapeDetectTrackbars, &CANNY_THRESH2, 256, on_trackbar);
        createTrackbar("Canny Aperture", shapeDetectTrackbars, &CANNY_APERTURE, 5, on_trackbar);
        createTrackbar("Poly Area Min", shapeDetectTrackbars, &POLY_AREA_MIN, 50000, on_trackbar);
        createTrackbar("Poly Area Max", shapeDetectTrackbars, &POLY_AREA_MAX, 50000, on_trackbar);
        createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat",
                       shapeDetectTrackbars, &MORPH_OP, 4, on_trackbar);
        createTrackbar("Element:\n 0: Rect - 1: Cross - 2: Ellipse", shapeDetectTrackbars, &MORPH_ELEMENT, 2,
                       on_trackbar);
        createTrackbar("Kernel size:\n 2n +1", shapeDetectTrackbars, &MORPH_SIZE, 21, on_trackbar);
        createTrackbar("Max distance", shapeDetectTrackbars, &MAX_DIST, 50000, on_trackbar);

    }


    void Detection::cameraCallback(const sensor_msgs::ImageConstPtr &msg) {
        currentImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }

    void Detection::altitudeCallback(const sensor_msgs::RangeConstPtr &msg) {
        if (msg->range != 0) {
            altRange = msg->range;
        }
    }

    void Detection::setPublishers() {
        //TODO : publish robot info
        if (nh_) {
            std::string topicName = TOPIC_NAMES[robotsPos];
            robots_publish = nh_->advertise<elikos_ros::RobotsPos>(topicName, 1);

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
            else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
        }
        return vecRobot;
    }

    void Detection::trackBlobs() {
        cvtColor(currentImage, hsv_w, COLOR_BGR2HSV);
        cvtColor(currentImage, hsv_c, COLOR_BGR2HSV);


        BLUR_AMOUNT = PRE_BLUR + 1;
        blur(hsv_w, hsv_w, Size(BLUR_AMOUNT, BLUR_AMOUNT), Point(-1, -1));

        blur(hsv_c, hsv_c, Size(BLUR_AMOUNT, BLUR_AMOUNT), Point(-1, -1));

        // Find the white in the image (the robot plastic casing)
        inRange(hsv_w, Scalar(H_MIN_W, S_MIN_W, V_MIN_W), Scalar(H_MAX_W, S_MAX_W, V_MAX_W), threshold_w);

        // Consolidate the white parts into one big blob to delimit the robot
        erode(threshold_w, threshold_w, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1),
              PRE_EROSIONS_W);
        dilate(threshold_w, threshold_w, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1), DILATIONS_W);
        erode(threshold_w, threshold_w, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1),
              POST_EROSIONS_W);

        //trackShape();

        inRange(hsv_c, Scalar(H_MIN_G, S_MIN_G, V_MIN_G), Scalar(H_MAX_G, S_MAX_G, V_MAX_G), threshold_g);

        // Consolidate the white parts into one big blob to delimit the robot
        erode(threshold_g, threshold_g, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1),
              PRE_EROSIONS_G);
        dilate(threshold_g, threshold_g, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1), DILATIONS_G);
        erode(threshold_g, threshold_g, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1),
              POST_EROSIONS_G);

        inRange(hsv_c, Scalar(H_MIN_R, S_MIN_R, V_MIN_R), Scalar(H_MAX_R, S_MAX_R, V_MAX_R), threshold_r);

        // Consolidate the white parts into one big blob to delimit the robot
        erode(threshold_r, threshold_r, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1),
              PRE_EROSIONS_W);
        dilate(threshold_r, threshold_r, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1), DILATIONS_W);
        erode(threshold_r, threshold_r, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)), Point(-1, -1),
              POST_EROSIONS_W);


        vector<RobotDesc> foundObjects_g = trackFilteredObjects(threshold_g, currentImage);
        vector<RobotDesc> foundObjects_r = trackFilteredObjects(threshold_r, currentImage);

        /* for (int i=0; i<foundObjects_g.size(); i++){
            printf("Found Color Object[%i]:\n", i);
            printf("xPos: %i\n", foundObjects_g[i].getXPos());
            printf("yPos: %i\n", foundObjects_g[i].getYPos());
        }*/
        //drawObjects(foundObjects_g, currentImage);
/*
        for (int i=0; i<foundObjects_w.size(); i++){
            printf("Found White Object[%i]:\n", i);
            printf("xPos: %i\n", foundObjects_w[i].getXPos());
            printf("yPos: %i\n", foundObjects_w[i].getYPos());
        }*/
        //drawObjects(foundObjects_g, currentImage);
        // cout << "Found color " << foundObjects_g.size() << " and white " << foundObjects_w.size() << endl;
        // foundRobots.clear();
        RobotDesc myRobot;
        foundRobots.clear();

        mask = cv::Mat::zeros(FRAME_HEIGHT, FRAME_WIDTH, CV_8U);
        for (int i = 0; i < foundObjects_g.size(); i++) {
                cv::circle(mask, cv::Point(foundObjects_g[i].getXPos(), foundObjects_g[i].getYPos()), MAX_DIST,
                           cv::Scalar(255, 0, 0), -1);
        }
/*
        for (int i=0; i< foundObjects_r.size(); i++){
                cv::circle(mask, cv::Point(foundObjects_r[i].getXPos(),foundObjects_r[i].getYPos()), MAX_DIST, cv::Scalar(255, 0, 0), -1);
        }
*/
        bitwise_and(mask, threshold_w, closeWhite);

        vector<RobotDesc> foundObjects_w = trackFilteredObjects(closeWhite, currentImage);


        //Finding green robots
        for (int i = 0; i < foundObjects_g.size(); i++) {
            for (int j = 0; j < foundObjects_w.size(); j++) {
                if (abs(foundObjects_g[i].getXPos() - foundObjects_w[j].getXPos()) < MAX_DIST &&
                    abs(foundObjects_g[i].getYPos() - foundObjects_w[j].getYPos()) < MAX_DIST) {
                    myRobot.setXPos(foundObjects_g[i].getXPos());
                    myRobot.setYPos(foundObjects_g[i].getYPos());
                    foundRobots.push_back(myRobot);
                    break;
                }
            }
        }
/*
        //Finding red robots
        for (int i=0; i< foundObjects_r.size(); i++){
            for (int j=0; j<foundObjects_w.size(); j++){
                if (abs(foundObjects_r[i].getXPos()-foundObjects_w[j].getXPos())<MAX_DIST &&
                    abs(foundObjects_r[i].getYPos()-foundObjects_w[j].getYPos())<MAX_DIST){
                    myRobot.setXPos(foundObjects_r[i].getXPos());
                    myRobot.setYPos(foundObjects_r[i].getYPos());
                    foundRobots.push_back(myRobot);
                    break;
                }
            }
        }
*/

        /*  bool redRobotFound = false;
        for (int i=0; i< foundObjects_r.size(); i++){
            for (int j=0; j<foundObjects_w.size(); j++){
                if (abs(foundObjects_r[i].getXPos()-foundObjects_w[j].getXPos())<MAX_DIST&&
                    abs(foundObjects_r[i].getYPos()-foundObjects_w[j].getYPos())<MAX_DIST){
                    myRobot.setXPos((foundObjects_g[i].getXPos()+foundObjects_w[j].getXPos())/2);
                    myRobot.setYPos((foundObjects_g[i].getYPos()+foundObjects_w[j].getYPos())/2);
                    foundRobots.push_back(myRobot);
                    redRobotFound = true;
                    //break;
                }
            }
        }
*/
        //let user know you found an object

        //draw object location on screen
        drawObjects(foundRobots, currentImage);
        /*  if (foundRobots.size() != 0)
            {
                for (int i = 0; i < foundRobots.size(); i++)
                {
                    elikos_ros::RobotPos pos = foundRobots.at(i).toMsg();
                    robotsPos_msg.robotsPos.push_back(pos);
                }
            }*/

    }

    void Detection::trackShape() {
        cvtColor(currentImage, grayscale_image, COLOR_BGR2GRAY);

        GaussianBlur(grayscale_image, grayscale_image, Size(9, 9), 2, 2);

        vector<Vec3f> circles;

        HoughCircles(grayscale_image, circles, CV_HOUGH_GRADIENT, 1, grayscale_image.rows / 8, 200, 100, 0, 0);

        //printf(circles.size());
        for (size_t i = 0; i < circles.size(); i++) {
            printf("HERE");
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle(currentImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(currentImage, center, radius, Scalar(0, 0, 255), 3, 8, 0);
        }
    }

    void Detection::showThreshold() {
        imshow("White threshold", threshold_w);
        //imshow("Cropped image", cropped_hsv);
        imshow("Green threshold", threshold_g);
        //imshow("Red threshold", threshold_r);
        imshow("Merged", closeWhite);
        //imshow("Morp", morph_ex);
        //imshow("Blurred", grayscale_image);
        //imshow("Canny Edges", canny);
        try {
            imshow("Circle", mask);
        }
        catch (Exception &e) {
            cout << "imshow exception: " << e.what() << endl << endl;
        }

    }

    void Detection::setSubscribers() {
        image_sub_ = it_.subscribe(TOPIC_NAMES[camera_image_raw], 1, &Detection::cameraCallback, this);
        sub = nh_->subscribe(TOPIC_NAMES[elikos_robotdetect_altitude], 1, &Detection::altitudeCallback, this);
    }

    void Detection::drawObjects(vector<RobotDesc> vecRobot, Mat &frame) {

        for (int i = 0; i < vecRobot.size(); i++) {
            cv::circle(frame, cv::Point(vecRobot.at(i).getHPos(), vecRobot.at(i).getVPos()), 10, cv::Scalar(0, 0, 255));
            cv::putText(frame, intToString(vecRobot.at(i).getHPos()) + " , " + intToString(vecRobot.at(i).getVPos()),
                        cv::Point(vecRobot.at(i).getHPos(), vecRobot.at(i).getVPos() + 20), 1, 1, Scalar(0, 255, 0));
        }
    }

    void Detection::drawObject(RobotDesc robot, Mat &frame) {
        cv::circle(frame, cv::Point(robot.getHPos(), robot.getVPos()), 20, cv::Scalar(255, 0, 0));
        cv::putText(frame, intToString(robot.getHPos()) + " , " + intToString(robot.getVPos()),
                    cv::Point(robot.getHPos(), robot.getVPos() + 20), 1, 1, Scalar(0, 255, 0));
    }


    void Detection::showCurrentImage() {
        imshow(windowName, currentImage);
    }


    void Detection::sendMsg() {
        robots_publish.publish(robotsPos_msg);
        //ROS_INFO_STREAM("I am advertising a robotInfo vector");
        foundRobots.clear();
    }


    void Detection::initCameraTF() {
        // Set up the camera's position relative to the fcu
        camera_.setOrigin(tf::Vector3(-0.10, 0, -0.05));
        camera_.setRotation(tf::Quaternion(tf::Vector3(0, 1, 0), PI / 2));
    }

    void Detection::computeTargetPosition() {
        char buf[50];
        int n = 0;
        memset(buf, 0, 50);

        // Append the camera frame to the fcu
        tf_broadcaster_.sendTransform(tf::StampedTransform(camera_, ros::Time::now(), "fcu", "camera"));

        // Clear minimum distance
        min_distance = 0;
        int robotIterator = -1;

        cout << "Altitude range: " << altRange << endl;
        int i = 0;
        for (vector<RobotDesc>::iterator iter = foundRobots.begin(); iter != foundRobots.end(); ++iter) {
            //initialize shit
            tf::Quaternion q = tf::createIdentityQuaternion();
            tf::Transform t = tf::Transform::getIdentity();

            // Set yaw and pitch of the target wrt the camera frame
            getRotationFromImage(q, *iter);

            t.setOrigin(tf::Vector3(0, 0, 0));
            t.setRotation(q);
            n = sprintf(buf, "turret%d", i);
            string frameid(buf);
            tf::StampedTransform stampedTF(t, ros::Time::now(), "camera", frameid);
            tf_broadcaster_.sendTransform(stampedTF);

            // Get the world to turret transform
            tf::StampedTransform world2turret;
            try {
                tf_listener_.lookupTransform("local_origin", frameid, ros::Time(0), world2turret);
            } catch (tf::LookupException &e) {
                ROS_ERROR(e.what());
            } catch (tf::ConnectivityException &e) {
                ROS_ERROR(e.what());
            } catch (tf::ExtrapolationException e) {
                ROS_ERROR(e.what());
            }

            // Get the smallest angle between the turret and the z axis
            //      - First get the vector pointing towards the x axis of the turret
            tf::Vector3 vect_x = tf::quatRotate(world2turret.getRotation(), tf::Vector3(1, 0, 0));

            //      - Then find it's angle with the camera's resting position (x pointing straight down (-z))
            tf::Vector3 zAxis(0, 0, -1);
            double zAxis_turret_angle = zAxis.angle(vect_x);

            // Get distance from turret to target (using angle and altitude)

            double cam_alt = world2turret.getOrigin().getZ();
            //cout<<"Altitude range: " << cam_alt << endl;
            if (cam_alt != 0) {
                double distance_from_target = cam_alt / cos(zAxis_turret_angle);

                if (min_distance == 0 || distance_from_target < min_distance) {
                    robotIterator = i;
                    // Add the robot transform as child of the turret
                    target_robot_.setOrigin(tf::Vector3(distance_from_target, 0, 0));
                    target_robot_.setRotation(tf::Quaternion(0, 0, 0, 1));
                    min_distance = distance_from_target;
                    cout << "robot " << i << " at distance " << distance_from_target << endl;
                }
            }


            turret_rotation_.push_back(q);
            turret_.push_back(t);
            turret_frame_id_.push_back(frameid);
            turret_world_.push_back(world2turret);
            turret_world_x_.push_back(vect_x);
            ++i;
        }

        std::stringstream ss;
        ss << "turret" << robotIterator;

        if (robotIterator >= 0) {
            tf_broadcaster_.sendTransform(
                    tf::StampedTransform(target_robot_, ros::Time::now(), ss.str(), "target_robot"));

            //draw object location on screen
            std::cout << "Robot iterator: " << robotIterator << "\n";
            drawObject(foundRobots[robotIterator], currentImage);
        }
    }

    void Detection::getRotationFromImage(tf::Quaternion &q, RobotDesc desc) {
        double pitch = ((double) (desc.getVPos() - CAM_HEIGHT / 2) / (double) CAM_HEIGHT) * CAMERA_FOV_V;
        double yaw = -((double) (desc.getHPos() - CAM_WIDTH / 2) / (double) CAM_WIDTH) * CAMERA_FOV_H;
        q.setRPY((double) 0.0, pitch, yaw);
    }

    void Detection::printROSparams() {

        printf("\n# Morphological operations iterations for colors filtering\n");
        printf("pre_blur: %i\n\n", PRE_BLUR);

        printf("pre_erosions_w: %i\n", PRE_EROSIONS_W);
        printf("dilations_w: %i\n", DILATIONS_W);
        printf("pre_erosions_g: %i\n", PRE_EROSIONS_G);
        printf("dilations_g: %i\n", DILATIONS_G);
        printf("post_erosions_g: %i\n\n", POST_EROSIONS_G);

        printf("# HSV filter parameters for the white part of the robot\n");
        printf("h_min_w: %i\n", H_MIN_W);
        printf("h_max_w: %i\n", H_MAX_W);
        printf("s_min_w: %i\n", S_MIN_W);
        printf("s_max_w: %i\n", S_MAX_W);
        printf("v_min_w: %i\n", V_MIN_W);
        printf("v_max_w: %i\n\n", V_MAX_W);

        printf("# HSV filter parameters for the green part of the robot\n");
        printf("h_min_g: %i\n", H_MIN_G);
        printf("h_max_g: %i\n", H_MAX_G);
        printf("s_min_g: %i\n", S_MIN_G);
        printf("s_max_g: %i\n", S_MAX_G);
        printf("v_min_g: %i\n", V_MIN_G);
        printf("v_max_g: %i\n\n", V_MAX_G);

        printf("# HSV filter parameters for the red part of the robot\n");
        printf("h_min_r: %i\n", H_MIN_R);
        printf("h_max_r: %i\n", H_MAX_R);
        printf("s_min_r: %i\n", S_MIN_R);
        printf("s_max_r: %i\n", S_MAX_R);
        printf("v_min_r: %i\n", V_MIN_R);
        printf("v_max_r: %i\n\n", V_MAX_R);
    }
}
