#include "WindowCV.hpp"
#include <string>
#include <sstream>

static const std::string strSelectedColor = "Selected Color: ";

static void mouseCallBackStatic(int event, int x, int y, int flags, void *param)
{
    if (param == 0)
    {
        ROS_INFO("NULL POINTER EXCEPTION");
        return;
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        // WindowCV *win = reinterpret_cast<WindowCV *>(param);
        // win->mouseCallBack(event, x, y, flags, param);
        // ROS_INFO("PRE CAST");
        WindowCV *win = reinterpret_cast<WindowCV *>(param);
        // ROS_INFO("POST CAST");

        win->mouseCallBack(event, x, y, flags);
        // ROS_INFO("POST mouseCallBACK");

        std::stringstream ss;
        ss << "Clicked at " << x << ":" << y;
        ROS_INFO(ss.str().c_str());
    }
}

static void onTrackBar(int a, void *param)
{
    WindowCV *win = reinterpret_cast<WindowCV *>(param);
    win->forceUpdate();
}

WindowCV::WindowCV()
{
    // cv::namedWindow(windowName, CV_GUI_EXPANDED);
    cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    cv::setMouseCallback(windowName, mouseCallBackStatic, this);
    cv::createTrackbar("ErrorRange", windowName, &DELTA, 100, onTrackBar, this);
}
WindowCV::~WindowCV()
{
}

bool WindowCV::update(cv::Mat &input, std::string &outputCommand)
{
    currentFrame_ = input;
    switch (selectedColor_)
    {
    case 0: //Red
        cv::putText(input, strSelectedColor + "RED", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2.0);
        break;
    case 1: //Green
        cv::putText(input, strSelectedColor + "GREEN", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
        break;
    case 2: //White
        cv::putText(input, strSelectedColor + "WHITE", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), 2.0);
        break;
    }
    cv::imshow(windowName, input);

    if (needToUpdate)
    {
        needToUpdate = false;
        std::stringstream ss;
        ss << "update" << '\t' << selectedColor_ << '\t' << H_ << '\t' << S_ << '\t' << V_ << '\t' << DELTA;
        outputCommand = ss.str();
        return true;
    }
    return false;
}

void WindowCV::keyPressed(char key)
{
    switch (key)
    {
    case '0':
        selectedColor_ = 0;
        break;
    case '1':
        selectedColor_ = 1;
        break;
    case '2':
        selectedColor_ = 2;
        break;
    }
}
void WindowCV::mouseCallBack(int event, int x, int y, int flags)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {

        cv::Vec3b tempBGR = currentFrame_.at<cv::Vec3b>(y, currentFrame_.size().width - x); //Get (BGR) of the clicked pixel
        cv::Mat testHSV;
        cv::cvtColor(currentFrame_, testHSV, CV_BGR2HSV);
        cv::Vec3b pxColorHSV = testHSV.at<cv::Vec3b>(y, testHSV.size().width - x); //Get (HSV) of the clicked pixel

        H_ = (int)(pxColorHSV(0));
        S_ = (int)(pxColorHSV(1));
        V_ = (int)(pxColorHSV(2));

        needToUpdate = true;
        //        cout << "RGB: " << static_cast<int> (tempBGR(0)) << " : " << static_cast<int> (tempBGR(1)) << " : " << static_cast<int> (tempBGR(2)) << endl;
        // pxColor = cv::Vec3b(pxColorHSV);
        // pxColorBGR = cv::Scalar(static_cast<int> (tempBGR(2)), static_cast<int> (tempBGR(1)), static_cast<int> (tempBGR(0)));
    }
}