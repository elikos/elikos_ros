#include "headers/WindowCV.hpp"

static void mouseCallBackStatic(int event, int x, int y, int flags, void *param)
{
    if (param == 0)
    {
        return;
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        WindowCV *win = reinterpret_cast<WindowCV *>(param);
        win->mouseCallBack(event, x, y, flags);
    }
}
WindowCV::WindowCV(std::string windowName) : windowName_(windowName)
{
    cv::namedWindow(windowName_, CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    cv::setMouseCallback(windowName_, mouseCallBackStatic, this);
}