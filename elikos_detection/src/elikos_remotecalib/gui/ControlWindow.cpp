#include "headers/ControlWindow.hpp"

static const std::string strSelectedColor = "Selected Color: ";

void ControlWindow::onTrackBar(int a, void *param)
{
    ControlWindow *win = reinterpret_cast<ControlWindow *>(param);
    win->shouldUpdeteNextFrame = true;
    win->calibWindow_->updatePreviewColors();
}

ControlWindow::ControlWindow(std::string windowName) : WindowCV(windowName)
{
    for (int i = 0; i < VALUES_SIZE; ++i)
    {
        cv::createTrackbar(keys_[i], windowName_, &values_[i], maxValues_[i], onTrackBar, this);
    }

    cv::moveWindow(windowName_, 100, 100);
}

bool ControlWindow::update(cv::Mat &input, std::string &outputCommand)
{
    switch (*selectedColor_)
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
    cv::imshow(windowName_, input);

    if (shouldUpdeteNextFrame)
    {
        std::stringstream ss;
        ss << "updHSV\t" << *selectedColor_;
        for (int i = 0; i < VALUES_SIZE; ++i)
        {
            ss << "\t" << values_[i];
        }
        outputCommand = ss.str();
        shouldUpdeteNextFrame = false;
        return true;
    }

    return false;
}
void ControlWindow::keyPressed(char key)
{
   
}
void ControlWindow::mouseCallBack(int event, int x, int y, int flags)
{
}
