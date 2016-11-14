#include <string>
#include <sstream>

#include "headers/CalibrationWindow.hpp"

static const std::string strSelectedColor = "Selected Color: ";

static void onTrackBar(int a, void *param)
{
    CalibrationWindow *win = reinterpret_cast<CalibrationWindow *>(param);
    win->forceUpdate();
}

CalibrationWindow::CalibrationWindow(std::string windowName) : WindowCV(windowName)
{
    cv::createTrackbar("ErrorRange", windowName_, &DELTA, 100, onTrackBar, this);
}

bool CalibrationWindow::update(cv::Mat &input, std::string &outputCommand)
{
    if (!paused)
    {
        currentFrame_ = input;
    }
    switch (selectedColor_)
    {
    case 0: //Red
        cv::putText(currentFrame_, strSelectedColor + "RED", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2.0);
        break;
    case 1: //Green
        cv::putText(currentFrame_, strSelectedColor + "GREEN", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
        break;
    case 2: //White
        cv::putText(currentFrame_, strSelectedColor + "WHITE", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), 2.0);
        break;
    }
    cv::imshow(windowName_, currentFrame_);

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

void CalibrationWindow::keyPressed(char key)
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
    case ' ':
        paused = !paused;
        break;
    }
}
void CalibrationWindow::mouseCallBack(int event, int x, int y, int flags)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {

        cv::Mat testHSV;
        cv::cvtColor(currentFrame_, testHSV, CV_BGR2HSV);
        cv::Mat subImgHSV = (testHSV)(cv::Rect(testHSV.size().width - x, y, 5, 5));

        cv::Vec3b &ColHSVTest = testHSV.at<cv::Vec3b>(y, testHSV.size().width - x);

        ////////////////////////////////////////////////////////////////////////////////
        int tS = 0, tV = 0;
        double tHCos = 0.0, tHSin = 0.0;
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                cv::Vec3b &tC = subImgHSV.at<cv::Vec3b>(j, subImgHSV.size().width - i);
                const ValeurPolaire &valH = valeursH[(int)(tC(0))];
                tHCos += valH.cosinus;
                tHSin += valH.sinus;
                tS += (int)(tC(1));
                tV += (int)(tC(2));
            }
        }
        tHCos /= 25.0;
        tHSin /= 25.0;

        H_ = getAngle(tHCos, tHSin);
        S_ = tS / 25;
        V_ = tV / 25;
        ////////////////////////////////////////////////////////////////////////////////

        ctrlWindow_->setValues(H_, S_, V_, DELTA);

        needToUpdate = true;
    }
}

void CalibrationWindow::setControlWindow(ControlWindow *ctrlWindow)
{
    ctrlWindow_ = ctrlWindow;
    ctrlWindow_->setSelectedColor(selectedColor_);
}