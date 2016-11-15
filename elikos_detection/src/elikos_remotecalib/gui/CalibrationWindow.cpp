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

CalibrationWindow::~CalibrationWindow()
{
    if (outputImage != 0)
    {
        delete (outputImage);
        outputImage = 0;
    }
}

static cv::Vec3b convertVec3b(cv::Vec3b pixel, int conversion)
{
    cv::Mat inputMat(1, 1, CV_8UC3);
    cv::Mat outputMat(1, 1, CV_8UC3);

    inputMat.at<cv::Vec3b>(0) = pixel;

    cv::cvtColor(inputMat, outputMat, conversion);

    return outputMat.at<cv::Vec3b>(0);
}

bool CalibrationWindow::update(cv::Mat &input, std::string &outputCommand)
{
    if (outputImage == 0)
    {
        outputImage = new cv::Mat(input.size().height, input.size().width + selectedColorSquareWidth_ * 3, input.type());
    }
    if (!paused)
    {
        currentFrame_ = input;
        input.copyTo(outputImage->rowRange(0, input.size().height).colRange(0, input.size().width));
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
    cv::imshow(windowName_, *outputImage);

    if (needToUpdate)
    {
        needToUpdate = false;
        std::stringstream ss;
        ss << "update" << '\t' << selectedColor_ << '\t' << H_[selectedColor_] << '\t' << S_[selectedColor_] << '\t' << V_[selectedColor_] << '\t' << DELTA;
        outputCommand = ss.str();
        ctrlWindow_->setValues(H_[selectedColor_], S_[selectedColor_], V_[selectedColor_], DELTA);

        updatePreviewColors();

        return true;
    }
    return false;
}

void CalibrationWindow::updatePreviewColors()
{
    {
        cv::Vec3b maxHSV(ctrlWindow_->getValues()[0], ctrlWindow_->getValues()[2], ctrlWindow_->getValues()[4]);

        cv::Mat mat((*outputImage)(
            cv::Rect(
                outputImage->size().width - selectedColorSquareWidth_ * (MAX_COLOR_NUM - selectedColor_),
                0,
                selectedColorSquareWidth_,
                outputImage->size().height / 3)));
        mat.setTo(convertVec3b(maxHSV, CV_HSV2BGR));
    }
    {
        cv::Mat mat((*outputImage)(
            cv::Rect(
                outputImage->size().width - selectedColorSquareWidth_ * (MAX_COLOR_NUM - selectedColor_),
                outputImage->size().height / 3 * 1,
                selectedColorSquareWidth_,
                outputImage->size().height / 3)));
        mat.setTo(convertVec3b(cv::Vec3b(H_[selectedColor_], S_[selectedColor_], V_[selectedColor_]), CV_HSV2BGR));
    }
    {
        cv::Vec3b minHSV(ctrlWindow_->getValues()[1], ctrlWindow_->getValues()[3], ctrlWindow_->getValues()[5]);
        cv::Mat mat((*outputImage)(
            cv::Rect(
                outputImage->size().width - selectedColorSquareWidth_ * (MAX_COLOR_NUM - selectedColor_),
                outputImage->size().height / 3 * 2,
                selectedColorSquareWidth_,
                outputImage->size().height / 3)));
        mat.setTo(convertVec3b(minHSV, CV_HSV2BGR));
    }
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
        cv::Mat subImgHSV = (testHSV)(cv::Rect(x, y, 5, 5));

        ////////////////////////////////////////////////////////////////////////////////
        int tS = 0, tV = 0;
        double tHCos = 0.0, tHSin = 0.0;
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                cv::Vec3b &tC = subImgHSV.at<cv::Vec3b>(j, subImgHSV.size().width - i);
                const ValeurPolaire &valH = PolarCoordTable::get()[(int)(tC(0))];
                tHCos += valH.cosinus;
                tHSin += valH.sinus;
                tS += (int)(tC(1));
                tV += (int)(tC(2));
            }
        }
        tHCos /= 25.0;
        tHSin /= 25.0;

        H_[selectedColor_] = PolarCoordTable::get().getAngle(tHCos, tHSin);
        S_[selectedColor_] = tS / 25;
        V_[selectedColor_] = tV / 25;
        ////////////////////////////////////////////////////////////////////////////////

        ctrlWindow_->setValues(H_[selectedColor_], S_[selectedColor_], V_[selectedColor_], DELTA);

        needToUpdate = true;
    }
}

void CalibrationWindow::setControlWindow(ControlWindow *ctrlWindow)
{
    ctrlWindow_ = ctrlWindow;
    ctrlWindow_->setSelectedColor(selectedColor_);
    ctrlWindow->setCalibWindow(this);
}
