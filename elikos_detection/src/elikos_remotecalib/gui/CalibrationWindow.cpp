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
    cv::createTrackbar("ErrorRange", windowName_, &DELTA_, 100, onTrackBar, this);
}

CalibrationWindow::~CalibrationWindow()
{
    if (outputImage != 0)
    {
        delete (outputImage);
        outputImage = 0;
    }
}

/**
 * Méthode utilitaire qui devrait probablement partir d'ici. Converti une couleur sous format Vec3b
 * vers un espace de couleurs (BRG, HSV, etc...)
 *
 * \param pixel      [in] le pixel à convertir
 * \param conversion [in] la conversion à effectuer (CV_BRG2HSV par exemple)
 *
 * \return le pixel converti
 */
static cv::Vec3b convertVec3b(const cv::Vec3b &pixel, int conversion)
{
    static cv::Mat inputMat(1, 1, CV_8UC3);
    static cv::Mat outputMat(1, 1, CV_8UC3);

    inputMat.at<cv::Vec3b>(0) = pixel;

    cv::cvtColor(inputMat, outputMat, conversion);

    return outputMat.at<cv::Vec3b>(0);
}

bool CalibrationWindow::update(cv::Mat &input, std::string &outputCommand)
{
    if (outputImage == nullptr)
    {
        outputImage = new cv::Mat(input.size().height, input.size().width + prewiewWidth_ * 3, input.type());
    }
    if (!paused)
    {
        currentFrame_ = input;
        input.copyTo(outputImage->rowRange(0, input.size().height).colRange(0, input.size().width));
    }
    switch (selectedColor_)
    {
    case RED:
        cv::putText(currentFrame_, strSelectedColor + "RED", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2.0);
        break;
    case GREEN:
        cv::putText(currentFrame_, strSelectedColor + "GREEN", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
        break;
    case WHITE:
        cv::putText(currentFrame_, strSelectedColor + "WHITE", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), 2.0);
        break;
    }
    cv::imshow(windowName_, *outputImage);

    if (needToUpdate)
    {
        needToUpdate = false;
        std::stringstream ss;
        ss << "update" << '\t' << selectedColor_ << '\t' << H_[selectedColor_] << '\t' << S_[selectedColor_] << '\t' << V_[selectedColor_] << '\t' << DELTA_;
        outputCommand = ss.str();
        ctrlWindow_->setValues(H_[selectedColor_], S_[selectedColor_], V_[selectedColor_], DELTA_);

        updatePreviewColors();

        return true;
    }
    return false;
}

void CalibrationWindow::updatePreviewColors()
{
    const unsigned NUMBER_OF_PALETTES = 3;
    cv::Vec3b colors[NUMBER_OF_PALETTES] = {
        cv::Vec3b(ctrlWindow_->getValues()[0], ctrlWindow_->getValues()[2], ctrlWindow_->getValues()[4]),
        cv::Vec3b(H_[selectedColor_], S_[selectedColor_], V_[selectedColor_]),
        cv::Vec3b(ctrlWindow_->getValues()[1], ctrlWindow_->getValues()[3], ctrlWindow_->getValues()[5])
    };

    int upperLeftX = outputImage->size().width - prewiewWidth_ * (NUMBER_OF_COLORS - selectedColor_);

    for(unsigned i = 0; i < NUMBER_OF_PALETTES; i++){
        int paletteHeight = outputImage->size().height / NUMBER_OF_PALETTES;

        cv::Mat mat((*outputImage)(
            cv::Rect(
                upperLeftX,
                paletteHeight * i,
                prewiewWidth_,
                paletteHeight)));
        mat.setTo(convertVec3b(colors[i], CV_HSV2BGR));
    }

    cv::Vec3b hsvTextColor(180 - colors[0][0], colors[0][1], 255 - colors[0][2]);
    cv::Vec3b bgrTextColor = convertVec3b(hsvTextColor, CV_HSV2BGR);

    cv::putText((*outputImage),
        COLOR_PREVIEW_NAME_[selectedColor_],
        cv::Point(upperLeftX, 50),
        cv::FONT_HERSHEY_PLAIN,
        3.0,
        CV_RGB(bgrTextColor[2], bgrTextColor[1], bgrTextColor[0]),
        2.0);
    
}

void CalibrationWindow::keyPressed(char key)
{
    switch (key)
    {
    case '0':
        selectedColor_ = RED;
        break;
    case '1':
        selectedColor_ = GREEN;
        break;
    case '2':
        selectedColor_ = WHITE;
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
        bool inBounds = true;
        inBounds &=  x < currentFrame_.size().width && x >= 0;
        inBounds &=  y < currentFrame_.size().height && y >= 0;
        if(inBounds){

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

            ctrlWindow_->setValues(H_[selectedColor_], S_[selectedColor_], V_[selectedColor_], DELTA_);

            needToUpdate = true;
        }
    }
}

void CalibrationWindow::setControlWindow(ControlWindow *ctrlWindow)
{
    ctrlWindow_ = ctrlWindow;
    ctrlWindow_->setSelectedColor(selectedColor_);
    ctrlWindow->setCalibWindow(this);
}
