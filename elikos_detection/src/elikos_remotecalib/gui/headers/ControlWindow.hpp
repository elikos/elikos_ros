#ifndef CONTROL_WINDOW
#define CONTROL_WINDOW

#include <string>

#include "WindowCV.hpp"
class ControlWindow;
#include "CalibrationWindow.hpp"

class ControlWindow : public WindowCV
{
private:
  static const int VALUES_SIZE = 9;
  const std::string keys_[VALUES_SIZE] = {"H+", "H-", "S+", "S-", "V+", "V-", "Pre-Erode", "Dilations", "Post-Erode"};
  const int maxValues_[VALUES_SIZE] = {180, 180, 255, 255, 255, 255, 255, 255, 255};

  int values_[VALUES_SIZE] = {};
  bool shouldUpdeteNextFrame = false;
  int *selectedColor_ = 0;

  CalibrationWindow* calibWindow_ = 0;

public:
  ControlWindow(std::string);
  virtual bool update(cv::Mat &input, std::string &outputCommand);
  virtual void keyPressed(char key);

  void setSelectedColor(int &selectedColor)
  {
    selectedColor_ = &selectedColor;
  }

  void updateTrackBars()
  {
    for (int i = 0; i < VALUES_SIZE; ++i)
    {
      cv::setTrackbarPos(keys_[i], windowName_, values_[i]);
    }
  }

  void setValues(const int &h, const int &s, const int &v, const int &delta)
  {
    values_[0] = (int)(h + 0.15 * delta);
    values_[1] = (int)(h - 0.15 * delta);
    values_[2] = (int)(s + 1.1 * delta);
    values_[3] = (int)(s - 1.1 * delta);
    values_[4] = v + 2 * delta;
    values_[5] = v - 2 * delta;
    updateTrackBars();
  }

  int* getValues(){return values_;}
  void setCalibWindow(CalibrationWindow* calibWindow_){this->calibWindow_ = calibWindow_;}

public:
  virtual void mouseCallBack(int event, int x, int y, int flags);
  static void onTrackBar(int a, void *param);
};

#endif