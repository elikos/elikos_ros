#ifndef CALIBRATION_WINDOW
#define CALIBRATION_WINDOW

#include "WindowCV.hpp"
#include "ControlWindow.hpp"
#include <cmath>

struct ValeurPolaire
{
  double cosinus = 0.0;
  double sinus = 0.0;

  ValeurPolaire()
  {
  }

  ValeurPolaire(double cos, double sin)
  {
    cosinus = cos;
    sinus = sin;
  }
};

class CalibrationWindow : public WindowCV
{
public:
  CalibrationWindow(std::string);
  virtual bool update(cv::Mat &input, std::string &outputCommand);
  virtual void keyPressed(char key);
  void forceUpdate()
  {
    needToUpdate = true;
  };

  void setControlWindow(ControlWindow *);
  void initValeursH()
  {
    for (int i = 0; i < 180; ++i)
    {
      valeursH[i] = {std::cos(i * 3.14159265359 / 90.0), std::sin(i * 3.14159265359 / 90.0)};
    }
  }

private:
  int DELTA = 35;
  int H_;
  int S_;
  int V_;
  int selectedColor_ = 0;
  bool needToUpdate = false;
  bool paused = false;
  ControlWindow *ctrlWindow_ = 0;
  ValeurPolaire valeursH[180];
  
  /* Utility used for getting the right Hue value */
  unsigned char getAngle(const double &cos, const double &sin)
  {
    double atan = std::atan2(sin, cos);
    atan += (atan < 0) ? 2 * 3.1415926535897 : 0;
    unsigned char angle = (unsigned char)(atan * 90 / 3.1415926535897);
    //    double angle = (atan * 90 / 3.14159265359);
    return angle;
  }

public:
  virtual void mouseCallBack(int event, int x, int y, int flags);
};
#endif