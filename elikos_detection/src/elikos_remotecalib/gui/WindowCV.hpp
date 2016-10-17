#ifndef WINDOWCV_CLASS
#define WINDOWCV_CLASS

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>
class WindowCV
{
public:
  WindowCV();
  ~WindowCV();
  bool update(cv::Mat &input, std::string &outputCommand);
  void keyPressed(char key);
  void forceUpdate()
  {
    needToUpdate = true;
  };

private:
  int DELTA = 35;
  int H_;
  int S_;
  int V_;
  int selectedColor_ = 0;
  bool needToUpdate = false;
  const std::string windowName = "Remote Calibration Window";
  cv::Mat currentFrame_;

public:
  void mouseCallBack(int event, int x, int y, int flags);
  // static void mouseCallBackStatic(int event, int x, int y, int flags, void* param);
};
#endif