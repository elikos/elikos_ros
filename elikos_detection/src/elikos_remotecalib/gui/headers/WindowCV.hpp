#ifndef WINDOWCV_CLASS
#define WINDOWCV_CLASS

#include <string>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
class WindowCV
{
public:
  WindowCV(std::string windowName);
  virtual bool update(cv::Mat &input, std::string &outputCommand) = 0;
  virtual void keyPressed(char key) = 0;

protected:
  const std::string windowName_;
  cv::Mat currentFrame_;

public:
  virtual void mouseCallBack(int event, int x, int y, int flags) = 0;
};
#endif