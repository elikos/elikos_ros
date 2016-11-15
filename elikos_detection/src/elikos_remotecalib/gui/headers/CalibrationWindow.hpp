#ifndef CALIBRATION_WINDOW
#define CALIBRATION_WINDOW

#include "WindowCV.hpp"

class CalibrationWindow;
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

/**
 * Classe qui gere les coordonés polaires et les angles. Évite de calculer des sin et des cos 
 */
class PolarCoordTable
{
private:
  static const size_t MAX_ANGLE = 180;
  const double MUL_TO_RAD = M_PI / 90.0;
  const double MUL_FROM_RAD = 1 / MUL_TO_RAD;
  ValeurPolaire valeurs[MAX_ANGLE];

  PolarCoordTable()
  {
    for (size_t i = 0; i < MAX_ANGLE; ++i)
    {
      valeurs[i] = {std::cos(i * MUL_TO_RAD), std::sin(i * MUL_TO_RAD)};
    }
  }

public:
    /**
     * Retourne une valeur polaire de l'angle fourni.
     *
     * \param angle [in] Un angle compris entre [0, 180[ (180 = 2*PI)
     * \return la valeur polaire de l'angle
     */
    const ValeurPolaire& operator[](unsigned char angle)
    {
      return valeurs[angle];
    }

    /**
     * Retourne la table de coordonés polaires avec laquelle travailler
     *
     * \return l'instance de la table de coordonés polaire
     */
    static PolarCoordTable& get()
    {
      static PolarCoordTable sinTable;
      return sinTable;
    }

    /**
     * Retourne l'angle depuis son sin et son cos
     * 
     * \param cos [in] le cosinus de l'angle (en radians)
     * \param sin [in] le sinus de l'angle (en radians)
     * 
     * \return l'angle qui correspond au sinus et cosinus, entre 0 et 180 (pour OpenCV)
     */
    unsigned char getAngle(const double &cos, const double &sin)
    {
      double atan = std::atan2(sin, cos);
      if (atan < 0)
      {
        atan += 2 * M_PI;
      }
      unsigned char angle = (unsigned char)(atan * MUL_FROM_RAD);
      return angle;
    }
};

class CalibrationWindow : public WindowCV
{
public:
  CalibrationWindow(std::string);
  ~CalibrationWindow();
  virtual bool update(cv::Mat &input, std::string &outputCommand);
  virtual void keyPressed(char key);
  void forceUpdate()
  {
    needToUpdate = true;
  };

  void setControlWindow(ControlWindow *);

private:
  static const int MAX_COLOR_NUM = 3;

  int DELTA = 35;
  int H_[MAX_COLOR_NUM] = {};
  int S_[MAX_COLOR_NUM] = {};
  int V_[MAX_COLOR_NUM] = {};
  int selectedColor_ = 0;

  int selectedColorSquareWidth_ = 50;
  cv::Mat* outputImage = 0;

  bool needToUpdate = false;
  bool paused = false;
  
  ControlWindow *ctrlWindow_ = 0;

  public:
    virtual void mouseCallBack(int event, int x, int y, int flags);
    void updatePreviewColors();
};
#endif