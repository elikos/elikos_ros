/*******************************************************************************
* Interface à implémenter afin de supporter la calibration à distance
*******************************************************************************/
#ifndef RC_CLIENT_CALIBRATABLE_H
#define RC_CLIENT_CALIBRATABLE_H

#include <yaml-cpp/yaml.h>

namespace remote_calib{
template<typename Msg>
class Calibrator;

template<typename Msg>
class Calibratable{
public:
    //Méthode qui devrait calibrer instatanément le noeud
    virtual void calibrate(const Msg* const message) = 0;
    
    //Méthode qui devrait charger la configuration passée en paramètres dans le
    // calibrable.
    virtual void loadCalibration(const YAML::Node& fileContent) = 0;

    //Méthode qui devrait enregistrer sa calibration dans le noeud fourni
    virtual void saveCalibration(YAML::Node& fileContent) = 0;

    bool isCalibrating = false;

    //Charge une calibration localement et à distance.
    void loadRemoteCalibration(const Msg& calibration);
protected:
    //Référence vers le calibrateur
    Calibrator<Msg>* calibrator_;
    friend class Calibrator<Msg>;
};


}



//==============================================================================
//==============================================================================
//                         Définition des templates
//==============================================================================
//==============================================================================
#include "Calibrator.h"

namespace remote_calib{

/******************************************************************************
* Charge la calibration sur ce noeud-ci et tout les noeuds qui écoutent. Utile 
* a appeler dans loadCalibration. Va appeler calibrate() au prochain spin de 
* ros.
*
* @param calibration    [in] la calibration a charger
******************************************************************************/
template<typename Msg>
void Calibratable<Msg>::loadRemoteCalibration(const Msg& calibration)
{
    if (calibrator_ != nullptr) {
        calibrator_->loadRemoteCalibration(calibration);
    }
}
}
#endif
