/*******************************************************************************
* Interface à implémenter afin de supporter la calibration à distance
*******************************************************************************/
#ifndef RC_CLIENT_CALIBRATABLE_H
#define RC_CLIENT_CALIBRATABLE_H

#include <yaml-cpp/yaml.h>

namespace remote_calib{

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
};


}

#endif
