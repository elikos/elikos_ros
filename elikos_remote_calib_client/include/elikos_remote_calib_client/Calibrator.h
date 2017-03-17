/*******************************************************************************
* Classe qui permet de facilement gérer la calibration (et les fichiers de 
* calibration) d'un noeud ros.
*******************************************************************************/
#ifndef RC_CLIENT_CALIBRATOR_H
#define RC_CLIENT_CALIBRATOR_H

#include <string>
#include <typeinfo>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <elikos_remote_calib_client/SaveConfig.h>

#include "elikos_remote_calib_client/Calibratable.h"

namespace remote_calib{

static const std::string REMOTE_CALIB_NAMESPCACE = "elikos_remote_calib";
static const std::string MESSAGE_TOPIC_NAME = "calib";
static const std::string SAVE_SERVICE_NAME = "save";

static const std::string CONFIG_FILE_NAME = "calibrations";
static const std::string CONFIG_FILE_EXTENTION = "yaml";

template<typename Msg>
class Calibrator{
public:
    //Constructeur
    Calibrator(Calibratable<Msg>& calibrable, const std::string& configFileDir);

    //Charge la calibration en mémoire depuis un fichier
    bool loadCalibration(const std::string& fileName);
    //Charge la calibration en mémoire depuis un fichier
    bool saveCalibration(const std::string& fileName);
private:
    void calibrationCallback(const boost::shared_ptr<Msg const>& msgPtr);
    bool saveCallback(elikos_remote_calib_client::SaveConfig::Request& req, elikos_remote_calib_client::SaveConfig::Response& res);

    ros::NodeHandle nodeHandle_;
    ros::Subscriber calibrationSubscriber_;

    ros::ServiceServer saveService_;
    //ros::ServiceServer loadService_;
    //ros::ServiceServer configService_;

    std::string configFileDir_;

    Calibratable<Msg>& calibratable_;
};


//==============================================================================
//==============================================================================
//                         Définition des templates
//==============================================================================
//==============================================================================



/*******************************************************************************
* Constructeur de la classe Calibrateur.
* @param calibrable     [in] une référence vers le calibrable dont il faut 
*                               appeler les méthodes
* @param configFileDir  [in] Le répertoire vers le dossier contenant du fichier 
*                               de configuration calibrations.yaml. Doit finit 
*                               par '/'
*******************************************************************************/
template<typename Msg>
Calibrator<Msg>::Calibrator(Calibratable<Msg>& calibrable, const std::string& configFileDir)
    : nodeHandle_("~" + REMOTE_CALIB_NAMESPCACE)
    , calibratable_( calibrable )
    , configFileDir_(configFileDir)
{
    calibrationSubscriber_ = nodeHandle_.subscribe(MESSAGE_TOPIC_NAME, 20, &Calibrator<Msg>::calibrationCallback, this);

    saveService_ = nodeHandle_.advertiseService(SAVE_SERVICE_NAME, &saveCallback, this);
}

/*******************************************************************************
* Méthode appelée lorsqu'un message de calibration est reçu de ROS.
* @param msgPtr     [in] un pointeur vers le message de ROS
*******************************************************************************/
template<typename Msg>
void Calibrator<Msg>::calibrationCallback(const boost::shared_ptr<Msg const>& msgPtr)
{
    calibratable_.calibrate(msgPtr.get());
}

}


#endif