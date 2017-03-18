/*******************************************************************************
* Classe qui permet de facilement gérer la calibration (et les fichiers de 
* calibration) d'un noeud ros.
*******************************************************************************/
#ifndef RC_CLIENT_CALIBRATOR_H
#define RC_CLIENT_CALIBRATOR_H

#include <string>
#include <typeinfo>
#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

#include <elikos_remote_calib_client/SaveConfig.h>
#include <elikos_remote_calib_client/GetConfigFiles.h>
#include <elikos_remote_calib_client/LoadConfig.h>

#include "elikos_remote_calib_client/Calibratable.h"

namespace remote_calib{

static const std::string REMOTE_CALIB_NAMESPCACE = "elikos_remote_calib";
static const std::string MESSAGE_TOPIC_NAME = "calib";
static const std::string SAVE_SERVICE_NAME = "save";
static const std::string GET_FILE_NAME_SERVICE_NAME = "get_config_files";
static const std::string LOAD_SERVICE_NAME = "load";

static const std::string CONFIG_FILE_NAME = "calibrations";
static const std::string CONFIG_FILE_EXTENTION = ".yaml";

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
    //Callback du message
    void calibrationCallback(const boost::shared_ptr<Msg const>& msgPtr);

    //Callback des services
    bool saveCallback(elikos_remote_calib_client::SaveConfig::Request& req, elikos_remote_calib_client::SaveConfig::Response& res);
    bool confCallback(elikos_remote_calib_client::GetConfigFiles::Request& req, elikos_remote_calib_client::GetConfigFiles::Response& res);
    bool loadCallback(elikos_remote_calib_client::LoadConfig::Request& req, elikos_remote_calib_client::LoadConfig::Response& res);

    //Ajoute un fichier au métafichier de configuration
    bool addToCalibrationMetafile(const std::string& filename);
    //Sauvegarde les paramètres dans le fichier de configuration
    void saveToCalibMetaFile();

    YAML::Node fileList_;

    ros::NodeHandle nodeHandle_;
    ros::Subscriber calibrationSubscriber_;

    ros::ServiceServer saveService_;
    ros::ServiceServer loadService_;
    ros::ServiceServer configService_;

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

    saveService_ = nodeHandle_.advertiseService(SAVE_SERVICE_NAME, &Calibrator<Msg>::saveCallback, this);
    configService_ = nodeHandle_.advertiseService(GET_FILE_NAME_SERVICE_NAME, &Calibrator<Msg>::confCallback, this);
    loadService_ = nodeHandle_.advertiseService(LOAD_SERVICE_NAME, &Calibrator<Msg>::loadCallback, this);


    try {
        fileList_ = YAML::LoadFile(configFileDir_ + CONFIG_FILE_NAME + CONFIG_FILE_EXTENTION);
    } catch ( YAML::BadFile exeption ) {
        std::cerr << "Warning, configuration file " << configFileDir_ + CONFIG_FILE_NAME + CONFIG_FILE_EXTENTION << "\ncould not be read. Creating it now." << std::endl;
        fileList_ = YAML::Node(YAML::NodeType::Sequence);
        saveToCalibMetaFile();
    }
    if(fileList_.Type() != YAML::NodeType::Sequence){
        std::cerr << "Warning, configuration file " << configFileDir_ + CONFIG_FILE_NAME + CONFIG_FILE_EXTENTION << "\nwas misformed. Fixing it now." << std::endl;
        fileList_ = YAML::Node(YAML::NodeType::Sequence);
        saveToCalibMetaFile();
    }
}



/*******************************************************************************
* Charge une calibration depuis un nom de fichier (relatif au répertoire 
* contenant le fichier configurations.yaml). Le fichier changé est passé au
* calibrable attaché.
*
* @param fileName   [in] le répertoire du fichier a partir du dossier contenant 
*                         configurations.yaml
*
* @return si tout c'est bien passé
*******************************************************************************/
template<typename Msg>
bool Calibrator<Msg>::loadCalibration(const std::string& fileName)
{
    try {
        YAML::Node config = YAML::LoadFile(configFileDir_ + fileName);
        calibratable_.loadCalibration(config);
        return true;
    } catch ( YAML::BadFile exeption ) {
        std::cerr << "Il y a eu une exception lors du chargement de paramètres.\n"
            << "Le nom du ficher était : " << fileName << "\n" << std::endl;
        return false;
    }
}


/*******************************************************************************
* Enregistre la configuration actuelle du calibrable dans le fichier de 
* configuration dont le nom est passé en paramètres.
*
* @param fileName   [in] le nom du ficher de configuration dans lequel 
*                         enregistrer la configuration
*
* @return si la configuration a été entrgistré correctement
*******************************************************************************/
template<typename Msg>
bool Calibrator<Msg>::saveCalibration(const std::string& fileName)
{
    if(fileName == ""){
        return false;
    }
    try {
        YAML::Node config;
        calibratable_.saveCalibration(config);

        std::ofstream file(configFileDir_ + fileName);

        if ( config.Type() == YAML::NodeType::Null ) {
            file << "\n";
        } else {
            YAML::Emitter yamlOut;
            yamlOut << config;

            file << yamlOut.c_str();
        }
        file.close();

        addToCalibrationMetafile(fileName);

        return true;
    } catch ( ... ) {//TODO remplacer par le bon truc
        std::cerr << "Il y a eu une exception lors de l'enregistrement de paramètres.\n"
            << "Le nom du ficher était : " << fileName << "\n" << std::endl;
        return false;
    }
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

/*******************************************************************************
* Méthode de callback du service de sauvegarde de calibration. Appelée 
* lorsqu'une instance de calibration à besoin de sauvegarder la calibration
*******************************************************************************/
template<typename Msg>
bool Calibrator<Msg>::saveCallback(elikos_remote_calib_client::SaveConfig::Request& req, elikos_remote_calib_client::SaveConfig::Response& res)
{
    res.sucess = saveCalibration(req.fileName);
    return true;
}

/*******************************************************************************
* Méthode de callback du service de sauvegarde de calibration. Appelée 
* lorsqu'une instance de calibration à besoin de connaitre les nom des fichiers
* de calibration.
*******************************************************************************/
template<typename Msg>
bool Calibrator<Msg>::confCallback(elikos_remote_calib_client::GetConfigFiles::Request& req, elikos_remote_calib_client::GetConfigFiles::Response& res)
{
    res.fileNames.clear();
    for(auto it = fileList_.begin(); it != fileList_.end(); ++it) {
        res.fileNames.push_back(it->as<std::string>());
    }
    return true;
}

/*******************************************************************************
* Méthode de callback du service de sauvegarde de calibration. Appelée 
* lorsqu'une instance de calibration à besoin de charger une configuration.
*******************************************************************************/
template<typename Msg>
bool Calibrator<Msg>::loadCallback(elikos_remote_calib_client::LoadConfig::Request& req, elikos_remote_calib_client::LoadConfig::Response& res)
{
    res.sucess = loadCalibration(req.fileName);
    return true;
}

/*******************************************************************************
* Ajoute une entrée de calibration au méta-fichier de calibration.
* 
* @param filename   [in] le fichier à ajouter au fichier de configuration
*
* @return vrai si le nom a du être ajouté, faux si il était déjà là
*******************************************************************************/
template<typename Msg>
bool Calibrator<Msg>::addToCalibrationMetafile(const std::string& filename)
{
    for(auto it = fileList_.begin(); it != fileList_.end(); ++it){
        if(it->as<std::string>() == filename){
            return false;
        }
    }

    fileList_.push_back(filename);
    saveToCalibMetaFile();
}

/*******************************************************************************
* Sauvegarde les paramètres dans le méta-fichier de configuration.
*******************************************************************************/
template<typename Msg>
void Calibrator<Msg>::saveToCalibMetaFile()
{
    std::ofstream file(configFileDir_ + CONFIG_FILE_NAME + CONFIG_FILE_EXTENTION);

    YAML::Emitter yamlOut;
    yamlOut << fileList_;
    file << yamlOut.c_str();
    file.close();

}

}//end namespace
#endif