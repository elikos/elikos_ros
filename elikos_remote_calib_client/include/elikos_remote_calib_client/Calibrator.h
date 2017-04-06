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
#include <cstdio>

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

#include <elikos_remote_calib_client/FileManipulation.h>
#include <elikos_remote_calib_client/GetConfigFiles.h>

#include "elikos_remote_calib_client/Calibratable.h"

namespace remote_calib{

static const std::string REMOTE_CALIB_NAMESPCACE = "elikos_remote_calib";
static const std::string MESSAGE_TOPIC_NAME = "calib";
static const std::string SAVE_SERVICE_NAME = "save";
static const std::string GET_FILE_NAME_SERVICE_NAME = "get_config_files";
static const std::string LOAD_SERVICE_NAME = "load";
static const std::string DELETE_FILE_SERVICE_NAME = "delete_file";

static const std::string CONFIG_FILE_NAME = "calibrations";
static const std::string CONFIG_FILE_EXTENTION = ".yaml";

template<typename Msg>
class Calibrator{
public:
    //Constructeur
    Calibrator(Calibratable<Msg>& calibrable, const std::string& configFileDir);

    //Charge la calibration en mémoire depuis un fichier
    bool loadCalibration(const std::string& fileName);
    //Enregistre la calibration en mémoire dans un fichier
    bool saveCalibration(const std::string& fileName);
private:
    //Callback du message
    void calibrationCallback(const boost::shared_ptr<Msg const>& msgPtr);

    //Callback des services
    bool saveCallback(elikos_remote_calib_client::FileManipulation::Request& req, elikos_remote_calib_client::FileManipulation::Response& res);
    bool confCallback(elikos_remote_calib_client::GetConfigFiles::Request& req, elikos_remote_calib_client::GetConfigFiles::Response& res);
    bool loadCallback(elikos_remote_calib_client::FileManipulation::Request& req, elikos_remote_calib_client::FileManipulation::Response& res);
    bool deleteFileCallback(elikos_remote_calib_client::FileManipulation::Request& req, elikos_remote_calib_client::FileManipulation::Response& res);

    //Ajoute un fichier au métafichier de configuration
    bool addToCalibrationMetafile(const std::string& filename);
    //Enlève un fichier du métafichier de configuration
    bool removeFromCalibrationMetafile(const std::string& filename);
    //Vérifie si un fichier est dans le métafichier de configuration
    bool isInCalibrationMetafile(const std::string& filename);
    //Sauvegarde les paramètres dans le fichier de configuration
    void saveToCalibMetaFile();
    //Une sorte de timeout pour savoir si on est en train d'être calibré
    void updateCalibratingStatus(const ros::TimerEvent &);

    YAML::Node fileList_;

    ros::NodeHandle nodeHandle_;
    ros::Subscriber calibrationSubscriber_;

    ros::Timer updateCallback_;

    ros::ServiceServer saveService_;
    ros::ServiceServer loadService_;
    ros::ServiceServer configService_;
    ros::ServiceServer deleteFileService_;

    std::string configFileDir_;

    Calibratable<Msg>& calibratable_;
private:
    static constexpr double PING_DELAY = 1.0;
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
    deleteFileService_ = nodeHandle_.advertiseService(DELETE_FILE_SERVICE_NAME, &Calibrator<Msg>::deleteFileCallback, this);

    updateCallback_ = nodeHandle_.createTimer(ros::Duration(PING_DELAY), &Calibrator<Msg>::updateCalibratingStatus, this);
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
bool Calibrator<Msg>::saveCallback(elikos_remote_calib_client::FileManipulation::Request& req, elikos_remote_calib_client::FileManipulation::Response& res)
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
bool Calibrator<Msg>::loadCallback(elikos_remote_calib_client::FileManipulation::Request& req, elikos_remote_calib_client::FileManipulation::Response& res)
{
    res.sucess = loadCalibration(req.fileName);
    return true;
}

/*******************************************************************************
* Méthode de callback du service de sauvegarde de calibration. Appelée losqu'une
* commande pour effacer un fichier a été envoyée.
*******************************************************************************/
template<typename Msg>
bool Calibrator<Msg>::deleteFileCallback(elikos_remote_calib_client::FileManipulation::Request& req, elikos_remote_calib_client::FileManipulation::Response& res)
{
    std::string fileName = req.fileName;
    if(isInCalibrationMetafile(fileName)){
        res.sucess = (remove((configFileDir_ + fileName).c_str()) == 0);
        if(res.sucess){
            removeFromCalibrationMetafile(fileName);
        }
    }else{
        res.sucess = false;
    }
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
    return true;
}

/*******************************************************************************
* Enlève un fichier de calibration du méta-fichier de calibration.
* 
* @param filename   [in] le fichier à enlever au fichier de configuration
*
* @return vrai si le nom a réussi à être enlevé, faux sinon
*******************************************************************************/
template<typename Msg>
bool Calibrator<Msg>::removeFromCalibrationMetafile(const std::string& filename)
{
    YAML::Node newNode(YAML::NodeType::value::Sequence);
    bool removed = false;

    for(int i = 0; i < fileList_.size(); ++i){
        std::string name = fileList_[i].as<std::string>();
        
        if(name == filename){
            removed = true;
        }else{
            newNode.push_back(name);
        }
    }
    fileList_ = newNode;
    saveToCalibMetaFile();
    return removed;
}

/*******************************************************************************
* Vérifie la présence d'un fichier dans le méta-fichier de calibration.
* 
* @param filename   [in] le fichier à chercher
*
* @return vrai si le nom a réussi à être trouvé, faux sinon
*******************************************************************************/
template<typename Msg>
bool Calibrator<Msg>::isInCalibrationMetafile(const std::string& filename)
{
    for(auto it = fileList_.begin(); it != fileList_.end(); ++it){
        if(it->as<std::string>() == filename){
            return true;
        }
    }
    return false;
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

/*******************************************************************************
* Appelée par un timer de ros. La méthode sert a des fins d'optimisation. Elle
* permet de savoir (dans le calibrable) si un calibrateur y est connecté.
*
* @param event  [in] Un TimerEvent provenant de ros. Permet d'enregistrer la 
*                       méthode comme callback
*******************************************************************************/
template<typename Msg>
void Calibrator<Msg>::updateCalibratingStatus(const ros::TimerEvent & event)
{
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    bool sucess = ros::master::execute("getSystemState", args, result, payload, true);

    if(!sucess){
        return;
    }


    std::string ourTopicName = 
        ros::names::append(
            nodeHandle_.getNamespace(),
            MESSAGE_TOPIC_NAME
        );

    

    XmlRpc::XmlRpcValue publisherList = payload[0];
    bool isCalibrating = false;
    for(int i = 0; i < publisherList.size(); ++i){
        std::string topicName = publisherList[i][0];
        if(topicName == ourTopicName){
            //Vrai si au moins un noeud nous calibre
            isCalibrating = publisherList[i][1].size() > 0;
            break;
        }
    }
    calibratable_.isCalibrating = isCalibrating;

}

}//end namespace
#endif
