#include "elikos_remote_calib_client/Calibrator.h"

#include <iostream>
#include <fstream>

namespace remote_calib{

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
bool loadCalibration(const std::string& fileName)
{
    try {
        YAML::Node config = YAML::LoadFile(configFileDir_ + fileName);
        calibratable_.loadCalibration(config);
        return true;
    } catch ( ... ) {//TODO remplacer par le bon truc
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
bool saveCalibration(const std::string& fileName)
{
    try {
        YAML::Node config;
        calibratable_.saveCalibration(config);

        YAML::Emitter out;
        out << config;

        std::ofstream file(configFileDir_ + fileName);
        file << out.c_str() << std::endl;
        file.close();
        
        return true;
    } catch ( ... ) {//TODO remplacer par le bon truc
        std::cerr << "Il y a eu une exception lors de l'enregistrement de paramètres.\n"
            << "Le nom du ficher était : " << fileName << "\n" << std::endl;
        return false;
    }
}

}