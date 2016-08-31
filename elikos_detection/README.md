# ELIKOS_DETECTION  
## Buts  
* Détection des robots cibles.  
* Calcul de leur position.  

## Stratégies  
* [Blob detection](https://www.google.ca/).  
* [TF transforms](http://wiki.ros.org/tf).  

## Launch files  
* `elikos_detection.launch`  
*Paramètres*  
    * `camera_name` : nom de la camera utilisée.  
    * `cfg` : nom du fichier de configuration utilisé.  
    
## Configuration   
* *Paramètres*  
    * `topic` : spécifie le topic sur lequel sont publiées les images de la caméra.  
    * `frame` : spécifie la frame tf représentant la position de la caméra.  
    * `calibration` : si `true`, lance l'outil de calibration.  
    * `file_in` : spécifie le nom du fichier de calibration initial.  
    * `file_out` : spécifie le nom du fichier de calibration généré par l'outil.  
    * `dir` : spécifie l'emplacement du fichier de calibration généré par l'outil.  
    * `cam_height` : spécifie la composante verticale de la résolution des images.  
    * `cam_width` : spécifie la composante horizontale de la résolution des images.  
    * `cam_fov_v` : spécifie la composante verticale du champs de vision de la caméra.  
    * `cam_fov_h` : spécifie la composante horizontale du champs de vision de la caméra.  
    * `min_height` : spécifie la hauteur minimale à laquelle le drone doit voler pour tenir compte des résultats de détection (en mètres).  

## Nodes  
* **elikos_detection** : Effectue le blob detection.  
* **elikos_transformation** : Transforme les coordonnées du repère de l'image vers le repère de l'arène.  

## Input topics  
* `(topic spécifié dans le fichier de configuration)`  
    * type : image couleur  
    * desc : images provenant des caméras.  

## Output topics  
* `elikos_target_robot_array`  
    * type : `elikos_ros::TargetRobotArray`  
    * desc : résultats du package.  
* `(topic spécifié dans le fichier de configuration)/debug`  
    * type : image couleur avec annotations  
    * desc : affichage de débogage pour la calibration. Des cercles de la couleur détectée sont ajoutés sur les blobs.  


## À savoir!  
* Plusieurs intances du package peuvent être lancées à la fois. (Une par caméra.)