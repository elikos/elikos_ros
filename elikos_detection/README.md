# ELIKOS_DETECTION  
## Buts  
* Détection des robots cibles.  
* Calcul de leur position.  

## Stratégies  
* Blob detection.  
* TF transforms.  

## Launch files  
* `detection.launch`  
*Paramètres*  
    * `calib` : si `true`, lance l'outil de calibration.  
    * `calib_in` : spécifie le nom du fichier de calibration initial.  
    * `calib_out` : spécifie le nom du fichier de calibration généré par l'outil.  
    * `calib_dir` : spécifie l'emplacement du fichier de calibration généré par l'outil.  
    * `image` : si `true`, lance rqt_image_view.  
    * `rviz` : si `true`, lance RViz.

## Nodes  
* detection : Effectue le blob detection.  
* transformation : Transforme les coordonnées du repère de l'image vers le repère de l'arène.  

## Input topics  
* `/cam1/camera/image_raw`  
    * type : image couleur  
    * desc : images provenant des caméras.  

## Output topics  
* `elikos_target_robot_array`  
    * type : `elikos_ros::TargetRobotArray`  
    * desc : résultats du package.  
* `camera_test/image_opencv`  
    * type : image couleur avec annotations  
    * desc : affichage de débogage pour la calibration. Des cercles de la couleur détectée sont ajoutés sur les blobs.  