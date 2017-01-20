# ELIKOS_PATH_PLANNING_STACK

## Buts
* Évitement d'obstacles
* Planification de trajectoires

## Stratégies  
* [moveit](http://moveit.ros.org/)  

## Launch files  
* `path_planning.launch`  
*Paramètres*  
    * aucun.  

## Packages  
* **elikos_path_planning** : Package principal. Contient le launch file pour le path planning. Reçoit les commandes de l'AI et contient le move_group (elikos_moveit_move_group) qui les envoie dans le planning pipeline de moveit.
* **elikos_moveit_config** : Package en partie auto-généré par le  MoveitSetupAssistant. Contient les configurations pour lier les senseurs 3D et les différents paramètres de moveit. Le launch file moveit_master.launch sert à lancer les différents packages de moveit.
* **elikos_moveit_controller_manager** : Custom plugin pour supporter les multi_dof_follow_joint_trajectory.
* **elikos_action_controller** : Reçoit les actions multi_dof_follow_joint_trajectory produites par moveit et broadcast les tf::Transforms représentant les points à franchir dans la trajectoire. La logique d'envoi des tf est dans src/elikos_actionController.cpp. Ce package n'est pas utilisé présentement.

## Input topics  
* `elikos_ai_cmd`  
    * type : `geometry_msgs::PoseStamped`  
    * desc : commande de position envoyée par l'AI. 

## Output topic  
* `elikos_trajectory`  
    * type : `elikos_ros::TrajectoryCmd`    
    * desc : Commande et trajectoire à destination du contrôle.  

## À savoir!
* Plusieurs parties du code sont librement inspirées du repository suivant : [https://github.com/AlessioTonioni/Autonomous-Flight-ROS](https://github.com/AlessioTonioni/Autonomous-Flight-ROS)
