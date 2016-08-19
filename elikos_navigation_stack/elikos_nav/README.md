# ELIKOS_NAV 
## Buts  
* Package principal de la navigation stack. 
* Recevoir les commandes de l'AI et les envoyer dans le planning pipeline de moveit.
* Obtenir la trajectoire à exécuter de moveit et envoyer une commande de position à mavros.

## Launch files  
* `nav.launch`  
*Description*  
    * Sert à lancer tous les éléments de la navigation stack.  

## Nodes  
* **elikos_moveit_move_group** : Node central pour utiliser moveit. 

## Input topics  
* `elikos_ai_cmd`  
    * type : `geometry_msgs::PoseStamped`  
    * desc : commande de position envoyée par l'AI. 

## Output tf frame  
* `elikos_setpoint`    
    * desc : Commande de position pour mavros.  