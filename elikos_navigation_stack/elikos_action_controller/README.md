# ELIKOS_ACTION_CONTROLLER 

## À savoir!
* Ce package n'est utilisé que pour que la navigation stack compile.
* L'utilisation de ce package pour effectuer le parsing de la trajectoire entraîne des problèmes de latence.  
Le parsing a donc été implémenté dans le node **elikos_moveit_move_group** du package **elikos_nav**.
* Le node **elikos_action_controller** est lancé dans `nav.launch` du package **elikos_nav**.

## Buts  
* Réception de la trajectoire générée et envoie de commande de position à mavros. 

## Output tf frame  
* `elikos_setpoint`    
    * desc : Commande de position pour mavros.  