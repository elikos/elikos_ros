Elikos_ROS
===

Package ROS "Elikos_ROS" contenant les différents modules servant à la localisation, à la détection d'obstacles et de cibles et à l'autonomie du quadricoptère.


##Wut is dis

Le but ultime de ce repo est de pouvoir le cloner directement sur le board de dev et tout le code devraient compiler sans problème. Il est donc important de porter une attention particulière à ce qui ce ramasse ici, en portant une attention particulière au différent Makefile.


##Prérequis

Comprendre l'utilisation de ROS (catkin, package, node, etc.): http://wiki.ros.org/ROS/Tutorials
Comprendre l'utilisation de Git (branch, fork, commit, pull/push request)

##Méthodologie

  - Faire un **fork** du repo
  - Travailler sur son fork
  - **Tester** son fork sur **Ninon** et sur le board
  - Faire un **Pull request**
  - ???
  - Profit
  
Cette façons de faire est pour s'assurer que seul le code **nécessaire** se retrouve sur le quad.

##Librairie externe et autre

Il est important de discuter avec l'équipe avant d'utiliser une nouvelle librairie pour le projet. Il faut s'assurer que la librairie est bien compatible avec le board et le code existant. Des variables d'environnement doivent être utilisées dans les makefile pour éviter que chaque personne se retrouve avec un makefile propre à ça machine.
