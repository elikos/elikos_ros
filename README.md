Elikos_ROS
===

Package ROS "Elikos_ROS" contenant les différents modules servant à la localisation, à la détection d'obstacles et de cibles et à l'autonomie du quadricoptère.  

[WikiElikos](https://elikos.github.io/documentation/)

## Dependencies

  Packages ROS :  
  
    sudo apt-get install -y ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras ros-$ROS_DISTRO-pointgrey-camera-driver ros-$ROS_DISTRO-moveit
    sudo apt install -y python-pip
    sudo pip install --upgrade pip
    sudo pip install numba scipy numpy numpy-quaternion`

## Wut is dis

Le but ultime de ce repo est de pouvoir le cloner directement sur le board de dev et tout le code devrait compiler sans problème. Il est donc important de porter une attention particulière à ce qui se ramasse ici, en portant une attention particulière au différent Makefile.


## Prérequis

  - Comprendre l'utilisation de ROS (catkin, package, node, etc.): http://wiki.ros.org/ROS/Tutorials
  - Comprendre l'utilisation de Git (branch, fork, commit, pull/push request)

## Méthodologie

  1. Faire un **fork** du repo et checkout dans son workspace
	1. Set upstream au repo de l'équipe  
		`git remote add upstream git@github.com:elikos/elikos_ros.git`
  2. Travailler sur son fork
  3. **Tester** son fork sur le **desktop dans le local** et sur le board
  4. Faire un **Pull request**
  
Cette façon de faire est pour s'assurer que seul le code **nécessaire** se retrouve sur le quad.

## Librairie externe et autre

Il est important de discuter avec l'équipe avant d'utiliser une nouvelle librairie pour le projet. Il faut s'assurer que la librairie est bien compatible avec le board et le code existant. Des variables d'environnement doivent être utilisées dans les makefile pour éviter que chaque personne se retrouve avec un makefile propre à sa machine.
