#include "MessageHandler_ttf.h"



MessageHandler_TTF::MessageHandler_TTF()
{
	sub_ = nh_.subscribe("robot_raw_array", 1, &MessageHandler_TTF::dispatchMessage, this);
}


MessageHandler_TTF::~MessageHandler_TTF()
{
}


void MessageHandler_TTF::dispatchMessage(const elikos_ros::RobotRawArray::ConstPtr& input)
{
		//Affichage à la console pour tester
		std::cout<<"input tracking_tf size: "<<input->robots.size()<<std::endl;
}
