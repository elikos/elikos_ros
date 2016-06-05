#include "MessageHandler.h"
#include "Agent.h"
#include "CmdLineParser.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_ai");

    ai::CmdLineParser parser(argc, argv);
    parser.parse();

    ai::MessageHandler::getInstance()->lookForMessages();
}
