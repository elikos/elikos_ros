#include "MessageHandler.h"
#include "Agent.h"
#include "CmdLineParser.h"

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_ai");

    ai::CmdLineParser* parser = new ai::CmdLineParser(argc, argv);
    parser->parse();
    delete parser;

    ai::MessageHandler::getInstance()->lookForMessages();
}
