#include "Translator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tf_translator");
    ai::Translator::getInstance()->lookForTf();
    ai::Translator::freeInstance(); 
    return 0;
}
