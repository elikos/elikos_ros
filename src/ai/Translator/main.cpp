#include "Translator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tf_translator");
    Translator::getInstance()->lookForTf();
    return 0;
}
