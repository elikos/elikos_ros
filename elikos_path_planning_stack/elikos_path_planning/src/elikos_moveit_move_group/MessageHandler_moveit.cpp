#include "MessageHandler_moveit.h"

MessageHandler_moveit::MessageHandler_moveit()
{
   sub_ = nh_.subscribe("elikos_decisionmaking_cmd", 1, &MessageHandler_moveit::dispatchMessageTarget, this);
   hasNewMessage_ = false;
}

MessageHandler_moveit::~MessageHandler_moveit()
{
}

void MessageHandler_moveit::dispatchMessageTarget(const elikos_main::AICmd::ConstPtr &input)
{
  input_ = *(input);
  hasNewMessage_ = true;
}

elikos_main::AICmd MessageHandler_moveit::getAICmd()
{
  return input_;
}
bool MessageHandler_moveit::hasNewMessage()
{
  bool hadNewMessage = hasNewMessage_;
  hasNewMessage_ = false;
  return hadNewMessage;
}
