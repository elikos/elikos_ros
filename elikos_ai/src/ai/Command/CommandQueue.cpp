#include "CommandQueue.h"
#include "CommandTypes.h"

namespace ai
{

void CommandQueue::push(std::unique_ptr<AbstractCommand> command)
{
    q_.push_back(std::move(command));
}

void CommandQueue::pop()
{
    q_.pop_front();
}

AbstractCommand* CommandQueue::back()
{
    return q_.back().get();
}
AbstractCommand* CommandQueue::front()
{
    return q_.front().get();
}

void CommandQueue::clear()
{
    q_.clear();
}

bool CommandQueue::empty()
{
    return q_.empty();
}

}
