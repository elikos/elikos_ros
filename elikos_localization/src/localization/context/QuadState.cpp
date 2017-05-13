#include "QuadState.h"

namespace localization
{

QuadState* QuadState::instance_ = nullptr;

QuadState* QuadState::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new QuadState();
    }
    return instance_;
}

void QuadState::freeInstance()
{
    delete instance_;
    instance_ = nullptr;
}

}