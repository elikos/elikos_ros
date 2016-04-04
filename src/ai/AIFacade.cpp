#include "AIFacade.h"
#include <memory>

std::unique_ptr<AIFacade> AIFacade::instance_{nullptr};

AIFacade::AIFacade()
{
}


AIFacade::~AIFacade()
{
}


AIFacade* AIFacade::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = std::unique_ptr<AIFacade>(new AIFacade());
    }
    return instance_.get();
}


void AIFacade::freeInstance()
{
    instance_ = nullptr;
}
