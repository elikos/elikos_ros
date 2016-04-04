#ifndef AI_FACADE_H
#define AI_FACADE_H

#include <memory>
#include "AbstractState.h"

class AIFacade
{
public:
    static AIFacade* getInstance();
    static void freeInstance();
    ~AIFacade();

private:
    AIFacade();
    static std::unique_ptr<AIFacade> instance_;

    std::unique_ptr<AbstractState> currentState_; 
};

#endif /// AI_FACADE_H
