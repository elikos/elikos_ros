//
// Created by elikos on 29/07/16.
//

#include "ResearchBehavior.h"
#include "AbstractArena.h"
#include "CommandTypes.h"

namespace ai
{

ResearchBehavior::ResearchBehavior(AbstractArena* arena)
    : AbstractBehavior(arena)
{
   
   q_.push(std::unique_ptr<TakeOffCommand>(new TakeOffCommand(&arena_->getQuad(), &tf_listener_)));
}

ResearchBehavior::~ResearchBehavior()
{
}

void ResearchBehavior::generateCommands()
{
    ros::NodeHandle nh;
    double dimension_c;
    nh.getParam("/elikos_ai/dimension_c", dimension_c);
    double research_altitude;
    nh.getParam("/elikos_ai/research_altitude", research_altitude);

    if (q_.empty()) {
    	q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0,  0.0, research_altitude })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0,  0.6*(dimension_c/2), research_altitude })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), { -0.6*(dimension_c/2),  0.0, research_altitude })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.6*(dimension_c/2),  0.0, research_altitude })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0, -0.6*(dimension_c/2), research_altitude })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0,  0.6*(dimension_c/2), research_altitude })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.6*(dimension_c/2),  0.0, research_altitude })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), { -0.6*(dimension_c/2),  0.0, research_altitude })));
        q_.push(std::unique_ptr<MovementCommand>(new MovementCommand(&arena_->getQuad(), {  0.0, -0.6*(dimension_c/2), research_altitude })));
    }
}

int ResearchBehavior::resolveCurrentStateLevelConcrete()
{
    int stateLevel = 2;
    if (arena_->getNbrOfUpdatedTargets() > 0) {
        stateLevel = 1;
    }
    return stateLevel;
}

}

