#include "CmdStandBy.h"

CmdStandBy::CmdStandBy(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)
{
}

void CmdStandBy::execute()
{
    // TODO: Assurer le maintien de la position ici.
}

void CmdStandBy::abort()
{

}

void CmdStandBy::ajustement()
{

}