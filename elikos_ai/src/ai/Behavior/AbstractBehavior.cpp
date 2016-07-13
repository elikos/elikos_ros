//
// Created by olivier on 07/07/16.
//

#include "AbstractBehavior.h"

namespace ai
{



void AbstractBehavior::behave()
{
    if (!q_.empty()) {
        if (q_.front()->execute()) {
            q_.pop();
        }
    }
}

}
