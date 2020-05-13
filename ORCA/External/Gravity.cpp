//
//  Gravity.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/13/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include "External.hpp"

using namespace ORCA;
using namespace arma;

Col<float> Gravity::getPosition(Link* link) {
    return link->getFirstMassMoment()/link->getMass();
}

Col<float> Gravity::getWorldValue(Link* link)  {
    return link->getMass() * this->__worldValue;
}
