//
//  Friction.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/8/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//


#include "External.hpp"

using namespace ORCA;
using namespace arma;
Friction::Friction() {
    this->setCoefficient(0);
}

Col<float> CoulombicFriction::getValue(Link* link) {
    return -this->getCoefficient()*sign(link->getDotGamma());
}

Col<float> ViscousFriction::getValue(Link* link) {
    return -this->getCoefficient()*(link->getDotGamma());
}

