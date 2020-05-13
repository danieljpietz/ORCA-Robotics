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

void CoulombicFriction::getValue(Link* link) {
    
    int i;
    Col<float> friction = -this->getCoefficient()*sign(link->getDotGamma());
    Col<float> val = Col<float>(link->getRobot()->getDOF(), fill::zeros);
    for (i = 0; i < link->getDOF(); i++) {
        val[link->getGammaIndex()[i]] = friction[i];
    }
    link->getRobot()->addToForceVector(val);
}

void ViscousFriction::getValue(Link* link) {
    int i;
       Col<float> friction = -this->getCoefficient()*(link->getDotGamma());
       Col<float> val = Col<float>(link->getRobot()->getDOF(), fill::zeros);
       for (i = 0; i < link->getDOF(); i++) {
           val[link->getGammaIndex()[i]] = friction[i];
       }
       link->getRobot()->addToForceVector(val);
}

