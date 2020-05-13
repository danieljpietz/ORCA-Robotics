//
//  Force.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/8/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include "../ORCA"


using namespace arma;
using namespace ORCA;

Force::Force() {
    this->setPosition(Col<float>(3, fill::zeros));
    this->setWorldValue(Col<float>(3,fill::zeros));
    this->setCoordinateFrame(Local);
}

void Force::getValue(Link* link) {
    Col<float> localPosition;
    if(__positionFrame == Local) {
        localPosition = this->getPosition(link);
    }
    else {
        localPosition = this->getPosition(link) - link->getOffsetGlobal();
    }
    
    Col<float> torque = skew(localPosition) * (link->getRotationMatrixGlobal().t())*this->getWorldValue(link);
    
    
    link->getRobot()->addToForceVector(-link->getJacobian().t() * join_vert(torque, this->getWorldValue(link)));
    
}


