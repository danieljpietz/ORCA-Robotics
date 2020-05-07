//
//  RLink.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/7/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//


#include "Link.hpp"

using namespace arma;
using namespace ORCA;

RLink::RLink(axis rotationAxis) {
    this->__rotationAxes.resize(0);
    this->__rotationAxes.push_back(rotationAxis);
    this->setDOF(1);
}

RLink::RLink(std::vector<axis> rotationAxes) {
    this->__rotationAxes = rotationAxes;
    this->setDOF((int)rotationAxes.size());
}

void RLink::updateRotationMatrixLocal() {
    int i;
    Mat<float> rotationMat = Mat<float>(3, 3, fill::eye);
    for (i = 0; i < this->getRotationAxes().size(); i++) {
        switch (this->getRotationAxes()[i]) {
            case X:
                rotationMat *= rotx(this->getGamma()[i]);
                break;
                
            case Y:
                rotationMat *= roty(this->getGamma()[i]);
                break;
            case Z:
                rotationMat *= rotz(this->getGamma()[i]);
            default:
                break;
        }
    }
    this->setRotationMatrixLocal(rotationMat);
}

void RLink::update() {
    
}
