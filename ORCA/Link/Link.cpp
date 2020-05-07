//
//  Link.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/6/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include "Link.hpp"

using namespace ORCA;
using namespace arma;

/**
 Default Constructor for Link. Sets all members to NULL or Zero
 */

Link::Link() {
    this->setRobot((Robot*) NULL);
    this->setPreviousLink((Link*) NULL);
    this->setMass(0);
    this->setFirstMassMoment(Col<float>(3, fill::zeros));
    this->setInertiaMatrix(Mat<float>(3, 3, fill::zeros));
    this->setLocalOffset(Col<float>(3, fill::zeros));
    this->setITilde(Mat<float>(3,3, fill::zeros));
    this->setIHat(Mat<float>(3,3, fill::zeros));
    this->setITildeBot(Mat<float>(3,3, fill::zeros));
    this->setIHatBot(Mat<float>(3,3, fill::zeros));
    this->setGamma(Col<float>(1,fill::zeros));
    this->setDotGamma(Col<float>(1,fill::zeros));
    this->setJacobian(Mat<float>(6,1,fill::zeros));
    this->setDotJacobian(Mat<float>(6,1,fill::zeros));
    this->setOmegaLocal(Col<float>(3,fill::zeros));
    this->setOmegaGlobal(Col<float>(3,fill::zeros));
    this->setDotRLocal(Col<float>(3,fill::zeros));
    this->setDotRGlobal(Col<float>(3,fill::zeros));
}

/**
 Updates dimensions of link matricies to have 'n' columns
 @param n Number of columns in matrix
 */

void Link::updateDOFDimensions(int n) {
    this->__jacobian.resize(6, n);
    this->__dotjacobian.resize(6,n);
    this->__IHatBot.resize(3,n);
    this->__ITildeBot.resize(3,n);
}
/**
Updates dimsneions of branch matricies to have 'n' columns
@param n Number of columns in matrix
*/
void Link::__updateDOFDimensions(int n) {
    this->updateDOFDimensions(n);
    int i;
    for (i = 0; i < this->getNextLinks().size(); i++) {
        this->getNextLinks()[i]->__updateDOFDimensions(n);
    }
}

/**
 Attatches a child link to this link
 @param link Child link to be attatched
 */

void Link::attatchLink(Link* link) {
    __nextLinks.push_back(link);
    link->setPreviousLink(this);
}

/**
 Recursve update of all link elements based on state of previous link
 */

void Link::update() {
    
}

/*
 Recursive update of this link and all links further down in the chain.
 */

void Link::updateBranch() {
    this->update();
    int i;
    for (i = 0; i < this->getNextLinks().size(); i++) {
        this->getNextLinks()[i]->updateBranch();
    }
}

