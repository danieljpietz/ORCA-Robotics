//
//  Robot.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/7/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include "Robot.hpp"

using namespace ORCA;
using namespace arma;

/**
 Updates the dimensions of links in the bot to match the robots DOF
 */

void Robot::startLinkDimensionUpdates() {
    this->getRootLink()->__updateDOFDimensions(this->getDOF());
}

void Robot::__getBranchMassMatrix(Link* link) {
    int i;
    this->__SystemMassMatrix += link->getSystemMassMatrix();
    for (i = 0; i < link->getNextLinks().size(); i++) {
        this->__getBranchMassMatrix(link->getNextLinks()[i]);
    }
}

void Robot::updateSystemMassMatrix() {
    this->__SystemMassMatrix = Mat<float>(this->getDOF(),this->getDOF(), fill::zeros);
    this->__getBranchMassMatrix(this->getRootLink());
}

void Robot::__getBranchCorCent(Link* link) {
    int i;
    this->__vecOfCorCent += link->getVectorOfCorCent();
    for (i = 0; i < link->getNextLinks().size(); i++) {
        this->__getBranchCorCent(link->getNextLinks()[i]);
    }
}

void Robot::updateVectorOfCorCent() {
    this->__vecOfCorCent = Col<float>(this->getDOF(),1, fill::zeros);
    this->__getBranchCorCent(this->getRootLink());
}

/**
Below are Public member functions for the Robot class
 */

/**
 Default constructor for Robot class
 */

Robot::Robot() {
    this->__rootLink =  (RootLink*)NULL;
    this->setDOF(0);
}

/**
 Sets the Root Link for the robot
 @param link New Root Link for the robot
 */

void Robot::setRootLink(RootLink* link) {
    this->__rootLink = link;
    link->setRobot(this);
    link->setGammaIndex(0);
    this->__DOF += link->getDOF();
    this->startLinkDimensionUpdates();
    link->setRobot(this);
    this->__gamma.resize(this->getDOF());
    this->__dotgamma.resize(this->getDOF());
    int i;
    for (i = this->getDOF(); i > this->getDOF() - link->getDOF(); i--) {
        this->setGammaAtIndex(i, 0);
        this->setDotGammaAtIndex(i, 0);
    }
}

/**
 Adds a new link to the robot
 @param newLink New Link to be added
 @param parentLink The link to attach the new link to
 */

void Robot::addLink(Link* newLink, Link* parentLink) {
    parentLink->attatchLink(newLink);
    newLink->setGammaIndex(this->getDOF());
    this->__DOF += newLink->getDOF();
    this->startLinkDimensionUpdates();
    newLink->setRobot(this);
    this->__gamma.resize(this->getDOF());
    this->__dotgamma.resize(this->getDOF());
    int i;
    for (i = this->getDOF(); i > this->getDOF() - newLink->getDOF(); i--) {
        this->setGammaAtIndex(i, 0);
        this->setDotGammaAtIndex(i, 0);
    }
}

void Robot::update() {
    this->getRootLink()->__updateBranch();
    this->updateSystemMassMatrix();
    this->updateVectorOfCorCent();
    
}
