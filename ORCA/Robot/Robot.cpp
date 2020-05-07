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
    __DOF += link->getDOF();
}

/**
 Adds a new link to the robot
 @param newLink New Link to be added
 @param parentLink The link to attach the new link to
 */

void Robot::addLink(Link* newLink, Link* parentLink) {
    parentLink->attatchLink(newLink);
    this->__DOF += newLink->getDOF();
    this->startLinkDimensionUpdates();
    newLink->setRobot(this);
}

/**
 Updates the dimensions of links in the bot to match the robots DOF
 */

void Robot::startLinkDimensionUpdates() {
    this->getRootLink()->__updateDOFDimensions(this->getDOF());
}
