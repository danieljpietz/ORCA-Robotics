//
//  Simulator.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/9/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include "Simulator.hpp"

using namespace ORCA;
using namespace arma;

Simulator::Simulator() {
    ODESolver<Col<float> > *solver = new ORCA_SIM_DEFAULT_SOLVER<Col<float> >();
    this->setSolver(solver);
    this->setStepSize(ORCA_SIM_DEFAULT_STEP_SIZE);
    this->__bot = (Robot*)NULL;
}

Simulator::Simulator(Robot* bot) {
    ODESolver<Col<float> > *solver = new ORCA_SIM_DEFAULT_SOLVER<Col<float> >();
    this->setSolver(solver);
    this->setStepSize(ORCA_SIM_DEFAULT_STEP_SIZE);
    this->__bot = bot;
}


void Simulator::step() {
    //this->getSolver()->step(getJointAccelerationsWrapper, this->getRobot()->getGammaState(), this->getStepSize());
}


