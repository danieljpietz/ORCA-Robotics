//
//  Simulator.hpp
//  ORCA
//
//  Created by Daniel Pietz on 5/9/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#ifndef Simulator_hpp
#define Simulator_hpp

#include "../ORCA"
#include "../ORCAMath/ORCAMath.hpp"
#include <armadillo>

#define ORCA_SIM_DEFAULT_SOLVER RungeKutta4
#define ORCA_SIM_DEFAULT_STEP_SIZE 0.005

namespace ORCA {

class Robot;
class Link;
class Force;

class Simulator {
protected:
    Robot* __bot;
    float __stepSize;
    ODESolver<Col<float> >* __solver;
public:
    /**
    Default constructor for Simulator Class. Default solver is  4-th Order runge kutta witha step size of 0.005
     */
    Simulator();
    
    /**
        Constructor for Simulator with a specified robot
     */
    
    Simulator(Robot* bot);
    
    /* Below are the public getters for the Simulator Class */
    Robot* getRobot() {
        return this->__bot;
    }
    
    float getStepSize() {
        return this->__stepSize;
    }
    
    ODESolver<Col<float> >* getSolver() {
        return this->__solver;
    }
    
    /* Below are the public setters for the Simulator class*/
    
    void setRobot(Robot* bot) {
        this->__bot = bot;
    }
    
    void setStepSize(float size) {
        this->__stepSize = size;
    }
    
    void setSolver(ODESolver<Col<float> >* solver) {
        this->__solver = solver;
    }
    
    /* Below are the public member functions for the Simulator class */
    
    virtual void step();
};
}
#endif /* Simulator_hpp */
