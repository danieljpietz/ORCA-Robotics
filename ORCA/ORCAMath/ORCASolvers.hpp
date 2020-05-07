//
//  ORCASolvers.hpp
//  ORCA
//
//  Created by Daniel Pietz on 5/5/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#ifndef ORCASolvers_hpp
#define ORCASolvers_hpp

#include <functional>
#include <cstdarg>
#include <math.h>


#include "ORCAMath.hpp"

#define ORCA_DEFAULT_STEP_SIZE (0.05)
#define ORCA_INT_MAX 10

namespace ORCA {

//TODO: Set better reference flags for documentation

/**
* Superclass for all Euler-derived ODE Solvers
* if no specific method is overriden, the 1st order
* Euler's method will be used as specified:
* https://en.wikipedia.org/wiki/Euler_method
*/

template <class T>
class ODESolver {
protected:
    
public:
    /**
     *Default Constructor for ODE Solver.
     */
    ODESolver() {
    }
    /**
     * Calculate a single step forward of the function using a first order Euler method
     * @param derivative Function pointer to derivative of ODE input state
     * @param currentState The current state of the system
     * @param stepSize Width of integration step
     */
    virtual T step(std::function<T(T)> derivative, T currentState, float stepSize) {
        return currentState + (stepSize * derivative(currentState)); //Iterate using Euler's Method
        
    }
    
    /**
    * Approximate the ODE over the given domain
    * @param derivative Function pointer to derivative of ODE input state
    * @param initialConditions The initial state of the system
    * @param startTime Lower bound for integration domain
    * @param stepSize Width of Integration Step
    * @param endTime Upper bound for integration domain
    */
    
    virtual std::vector<T> solve(std::function<T(T)> derivative, T initialConditions, float startTime, float endTime, float stepSize) {
        
        assert(startTime > endTime);
        assert(stepSize > 0);
        
        // Calculate the size of the output
        int subdivisionCount = ceil((endTime - startTime) / stepSize);
        
        //Initalize the output vector
        
        std::vector<T> result(subdivisionCount);
        
        //Set Initial Conditions
        result[0] = initialConditions;
        
        //Step through the timespan iterating with the 'step' function
        int i;
        for (i = 0; i < subdivisionCount; i++) {
            result[i+1] = this->step(derivative, result[i], stepSize);
        }
        
        return result;
    }
    
};

/**
* 4th Order Runge-Kutta solver.
* Subclass of ORCA::ODESolver
* Runge-Kutta  will be used as specified:
* https://lpsa.swarthmore.edu/NumInt/NumIntFourth.html
*/

template <class T>
class RungeKutta4 : public ODESolver<T> {
public:
    /**
     *Default Constructor for 4-the order Runge-Kutta ODE Solver.
     */
    RungeKutta4() {
    }
    
    /**
    * Calculate a single step forward of the function using a 4-th Order
    * Runge Kutta Integration
    * @param derivative Function pointer to derivative of ODE input state
    * @param currentState The current state of the system
    * @param stepSize Width of integration step
    */
    
    T step(std::function<T(T)> derivative, T currentState, float stepSize) override {
        T k1 = derivative(currentState);
        T k2 = derivative(currentState + (k1 * stepSize / 2));
        T k3 = derivative(currentState + (k2 * stepSize / 2));
        T k4 = derivative(currentState + (k3 * stepSize));
        
        return (currentState + (stepSize * ((k1 / 6) + (k2 / 3) + (k3 / 3) + (k4 / 6))));
    }
    
};

}
#endif /* ORCASolvers_hpp */
