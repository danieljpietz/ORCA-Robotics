//
//  Robot.hpp
//  ORCA
//
//  Created by Daniel Pietz on 5/7/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#ifndef Robot_hpp
#define Robot_hpp

#include "../ORCAMath/ORCAMath.hpp"
#include "../ORCA"
#include <functional>
#include <cstdio>

using namespace arma;

namespace ORCA {
class Link;
class RootLink;
template <class T>
class ODESolver;
class Robot;

void __step(Robot* bot);


class Robot {
    
protected:
    
    /* Below are constant parameters for a robot           */
    /* These should be nulled and zeroed on initalization  */
    /* and can Be set by the public and protected dsetters */
    
    RootLink* __rootLink;                                   // Root Link for the robot
    ODESolver<Col<float> >* __solver;                       // Solver for simulation the robot
    float __stepSize;
    
    /* Below are runtime constants for a robotic link     */
    /* These should be nulled and zeroed on initalization */
    /* and updated when changes are made to the robot     */
    
    int __DOF;                                                // Degrees of freedom for the robot
    
    /* Below are the varying parameters of a robotic link */
    /* These should be nulled at zeroed on initalization  */
    /* and updated when the robot is updated              */
    
    Col<float> __gamma;                                     // All joint values for the robot
    Col<float> __dotgamma;                                  // All joint velocities for the robot
    Mat<float> __SystemMassMatrix;                          // System Mass Matrix for the robot
    Col<float> __vecOfCorCent;                              // Vector for Coriolis and Centreptial terms for the robot
    Col<float> __forceVector;                               // Vector of external forces acting on the system
    
    /* Below are the protected setters of the Robot class */
    
    void setDOF(int DOF) {
        this->__DOF = DOF;
    }
    
    /* Below are the protected member functions of the Robot class */
    
    void startLinkDimensionUpdates();                       // Begins the traversal of the link trees updating matrix dimensions
    void __getBranchMassMatrix(Link* link);                 // Traversal of the tree summing system mass matrix
    void updateSystemMassMatrix();                          // Starts the system mass matrix traversal
    void __getBranchCorCent(Link* link);                    // Traversal of the tree summing system mass matrix
    void updateVectorOfCorCent();                           // Starts the system mass matrix traversal
    
    
    
    
    
public:
    
    /* Below are all constructors for the Robot class */
    
    Robot();
    
    /* Below are all member functions for the Robot class */
    
    virtual void setRootLink(RootLink* link);               // Set the root link for the robot
    virtual void addLink(Link* newLink, Link* parentLink);  // Add a new link to the robot
    virtual void update();
    
    /* Below are all of the public getters for the Robot class */
    
    RootLink* getRootLink() {
        return this->__rootLink;
    }
    
    int getDOF() {
        return this->__DOF;
    }
    
    Col<float> getGamma() {
        return this->__gamma;
    }
    
    Col<float> getDotGamma() {
        return this->__dotgamma;
    }
    
    Col<float> getGammaState() {
        return join_vert(this->getGamma(), this->getDotGamma());
    }
    
    Mat<float> getSystemMassMatrix() {
        return this->__SystemMassMatrix;
    }
    
    Col<float> getVectorOfCorCent() {
        return this->__vecOfCorCent;
    }
    
    ODESolver<Col<float> >* getSolver() {
        return this->__solver;
    }
    
    float getStepSize() {
        return this->__stepSize;
    }
    
    Col<float> getForces() {
        return this->__forceVector;
    }
    
    /* Below are all of the public setters for the Robot class */
    
    void setGamma(Col<float> gamma) {
        this->__gamma = gamma;
    }
    
    void setGammaState(Col<float> state) {
        assert(state.n_cols == 2*this->getDOF() || state.n_rows == 2*this->getDOF());
        int i;
        for (i = 0; i < this->getDOF(); i++) {
            this->setGammaAtIndex(i, state[i]);
            this->setDotGammaAtIndex(i, state[i+this->getDOF()]);
        }
    }
    
    void setGammaAtIndex(int index, float value) {
        this->__gamma[index] = value;
    }
    
    void setDotGamma(Col<float> dotgamma) {
        this->__dotgamma = dotgamma;
    }
    
    void setDotGammaAtIndex(int index, float value) {
        this->__dotgamma[index] = value;
    }
    
    void setSolver(ODESolver<Col<float> >* solver) {
        this->__solver = solver;
    }
    
    void setStepSize(float size) {
        this->__stepSize = size;
    }
    
    
    /* Below are all of the public member functions for the Robot class */
    
    virtual Col<float> getJointAccelerations() {
        return solve(this->getSystemMassMatrix(), this->getForces() - this->getVectorOfCorCent(),
                     solve_opts::fast);
    }
    
    virtual Col<float> getJointAccelerations(Col<float> gamma, Col<float> dotgamma) {
        this->setGamma(gamma);
        this->setDotGamma(dotgamma);
        this->update();
        return this->getJointAccelerations();
    }
    
    virtual Col<float> getJointAccelerations(Col<float> vec) {
        assert(vec.size() % 2 == 0);
        int i;
        Col<float> gamma = Col<float>(vec.size()/2);
        Col<float> dotgamma = Col<float>(vec.size()/2);;
        for (i = 0; 2*i < vec.size(); i++) {
            gamma[i] = vec[i];
            dotgamma[i] = vec[i + vec.size()/2];
        }
        return this->getJointAccelerations(gamma, dotgamma);
    }
    
    virtual Col<float> getJointAccelerationsFromState(Col<float> vec) {
        return this->getJointAccelerations(vec);
    }
    
    virtual Col<float> getStateDerivative(Col<float> vec) {
        Col<float> dotgamma = Col<float>(vec.size()/2, 1, fill::zeros);
        int i;
        for (i = 0; i < vec.size()/2; i++) {
            dotgamma[i] = vec[i + vec.size()/2];
        }
        return join_vert(dotgamma, this->getJointAccelerations(vec));
    }
    
    virtual void step() {
        Col<float> currentState = this->getGammaState();
        float stepSize = this->getStepSize();
        Col<float> k1 = this->getStateDerivative(currentState);
        Col<float> k2 = this->getStateDerivative(currentState + (k1 * stepSize / 2));
        Col<float> k3 = this->getStateDerivative(currentState + (k2 * stepSize / 2));
        Col<float> k4 = this->getStateDerivative(currentState + (k3 * stepSize));
        this->setGammaState(currentState + (stepSize * ((k1 / 6) + (k2 / 3) + (k3 / 3) + (k4 / 6))));
    }
    
    virtual void addToForceVector(Col<float> vec) {
        this->__forceVector += vec;
    }
    
    virtual void updateForces();
    
    virtual void __getBranchForcesRecursive(Link* link);
    
};



}
#endif /* Robot_hpp */
