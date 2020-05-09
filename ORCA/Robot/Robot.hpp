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

using namespace arma;

namespace ORCA {
class Link;
class RootLink;

class Robot {
protected:
    
    /* Below are constant parameters for a robot           */
    /* These should be nulled and zeroed on initalization  */
    /* and can Be set by the public and protected dsetters */
    
    RootLink* __rootLink;                                   //Root Link for the robot
    
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
    
    Mat<float> getSystemMassMatrix() {
        return this->__SystemMassMatrix;
    }
    
    Col<float> getVectorOfCorCent() {
        return this->__vecOfCorCent;
    }
    
    /* Below are all of the public setters for the Robot class */
    
    void setGamma(Col<float> gamma) {
        this->__gamma = gamma;
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
    

};

}

#endif /* Robot_hpp */
