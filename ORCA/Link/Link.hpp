//
//  Link.hpp
//  ORCA
//
//  Created by Daniel Pietz on 5/6/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#ifndef Link_hpp
#define Link_hpp

#include "../ORCA"
#include "../ORCAMath/ORCAMath.hpp"
#include <armadillo>

using namespace arma;

namespace ORCA {

class Robot;

class Link {
    friend class Robot;
protected:
    
    /* Below are constant parameters for a robotic link    */
    /* These should be nulled and zeroed on initalization  */
    /* and can Be set by the public and protected dsetters */
    
    Robot* __robot;                         // Pointer to the robot this link is a member of
    Link* __previousLink;                   // Pointer to the link this link is attatched to higher up in the chain
    std::vector<Link*> __nextLinks;         // Pointers to the links attatched to further down in the chain
    float __mass;                           // Mass of the link in kilograms
    Col<float> __firstMassMoment;           // Vector of First Mass Moment measured from the links locael frame (kg*m)
    Mat<float> __inertiaMatrix;             // Matrix of second mass moment measured in the links local frame (kg*m^2)
    Col<float> __localOffset;               // The position of this frame in the frame of the previous link
    Mat<float> __ITilde;                    // Matrix that maps the joint velocity into the links local angular velocity
    Mat<float> __IHat;                      // Matrix that maps the joint velocity into the links local linear velocity
    Mat<float> __ITildeBot;                 // Matrix that maps the robot joint velocities into the links local angular velocity
    Mat<float> __IHatBot;                   // Matrix that maps the robot joint velocities into the links local linear velocity
    int __DOF;                              // Degrees of freedom the link has
    
    /* Below are runtime constants for a robotic link     */
    /* These should be nulled and zeroed on initalization */
    /* and updated when changes are made to the robot     */
    
    std::vector<int> __gammaIndex;          // The indexes of this link's degrees of freedom in the robot state vector
    
    /* Below are the varying parameters of a robotic link */
    /* These should be nulled at zeroed on initalization  */
    /* and updated when the robot is updated              */
    
    Col<float> __gamma;                     // The value of this link's joint angle
    Col<float> __dotgamma;                  // The value of this link's joint velocity
    Mat<float> __jacobian;                  // Matrix that maps the robot joint angles into the link angular and linear velocities
    Mat<float> __dotjacobian;               // The time-derivative of the jacobian matrix
    Col<float> __omegaLocal;                // The angular velocty of the link in the joint space
    Col<float> __omega;                     // The angular velocty of the link in the world space
    Col<float> __dotrLocal;                 // The linear velocity of the link in the joint space
    Col<float> __dotr;                      // The linear velocity of the link in the world space
    Mat<float> __rotationMatrixLocal;       // The rotation of the of the link in the space of the previous link
    Mat<float> __rotationMatrixGlobal;      // The rotation of the of the link in the world space
    Mat<float> __localMassMatrix;         // The local mass matrix for the link
    Mat<float> __systemMassMatrix;        // System Mass Matrix for the link
    Col<float> __vecOfCorCent;            // Vector for Coriolis and Centreptial terms for the link
    
    /* Below are the protected setters for the Link class */
    
    void setRobot(Robot* robot) {
        this->__robot = robot;
    }
    
    void setPreviousLink(Link* link) {
        this->__previousLink = link;
    }
    
    void setITildeBot(Mat<float> matrix) {
        this->__ITildeBot = matrix;
    }
    
    void setIHatBot(Mat<float> matrix) {
        this->__IHatBot = matrix;
    }
    
    void setGamma(Col<float> vec) {
        this->__gamma = vec;
    }
    
    void setDotGamma(Col<float> vec) {
        this->__dotgamma = vec;
    }
    
    void setJacobian(Mat<float> mat) {
        this->__jacobian = mat;
    }
    
    void setDotJacobian(Mat<float> mat) {
        this->__dotjacobian = mat;
    }
    
    void setOmegaLocal(Col<float> vec) {
        this->__omegaLocal = vec;
    }
    
    void setOmegaGlobal(Col<float> vec) {
        this->__omega = vec;
    }
    
    void setDotRLocal(Col<float> vec) {
        this->__dotrLocal = vec;
    }
    
    void setDotRGlobal(Col<float> vec) {
        this->__dotr = vec;
    }
    
    void setRotationMatrixLocal(Mat<float> mat) {
        this->__rotationMatrixLocal = mat;
    }
    
    void setRotationMatrixGlobal(Mat<float> mat) {
        this->__rotationMatrixGlobal = mat;
    }
    
    void setDOF(int DOF) {
        this->__DOF = DOF;
    }
    
    void setLocalMassMatrix(Mat<float> mat) {
        this->__localMassMatrix = mat;
    }
    
    void setSystemMassMatrix(Mat<float> mat) {
        this->__systemMassMatrix = mat;
    }
    
    void setVectorOfCorCent(Col<float> vec) {
        this->__vecOfCorCent = vec;
    }
    
    /* Below are protected functions for the Link class */
    
    virtual void updateDOFDimensions(int n);    // Update all matrix dimensions that depend on the DOF of the robot
    virtual void __updateDOFDimensions(int n);  // Recursive caller for updateDOFDimensions function
    virtual void attatchLink(Link* link);       // Attatch a link 
    virtual void __update();                    // Recursively udpates all states in the link based on the state of the previous link
    virtual void update() {};                   // Subclass Specific update function for link
    virtual void __updateBranch();              // Calls the update() function on the current link, updateBranch() of all next links in the chain
    virtual void setGammaIndex(int baseIndex);  // Assigns the gamma indicies in the gammaIndex vector
    virtual void updateGamma();                 // Retrieves the links value from the robot
    virtual void updateLocalMassMatrix();       // Updates the local mass matrix for the link
    virtual void updateSystemMassMatrix();      // Updates the system mass matrix for the link
    virtual void updateVectorOfCorCent();       // Updates the vector of coriolis and centripital terms for the robot
        
    Link();
    
public:
    
    Link(axis rotationAxis);
    Link(std::vector<axis> rotationAxes);
    
    /* Below are the getters for public members of the Link class*/
    
    Robot* getRobot() {
        return this->__robot;
    }
    
    Link* getPreviousLink() {
        return this->__previousLink;
    }
    
    std::vector<Link*> getNextLinks() {
        return this->__nextLinks;
    }
    
    float getMass() {
        return this->__mass;
    }
    
    Col<float> getFirstMassMoment() {
        return this->__firstMassMoment;
    }
    
    Mat<float> getInertiaMatrix() {
        return this->__inertiaMatrix;
    }
    
    Col<float> getlocalOffset() {
        return this->__localOffset;
    }
    
    Mat<float> getITilde() {
        return this->__ITilde;
    }
    
    Mat<float> getIHat() {
        return this->__IHat;
    }
    
    Mat<float> getITildeBot() {
        return this->__ITildeBot;
    }
    
    Mat<float> getIHatBot() {
        return this->__IHatBot;
    }
    
    Col<float> getGamma() {
        return this->__gamma;
    }
    
    Col<float> getDotGamma() {
        return this->__dotgamma;
    }
    
    Mat<float> getJacobian() {
        return this->__jacobian;
    }
    
    Mat<float> getDotJacobian() {
        return this->__dotjacobian;
    }
    
    Col<float> getOmegaLocal() {
        return this->__omegaLocal;
    }
    
    Col<float> getOmegaGlobal() {
        return this->__omega;
    }
    
    Col<float> getDotRLocal() {
        return this->__dotrLocal;
    }
    
    Col<float> getDotR() {
        return this->__dotr;
    }
    
    Mat<float> getRotationMatrixLocal() {
        return this->__rotationMatrixLocal;
    }
    
    Mat<float> getRotationMatrixGlobal() {
        return this->__rotationMatrixGlobal;
    }
    
    int getDOF() {
        return this->__DOF;
    }
    
    Mat<float> getLocalMassMatrix() {
        return this->__localMassMatrix;
    }
    
    Mat<float> getSystemMassMatrix() {
        return this->__systemMassMatrix;
    }
    
    Col<float> getVectorOfCorCent() {
        return this->__vecOfCorCent;
    }
    
    /* Below are the setters for the public members of the link class */
    
    void setMass(float mass) {
        this->__mass = mass;
    }
    
    void setFirstMassMoment(Col<float> moment) {
        this->__firstMassMoment = moment;
    }
    
    void setInertiaMatrix(Mat<float> matrix) {
        this->__inertiaMatrix = matrix;
    }
    
    void setITilde(Mat<float> matrix) {
        this->__ITilde = matrix;
    }
    
    void setIHat(Mat<float> matrix) {
        this->__IHat = matrix;
    }
    
    void setLocalOffset(Col<float> offset) {
        this->__localOffset = offset;
    }
    
};

/**
 Root Link of a robot. Updates are overwritten by the user rather than recersively
 */

class RootLink : public Link {
protected:
public:
    
    RootLink();
    virtual void __update() override {
        this->update();
        this->updateLocalMassMatrix();
        this->updateSystemMassMatrix();
        this->updateVectorOfCorCent();
    }
    virtual void update() override;         // Called everytime the robot is updated. Must be overloaded by user
    virtual void initialize();              // Called when the root link is initialized. Must be overloaded by user
    virtual void setJacobianAtIndex(int row, int col, float val) {
        this->setJacobian(setValueAtIndex(this->getJacobian(), row, col, val));
    }
};

/* Below are non-member functions for the RootLink class */

/**
 * Revolute Link described by pure rotational motion
 */

class RLink : public Link {
protected:
    std::vector<axis> __rotationAxes;
    
    /* Below are all member functions for revolute joints */
    virtual void updateRotationMatrixLocal();                  // Updates the local rotation matrix based on the current gamma value
    virtual void updateRotationMatrixGlobal();                 // Updates the global rotation matrix based on the previous link and current gamma value
public:
    RLink(axis rotationAxis);
    RLink(std::vector<axis> rotationAxes);
    
    
    virtual void update() override;
    
    /* Below are the public getters for the RLink class */
    
    std::vector<axis> getRotationAxes() {
        return this->__rotationAxes;
    }
};

}

#endif /* Link_hpp */
