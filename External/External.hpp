//
//  External.hpp
//  ORCA
//
//  Created by Daniel Pietz on 5/8/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#ifndef External_hpp
#define External_hpp

#include "../Link/Link.hpp"
#include "../ORCAMath/ORCAMath.hpp"
#include <armadillo>

typedef enum {Local, Global} CoordinateType;


namespace ORCA {

class Link;
class Robot;
class RootLink;

/**
 Base class for a force in the joint space. These are 1 Dimensional forces are applied directly to the joints
 and model things such as friction
 */

class JointForce {
protected:
public:
    virtual void getValue(Link* link) {
        
    };
    
};

class Force : public JointForce {
protected:
    CoordinateType __positionFrame;
    Col<float> __position;
    Col<float> __worldValue;
    
public:
    Force();
    
    virtual Col<float> getPosition() {
        return this->__position;
    }
    
    virtual Col<float> getPosition(Link* link) {
        return this->__position;
    }
    
    virtual Col<float> getWorldValue() {
        return this->__worldValue;
    }
    
    CoordinateType getPositionFrame() {
        return this->__positionFrame;
    }
    
    /* Below are the public setters for the Force class */
    
    virtual void setPosition(Col<float> vec) {
        this->__position = vec;
    }
    
    void setCoordinateFrame(CoordinateType frame) {
        this->__positionFrame = frame;
    }
    
    virtual void setWorldValue(Col<float> vec) {
        this->__worldValue = vec;
    }
    
    virtual Col<float> getWorldValue(Link* link) {
        return __worldValue;
    }
    
    virtual void getValue(Link* link);
};

class Gravity : public Force {
    Gravity();
public:
    Gravity(Col<float> direction) {
        this->__worldValue = direction;
    }
    virtual Col<float> getPosition(Link* link) override;
    virtual Col<float> getWorldValue(Link* link) override;
    
};

class Friction : public JointForce {
protected:
    float __coefficient;
public:
    Friction();
    void setCoefficient(float c) {
        this->__coefficient = c;
    }
    
    float getCoefficient() {
        return this->__coefficient;
    }
        
};

class CoulombicFriction : public Friction {    
public:
    CoulombicFriction() {}
    void getValue(Link* link) override;
};

class ViscousFriction : public Friction {
public:
    ViscousFriction() {}
    void getValue(Link* link) override;
};

}

#endif /* External_hpp */
