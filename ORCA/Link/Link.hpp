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
#include <armadillo>

using namespace arma;

namespace ORCA {
class Link {
protected:
    
    /* Below are constant parameters for a robotic link*/
    /* These should be zeroed on initalization and can */
    /* Be set by the public setters */
    
    float mass;                     // Mass of the link in kilograms
    Col<float> firstMassMoment;     // Vector of First Mass Moment measured from the links locael frame (kg*m)
    Col<float> InertiaMatrix;       // Matrix of second mass moment measured in the links local frame (kg*m^2)
    Col
    
    
    
public:
};
}

#endif /* Link_hpp */
