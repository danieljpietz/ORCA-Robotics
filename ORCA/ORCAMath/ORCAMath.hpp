//
//  ORCAMath.hpp
//  ORCA
//
//  Created by Daniel Pietz on 5/5/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#ifndef ORCAMath_hpp
#define ORCAMath_hpp

#include <armadillo>

#include "ORCASolvers.hpp"

using namespace arma;

namespace ORCA {

/**
 * Returns the DCM representing a rotation
 * about the 'x' axis of angle theta
 * @param theta theta
 */

Mat<float> rotx(float theta);
Mat<float> roty(float theta);
Mat<float> rotz(float theta);
Mat<float> rotq(Col<float> q);
Mat<float> skew(Col<float> vec);
Col<float> quaternionProduct(Col<float> q1, Col<float> q2);

}

#endif /* ORCAMath_hpp */
