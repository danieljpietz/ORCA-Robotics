//
//  Force.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/8/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include "../ORCA"


using namespace arma;
using namespace ORCA;

Force::Force() {
    
}

Col<float> Force::getValue(Link* link) {
    if(__positionFrame == Local) {
        return skew(__position) * (link->getRotationMatrixLocal().t())*this->getWorldValue(link);
    }
    else {
        return skew(__position - link->getOffsetGlobal()) * (link->getRotationMatrixLocal().t())*this->getWorldValue(link);
    }
}

