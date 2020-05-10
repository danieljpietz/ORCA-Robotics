//
//  Force.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/8/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include "../ORCA"

class Link;
class Robot;

using namespace arma;

namespace ORCA {
class Force {
protected:
    Col<float> __value;
    std::vector<Link*> __links;
public:
    
    /* Below are the public getters for the Force class */
    
    std::vector<Link*> getLinks() {
        return this->__links;
    }
    
    Col<float> getValue() {
        return __value;
    }
    
    /* Below are the public setters for the Force class */
    
    void setLinks(std::vector<Link*> links) {
        this->__links = links;
    }
    
    void addLink(Link* link) {
        this->__links.push_back(link);
    }
    
    void setValue(Col<float> value) {
        this->__value = value;
    }
};

}


