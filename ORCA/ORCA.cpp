//
//  ORCA.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/6/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include <iostream>
#include "ORCA.hpp"
#include "ORCAPriv.hpp"

void ORCA::HelloWorld(const char * s)
{
    ORCAPriv *theObj = new ORCAPriv;
    theObj->HelloWorldPriv(s);
    delete theObj;
};

void ORCAPriv::HelloWorldPriv(const char * s) 
{
    std::cout << s << std::endl;
};

