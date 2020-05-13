//
//  ORCAMath.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/5/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include "ORCAMath.hpp"

using namespace arma;


Mat<float> ORCA::rotx(float theta) {
    
    Mat<float> returnMat(3,3);
    float ct = cos(theta);
    float st = sin(theta);
    returnMat << 1 << 0 << 0 << endr
    << 0 << ct << -st <<endr
    << 0 << st << ct;
    return returnMat;
}

Mat<float> ORCA::roty(float theta) {
    
    Mat<float> returnMat(3,3);
    float ct = cos(theta);
    float st = sin(theta);
    returnMat << ct << 0 << st << endr
    << 0 << 1 << 0 << endr
    << -st << 0 << ct;
    return returnMat;
}

Mat<float> ORCA::rotz(float theta) {
    
    Mat<float> returnMat(3,3);
    float ct = cos(theta);
    float st = sin(theta);
    returnMat << ct << -st << 0 << endr
    << st << ct << 0 << endr
    << 0 << 0 << 1;
    return returnMat;
}

Mat<float> ORCA::skew(Col<float> vec) {
    
    Mat<float> returnMat(3,3);
    returnMat << 0 << -vec[2] << vec[1] << endr <<  vec[2] << 0 << -vec[0] << endr << -vec[1] << vec[0] << 0;
    return returnMat;
}

Mat<float> ORCA::rotq(Col<float> q) {
    
    float t2 = q[0]*q[0];
    float t3 = q[1]*q[1];
    float t4 = q[2]*q[2];
    float t5 = q[3]*q[3];
    float t6 = q[0]*q[1]*2.0;
    float t7 = q[0]*q[2]*2.0;
    float t8 = q[0]*q[3]*2.0;
    float t9 = q[1]*q[2]*2.0;
    float t10 = q[1]*q[3]*2.0;
    float t11 = q[2]*q[3]*2.0;
    
    Mat<float> returnMat(3,3);
    
    returnMat << t2+t3-t4-t5 << -t8+t9 << t7+t10 << endr << t8+t9 << t2+t4-t3-t5 << -t6+t11 << endr << -t7+t10 << t6+t11 << t2+t5-t3-t4;
    
    return returnMat;
}



Col<float> ORCA::quaternionProduct(Col<float> q1, Col<float> q2) {
    Mat<float> tempMat(4,4);
    tempMat << q1[0] << -q1[1] << -q1[2] << -q1[3] << endr
    << q1[1] << q1[0] << -q1[3] << q1[2] << endr
    << q1[2] <<  q1[3] <<  q1[0] << -q1[1] << endr
    << q1[3] << -q1[2] << q1[1] << q1[0];
    return tempMat * q2;
}
