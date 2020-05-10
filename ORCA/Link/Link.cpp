//
//  Link.cpp
//  ORCA
//
//  Created by Daniel Pietz on 5/6/20.
//  Copyright © 2020 Daniel Pietz. All rights reserved.
//

#include "Link.hpp"

using namespace ORCA;
using namespace arma;

/**
 Default Constructor for Link. Sets all members to NULL or Zero
 */

Link::Link() {
    this->setRobot((Robot*) NULL);
    this->setPreviousLink((Link*) NULL);
    this->setMass(0);
    this->setFirstMassMoment(Col<float>(3, fill::zeros));
    this->setInertiaMatrix(Mat<float>(3, 3, fill::zeros));
    this->setLocalOffset(Col<float>(3, fill::zeros));
    this->setITilde(Mat<float>(3,1, fill::zeros));
    this->setIHat(Mat<float>(3,1, fill::zeros));
    this->setITildeBot(Mat<float>(3,1, fill::zeros));
    this->setIHatBot(Mat<float>(3,1, fill::zeros));
    this->setGamma(Col<float>(1,fill::zeros));
    this->setDotGamma(Col<float>(1,fill::zeros));
    this->setJacobian(Mat<float>(6,1,fill::zeros));
    this->setDotJacobian(Mat<float>(6,1,fill::zeros));
    this->setOmegaLocal(Col<float>(3,fill::zeros));
    this->setOmegaGlobal(Col<float>(3,fill::zeros));
    this->setDotRLocal(Col<float>(3,fill::zeros));
    this->setDotRGlobal(Col<float>(3,fill::zeros));
}

/**
 Updates dimensions of link matricies to have 'n' columns
 @param n Number of columns in matrix
 */

void Link::updateDOFDimensions(int n) {
    this->__jacobian.resize(6, n);
    this->__dotjacobian.resize(6,n);
    this->__IHatBot = Mat<float>(3,n,fill::zeros);
    this->__ITildeBot = Mat<float>(3,n,fill::zeros);
    int i;
    for (i = 0; i < this->getDOF(); i++) {
        this->__IHatBot.col(this->__gammaIndex[i]) = this->getIHat().col(i);
    }
}
/**
 Updates dimsneions of branch matricies to have 'n' columns
 @param n Number of columns in matrix
 */
void Link::__updateDOFDimensions(int n) {
    this->updateDOFDimensions(n);
    int i;
    for (i = 0; i < this->getNextLinks().size(); i++) {
        this->getNextLinks()[i]->__updateDOFDimensions(n);
    }
}

/**
 Attatches a child link to this link
 @param link Child link to be attatched
 */

void Link::attatchLink(Link* link) {
    __nextLinks.push_back(link);
    link->setPreviousLink(this);
}

/**
Recursive update of this link and all links further down in the chain.
*/


void Link::__updateBranch() {
    this->__update();
    int i;
    for (i = 0; i < this->getNextLinks().size(); i++) {
        this->getNextLinks()[i]->__updateBranch();
    }
}

/**
 Recursve update of all link elements based on state of previous link
 */

void Link::__update() {
    this->updateGamma();
    this->update();
    Col<float> botGamma = this->getRobot()->getGamma();
    Col<float> botDotGamma = this->getRobot()->getDotGamma();
    
    Col<float> velocity_prev = this->getPreviousLink()->getJacobian() *  botDotGamma;
    Col<float> w_prev;
    w_prev << velocity_prev[0] << endr << velocity_prev[1] << endr << velocity_prev[2];
    Col<float> dotr_prev;
    dotr_prev << velocity_prev[3] << endr << velocity_prev[4] << endr << velocity_prev[5];
    Mat<float> T_prev = this->getPreviousLink()->getRotationMatrixGlobal();
    Mat<float> J_prev = this->getPreviousLink()->getJacobian();
    Mat<float> dotJ_prev = this->getPreviousLink()->getDotJacobian();
    Col<float> r_local = this->getlocalOffset();
    Col<float> w_local = this->getIHat() * this->getDotGamma();
    Col<float> dotr_local = this->getITilde() * this->getDotGamma();
    
    
    Mat<float> tempMatrix1 = join_horiz(this->getRotationMatrixLocal().t(), Mat<float>(3, 3, fill::zeros));
    Mat<float> tempMatrix2 = join_horiz(-T_prev * skew(r_local), Mat<float>(3, 3, fill::eye));
    Mat<float> J = join_vert(tempMatrix1, tempMatrix2)*J_prev + join_vert(this->getIHatBot(), T_prev*this->getITildeBot());
    Col<float> velocity_current = J*botDotGamma;
    Col<float> w_current;
    w_current << velocity_current[0] << endr << velocity_current[1] << endr << velocity_current[2];
    Col<float> dotr_current;
    dotr_current << velocity_current[3] << endr << velocity_current[4] << endr << velocity_current[5];
    Mat<float> tempMatrix3 = join_horiz(-skew(w_local)*this->getRotationMatrixLocal().t(), Mat<float>(3, 3, fill::zeros));
    Mat<float> tempMatrix4 = join_horiz(-T_prev*(skew(w_prev)*skew(r_local)+skew(dotr_local)), Mat<float>(3, 3, fill::zeros));
    Mat<float> tempMatrix5 = join_horiz(this->getRotationMatrixLocal().t(), Mat<float>(3, 3, fill::zeros));
    Mat<float> tempMatrix6 = join_horiz(-T_prev*skew(r_local), Mat<float>(3, 3, fill::eye));
    
    Mat<float> dotJCurrent = join_vert(tempMatrix3, tempMatrix4)*J_prev + join_vert(tempMatrix5,tempMatrix6)*dotJ_prev + join_vert(Mat<float>(3,this->getRobot()->getDOF(), fill::zeros), T_prev*skew(w_prev)*this->getITildeBot());
    
    this->setOmegaLocal(w_local);
    this->setDotRLocal(dotr_local);
    this->setOmegaGlobal(w_current);
    this->setDotRGlobal(dotr_current);
    this->setJacobian(J);
    this->setDotJacobian(dotJCurrent);
    this->updateLocalMassMatrix();
    this->updateSystemMassMatrix();
    this->updateVectorOfCorCent();
}



void Link::setGammaIndex(int baseIndex) {
    this->__gammaIndex.resize(0);
    int i;
    for (i = 0; i < this->getDOF(); i++) {
        this->__gammaIndex.push_back(baseIndex + i);
    }
}

void Link::updateGamma() {
    int i;
    for (i = 0; i < this->getDOF(); i++) {
        this->__gamma[i] = this->getRobot()->getGamma()[this->__gammaIndex[i]];
        this->__dotgamma[i] = this->getRobot()->getDotGamma()[this->__gammaIndex[i]];
    }
}

void Link::updateLocalMassMatrix() {
    //[J11,skew(GAMMA11)*T1.';T1*skew(GAMMA11).',m1*eye(3)]
    Mat<float> temp1 = join_horiz(this->getInertiaMatrix(), skew(this->getFirstMassMoment()) * this->getRotationMatrixGlobal().t());
    
    Mat<float> temp2 = join_horiz(this->getRotationMatrixGlobal() * skew(this->getFirstMassMoment()).t(), this->getMass() * Mat<float>(3,3, fill::eye));
    this->setLocalMassMatrix(join_vert(temp1, temp2));
}

void Link::updateSystemMassMatrix() {
    //J1.'*M1*J1
    this->setSystemMassMatrix((this->getJacobian().t() * this->getLocalMassMatrix()) * this->getJacobian());
}

void Link::updateVectorOfCorCent() {
    //J1.'*M1*dotJ1*dotgamma + J1.'*[cross(w1,J11*w1);T1*cross(w1,cross(w1,GAMMA11))];
    Col<float> dotgamma = this->getRobot()->getDotGamma();
    Mat<float> T = this->getRotationMatrixGlobal();
    Col<float> omega = this->getOmegaGlobal();
    Mat<float> J = this->getJacobian();
    Mat<float> JJ = this->getInertiaMatrix();
    Col<float> GAMMA = this->getFirstMassMoment();
    this->setVectorOfCorCent(J.t()*(this->getLocalMassMatrix()*this->getDotJacobian()*dotgamma + join_vert(cross(omega,JJ*omega), T*cross(omega,cross(omega,GAMMA)))));
}
