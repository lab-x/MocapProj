//
//  Joint.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include "Joint.h"
Joint::Joint(){
    position = Position();
    FWDbasic = Axes();
    BWDbasic = Axes();
}

Joint::Joint(Position P, Axes F, Axes B) {
    position = P;
    FWDbasic = F;
    BWDbasic = B;
}

Joint::Joint(Joint Joint[]) {
    position = Joint->position;
    FWDbasic = Joint->FWDbasic;
    BWDbasic = Joint->BWDbasic;
}

Joint::Joint( const Joint& other ) {
    position = other.position;
    FWDbasic = other.FWDbasic;
    BWDbasic = other.BWDbasic;
}


Joint& Joint::operator=(const Joint& rhs) {
        this->position = rhs.position;
  		this->FWDbasic = rhs.FWDbasic;
        this->BWDbasic = rhs.BWDbasic;
    return *this;
}

Position Joint::getPosition(){
    return this->position;
}
Axes Joint::getFWDAxes(){
    return this->FWDbasic;
}
Axes Joint::getBWDAxes(){
    return this->BWDbasic;
}
void Joint::setFWDAxes(Axes rhs){
    this->FWDbasic = rhs;
}
void Joint::setBWDAxes(Axes rhs){
    this->BWDbasic = rhs;
}

void Joint::setPosition(Position rhs){
    this->position = rhs;
}
