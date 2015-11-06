//
//  point.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include "point.h"
Point::Point(){
    position = Position();
    FWDbasic = Axes();
    BWDbasic = Axes();
}

Point::Point(Position P, Axes F, Axes B) {
    position = P;
    FWDbasic = F;
    BWDbasic = B;
}

Point::Point(Point point[]) {
    position = point->position;
    FWDbasic = point->FWDbasic;
    BWDbasic = point->BWDbasic;
}

Point::Point( const Point& other ) {
    position = other.position;
    FWDbasic = other.FWDbasic;
    BWDbasic = other.BWDbasic;
}


Point& Point::operator=(const Point& rhs) {
        this->position = rhs.position;
  		this->FWDbasic = rhs.FWDbasic;
        this->BWDbasic = rhs.BWDbasic;
    return *this;
}

Position Point::getPosition(){
    return this->position;
}
Axes Point::getFWDAxes(){
    return this->FWDbasic;
}
Axes Point::getBWDAxes(){
    return this->BWDbasic;
}
void Point::setFWDAxes(Axes rhs){
    this->FWDbasic = rhs;
}
void Point::setBWDAxes(Axes rhs){
    this->BWDbasic = rhs;
}

void Point::setPosition(Position rhs){
    this->position = rhs;
}
