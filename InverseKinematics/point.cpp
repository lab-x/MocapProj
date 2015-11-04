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
    axis = Axis();
}

Point::Point(Position P, Axis A) {
    position = P;
    axis = A;
}

Point::Point(Point point[]) {
    position = point->position;
    axis = point->axis;
}

Point::Point( const Point& other ) {
    position = other.position;
    axis = other.axis;
}
 //Axis Point::setAxis(Axis a) {
//
//}

Point& Point::operator=(const Point& rhs) {
        this->position = rhs.position;
  		this->axis = rhs.axis;
  		return *this;
}

Position Point::getPosition(){
    return this->position;
}

Axis Point::getAxis(){
    return this->axis;
}

void Point::setPosition(Position rhs){
    this->position = rhs;
}

void Point::setAxis(Axis rhs){
    this->axis = rhs;
}

