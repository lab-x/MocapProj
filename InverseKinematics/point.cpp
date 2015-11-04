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
    axes = Axes();
}

Point::Point(Position P, Axes A) {
    position = P;
    axes = A;
}

Point::Point(Point point[]) {
    position = point->position;
    axes = point->axes;
}

Point::Point( const Point& other ) {
    position = other.position;
    axes = other.axes;
}


Point& Point::operator=(const Point& rhs) {
        this->position = rhs.position;
  		this->axes = rhs.axes;
  		return *this;
}

Position Point::getPosition(){
    return this->position;
}

Axes Point::getAxes(){
    return this->axes;
}

void Point::setPosition(Position rhs){
    this->position = rhs;
}

void Point::setAxes(Axes rhs){
    this->axes = rhs;
}

