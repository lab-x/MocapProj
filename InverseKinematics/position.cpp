//
//  position.cpp
//  InverseKinematics
//
//  Created by MengTsao on 3/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//
#include <iostream>
#include <cmath>
#include <stdlib.h>

#include "position.h"


Position::Position(float xx, float yy, float zz) {
    values[0] = xx;
    values[1] = yy;
    values[2] = zz;
}

Position::Position(float point[]) {
    values[0]= point[0];
    values[1] = point[1];
    values[2] = point[2];
}

Position::Position( const Position& other ) {
    values[0] = other.values[0];
  		values[1] = other.values[1];
  		values[2] = other.values[2];
}


float* Position::getValues() {
    return values;
}

float Position::getX() {
    return values[0];
}

float Position::getY() {
    return values[1];
}

float Position::getZ() {
    return values[2];
}

float Position::getDistance() {
    return sqrt(pow(values[0], 2)
                + pow(values[1], 2)
                + pow(values[2], 2));
}

float Position::Distance(Position lhs, Position rhs) {
    Position dist = rhs -lhs;
    return sqrt(pow(dist.values[0], 2)
                + pow(dist.values[1], 2)
                + pow(dist.values[2], 2));
}

void Position::normalize() {
    float length = sqrt(pow(values[0], 2)
                        + pow(values[1], 2)
                        + pow(values[2], 2));
    values[0] = values[0] / length;
    values[1] = values[1] / length;
    values[2] = values[2] / length;
}

Position& Position::operator+=(const Position& rhs) {
    // actual addition of rhs to *this
  		this->values[0] += rhs.values[0];
  		this->values[1] += rhs.values[1];
  		this->values[2] += rhs.values[2];
    return *this;
}

Position& Position::operator-=(const Position& rhs) {
    // actual addition of rhs to *this
  		this->values[0] -= rhs.values[0];
  		this->values[1] -= rhs.values[1];
  		this->values[2] -= rhs.values[2];
    return *this;
}


Position& Position::operator=(const Position& rhs) {
    this->values[0] = rhs.values[0];
  		this->values[1] = rhs.values[1];
  		this->values[2] = rhs.values[2];
  		return *this;
}

bool Position::operator == (const Position &rhs) const {
 	 	return (this->values[0] == rhs.values[0] &&
                this->values[1] == rhs.values[1] &&
                this->values[2] == rhs.values[2]);
}
