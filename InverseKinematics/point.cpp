//
//  point.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//


#include "point.h"


Point::Point(float xx, float yy, float zz) {
    values[0] = xx;
    values[1] = yy;
    values[2] = zz;
}

Point::Point(float point[]) {
    values[0]= point[0];
    values[1] = point[1];
    values[2] = point[2];
}

Point::Point( const Point& other ) {
    values[0] = other.values[0];
  		values[1] = other.values[1];
  		values[2] = other.values[2];
}


float* Point::getValues() {
    return values;
}

float Point::getX() {
    return values[0];
}

float Point::getY() {
    return values[1];
}

float Point::getZ() {
    return values[2];
}

float Point::getDistance() {
    return sqrt(pow(values[0], 2)
                + pow(values[1], 2)
                + pow(values[2], 2));
}

float Point::Distance(Point lhs, const Point& rhs) {
    Point dist = rhs -lhs;
    return sqrt(pow(dist.values[0], 2)
                + pow(dist.values[1], 2)
                + pow(dist.values[2], 2));
}

void Point::normalize() {
    float length = sqrt(pow(values[0], 2)
                        + pow(values[1], 2)
                        + pow(values[2], 2));
    values[0] = values[0] / length;
    values[1] = values[1] / length;
    values[2] = values[2] / length;
}

Point& Point::operator+=(const Point& rhs) {
    // actual addition of rhs to *this
  		this->values[0] += rhs.values[0];
  		this->values[1] += rhs.values[1];
  		this->values[2] += rhs.values[2];
    return *this;
}

Point& Point::operator-=(const Point& rhs) {
    // actual addition of rhs to *this
  		this->values[0] -= rhs.values[0];
  		this->values[1] -= rhs.values[1];
  		this->values[2] -= rhs.values[2];
    return *this;
}


Point& Point::operator=(const Point& rhs) {
    this->values[0] = rhs.values[0];
  		this->values[1] = rhs.values[1];
  		this->values[2] = rhs.values[2];
  		return *this;
}

bool Point::operator == (const Point &rhs) const {
 	 	return (this->values[0] == rhs.values[0] &&
                this->values[1] == rhs.values[1] &&
                this->values[2] == rhs.values[2]);
}
