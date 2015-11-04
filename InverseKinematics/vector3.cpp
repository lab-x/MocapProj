//
//  vector3.cpp
//  InverseKinematics
//
//  Created by MengTsao on 3/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h> 

#include "vector3.h"

Vector3::Vector3() {
    values[0] = 1;
    values[1] = 0;
    values[2] = 0;
}

Vector3::Vector3(float xx, float yy, float zz) {
    values[0] = xx;
    values[1] = yy;
    values[2] = zz;
}


Vector3& Vector3::operator=(const Vector3& rhs) {
    this->values[0] = rhs.values[0];
    this->values[1] = rhs.values[1];
    this->values[2] = rhs.values[2];
    return *this;
}


float Vector3::getX() {
    return values[0];
}
float Vector3::getY() {
    return values[1];
}
float Vector3::getZ() {
    return values[2];
}

Vector3 Vector3::Normalize(Vector3 vect){
    return Vector3(vect / Vector3::Getlen(vect));
}

float Vector3::Getlen(Vector3 vect){
    return sqrt(vect.getX() * vect.getX() + vect.getY() * vect.getY() + vect.getZ() * vect.getZ());
}
void Vector3::SetLength(Vector3 vect, float length)
{
    vect = vect * (length / Getlen(vect));
}

float Vector3::Dot(Vector3 vec1, Vector3 vec2)
{
    return vec1.getX() * vec2.getX() + vec1.getY() * vec2.getY() + vec1.getZ() * vec2.getZ();
}
Vector3 Vector3::cross(Vector3 vec1, Vector3 vec2)
{
    return Vector3(vec1.getY() * vec2.getZ() - vec1.getZ() * vec2.getY(),
                   vec1.getZ() * vec2.getX() - vec1.getX() * vec2.getZ(),
                   vec1.getX() * vec2.getY() - vec1.getY() * vec2.getX());
}

