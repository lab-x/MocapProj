//
//  axis.cpp
//  InverseKinematics
//
//  Created by MengTsao on 3/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include "axis.h"


Axis::Axis(){
    axis[0] = Vector3(1,0,0);
    axis[1] = Vector3(0,1,0);
    axis[2] = Vector3(0,0,1);
    
}
Axis::Axis(Vector3 X, Vector3 Y, Vector3 Z){
    axis[0] = X;
    axis[1] = Y;
    axis[2] = Z;
}

Axis::Axis(Axis& other){
    axis[0] = other.axis[0];
    axis[1] = other.axis[1];
    axis[2] = other.axis[2];
}

Axis& Axis::operator=(const Axis& rhs) {
    this->axis[0] = rhs.axis[0];
    this->axis[1] = rhs.axis[1];
    this->axis[2] = rhs.axis[2];
    return *this;
}

Vector3 Axis::GetXAxis() {
    return axis[0];
}

Vector3 Axis::GetYAxis() {
    return axis[1];
}

Vector3 Axis::GetZAxis() {
    return axis[2];
}
