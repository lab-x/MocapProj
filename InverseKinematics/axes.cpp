//
//  axis.cpp
//  InverseKinematics
//
//  Created by MengTsao on 3/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include "axes.h"


Axes::Axes(){
    axis[0] = Vector3(1,0,0);
    axis[1] = Vector3(0,1,0);
    axis[2] = Vector3(0,0,1);
    
}
Axes::Axes(Vector3 X, Vector3 Y, Vector3 Z){
    axis[0] = X;
    axis[1] = Y;
    axis[2] = Z;
}

Axes::Axes(Axes& other){
    axis[0] = other.axis[0];
    axis[1] = other.axis[1];
    axis[2] = other.axis[2];
}

Axes& Axes::operator=(const Axes& rhs) {
    this->axis[0] = rhs.axis[0];
    this->axis[1] = rhs.axis[1];
    this->axis[2] = rhs.axis[2];
    return *this;
}

Vector3 Axes::GetXAxis() {
    return axis[0];
}

Vector3 Axes::GetYAxis() {
    return axis[1];
}

Vector3 Axes::GetZAxis() {
    return axis[2];
}
