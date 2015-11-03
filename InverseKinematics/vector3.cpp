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

Vector3::Vector3(float xx, float yy, float zz) {
    values[0] = xx;
    values[1] = yy;
    values[2] = zz;
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

