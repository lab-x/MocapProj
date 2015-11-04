//
//  quaternion.h
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__quaternion__
#define __InverseKinematics__quaternion__
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h> 
#include "vector3.h"

class Quaternion {
    float values[4];

public:
    Quaternion();
    Quaternion(float w, float x, float y, float z);
    Quaternion(float quaternion[]);
    Quaternion(const Quaternion& other );
    Quaternion& operator=(const Quaternion& rhs);
 
    float* getValues();
    float getW();
    float getX();
    float getY();
    float getZ();
    void normalize();
    
    
    friend Quaternion operator*(Quaternion lhs, Quaternion rhs){
        Quaternion rst;
        rst.values[0] = lhs.values[0] * rhs.values[0] - lhs.values[1] * rhs.values[1] - lhs.values[2] * rhs.values[2] - lhs.values[3] * rhs.values[3];
        
        rst.values[1] = lhs.values[0] * rhs.values[1] + lhs.values[1] * rhs.values[0] + lhs.values[2] * rhs.values[3] - lhs.values[3] * rhs.values[2];
        
        rst.values[2] = lhs.values[0] * rhs.values[2] - lhs.values[1] * rhs.values[3] + lhs.values[2] * rhs.values[0] + lhs.values[3] * rhs.values[1];
        
        rst.values[3] = lhs.values[0] * rhs.values[3] + lhs.values[1] * rhs.values[2] - lhs.values[2] * rhs.values[1] + lhs.values[3] * rhs.values[0];
        return rst;
    };

public :
    static Vector3 rotVbyQ(Vector3 v, Quaternion q);
    static Quaternion v2q(Vector3 vec1, Vector3 vec2);
    static Quaternion conjugate(Quaternion q1);
};

#endif /* defined(__InverseKinematics__quaternion__) */
