//
//  axis.h
//  InverseKinematics
//
//  Created by MengTsao on 3/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__axis__
#define __InverseKinematics__axis__

#include <stdio.h>
#include <stdio.h>
#include "vector3.h"

class Axis{
    Vector3 axis[3];
public:
    
    Axis(Vector3 X, Vector3 Y, Vector3 Z);
    
    Axis() {
        Vector3 X(1,0,0);
        Vector3 Y(0,1,0);
        Vector3 Z(0,0,1);
        Axis(X, Y, Z);
    }
    
    Axis(const Axis& other);
    Vector3 GetXAxis();
    Vector3 GetYAxis();
    Vector3 GetZAxis();
    
public :
static void Normalize(Vector3& vect){
    }

};


#endif /* defined(__InverseKinematics__axis__) */
