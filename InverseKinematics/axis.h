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
    Axis();
    Axis(Vector3 X,Vector3 Y,Vector3 Z);
    Axis(Axis& other);
    Axis& operator=(const Axis& rhs);
    
    Vector3 GetXAxis();
    Vector3 GetYAxis();
    Vector3 GetZAxis();
    public :
    static void Normalize(Vector3& vect);
    
};


#endif /* defined(__InverseKinematics__axis__) */
