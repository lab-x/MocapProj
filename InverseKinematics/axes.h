//
//  axes.h
//  InverseKinematics
//
//  Created by MengTsao on 3/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__axes__
#define __InverseKinematics__axes__

#include <stdio.h>
#include <stdio.h>
#include "vector3.h"


class Axes{
    Vector3 axis[3];


public:
    Axes();
    Axes(Vector3 X,Vector3 Y,Vector3 Z);
    Axes(Axes& other);
    Axes& operator=(const Axes& rhs);
    
    Vector3 GetXAxis();
    Vector3 GetYAxis();
    Vector3 GetZAxis();
    public :
    static void Normalize(Vector3& vect);
    
};


#endif /* defined(__InverseKinematics__axes__) */
