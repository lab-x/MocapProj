//
//  fabrik.h
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__fabrik__
#define __InverseKinematics__fabrik__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>

#include "Joint.h"
#include "axes.h"
#include "quaternion.h"
#include "vector3.h"

class Fabrik {
    Joint goal;
    Joint joints[4];
    float d[3];
    Vector3 bone[3];
    Quaternion QW[3];
    Quaternion QL[3];
    Vector3 Euler[3];
    float tol;
    float epsilon;
    Vector3 initialBone[3] ;

public:
    Fabrik() {}
    Fabrik(float tolerance, float eps);
    
    void setJoints(Joint one, Joint two, Joint three, Joint four);
    void setGoal(Joint x);
    
    Joint* getJoints();
    Vector3* getEulers();
    
    void compute(int LinkNo);
    void shrinkEnd();
    void Rotation_Constraint(Joint& This, Joint Prev, Axes PprevAxes);
    void Orientation_Constraint(Joint& This, Joint Prev, int Type);
    void SetOrientation(Joint& This, Joint Previous, int Type);
    void GenQW(int LinkNo);
    void GenBones(int LinkNo);
    void GenQL(int LinkNo);
    void GenEuler(int LinkNo);
    void Position_Constraints(Joint &This, int thisID, int Dir, int LinkNo);
};


#endif /* defined(__InverseKinematics__fabrik__) */

