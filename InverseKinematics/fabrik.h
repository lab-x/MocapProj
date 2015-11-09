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
public:
    Fabrik() {}
    Fabrik(float tolerance, float eps);
    
    void setJoints(Joint one, Joint two, Joint three, Joint four);
    void setGoal(Joint x);
    
    Joint* getJoints();
    Vector3* getEulers();
    
    void compute();
    void shrinkEnd();
    void Rotation_Constraint(Joint& This, Joint Prev, Axes PprevAxes);
    void Orientation_Constraint(Joint& This, Joint Prev, int Type);
    void SetOrientation(Joint& This, Joint Previous, int Type);
    void GenQW();
    void GenBones();
    void GenQL();
    void GenEuler();
};


#endif /* defined(__InverseKinematics__fabrik__) */

