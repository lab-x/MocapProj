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
#include "point.h"
#include "axes.h"


class Fabrik {
    Point goal;
    Point joints[4];
    float d[3];
    float tol;
    float epsilon;
public:
    Fabrik() {}
    Fabrik(float tolerance, float eps);
    
    void setJoints(Point one, Point two, Point three, Point four);
    void setGoal(Point x); 
    Point* getJoints();
    void compute();
    void shrinkEnd();
    void Rotation_Constraint(Point& This, Point Prev, Axes PprevAxes);
    void Orientation_Constraint(Point& This, Point Prev);
    void SetOrientation(Point& This, Point Previous);

};


#endif /* defined(__InverseKinematics__fabrik__) */

