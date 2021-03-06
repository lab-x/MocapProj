//
//  Joint.h
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__Joint__
#define __InverseKinematics__Joint__

#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include "position.h"
#include "axes.h"

class Joint {
    Position position;
    Axes     BWDbasic;
    Axes     FWDbasic;
//call public :
public:
    Position getPosition();
    Axes getBWDAxes();
    Axes getFWDAxes();
    void setBWDAxes(Axes rhs);
    void setFWDAxes(Axes rhs);
    void setPosition(Position rhs);
    
public:
    Joint();
    Joint(Position P, Axes F, Axes B);
    Joint(Joint Joint[]);
    Joint(const Joint& other );
    Joint& operator=(const Joint& rhs);
    
};

#endif /* defined(__InverseKinematics__Joint__) */






