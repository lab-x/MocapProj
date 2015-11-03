//
//  point.h
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__point__
#define __InverseKinematics__point__

#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include "position.h"
#include "axis.h"

class Point {
    Position position;
    Axis     axis;
//call public :
public:
    Point() {}
    Point(Position P, Axis A);
    
    Point(Point point[]);
    Point(const Point& other );
    
    Position getPosition();
    Axis getAxis();
    
   // Point setPosition(Position P);
   // Point setAxis(Axis A);
    Point& operator=(const Point& rhs);
    
};

#endif /* defined(__InverseKinematics__point__) */






