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
#include "axes.h"

class Point {
    Position position;
    Axes     axes;
//call public :
public:
    Position getPosition();
    Axes getAxes();
    void setPosition(Position rhs);
    void setAxes(Axes rhs);

public:
    Point();
    Point(Position P, Axes A);
    Point(Point point[]);
    Point(const Point& other );
    Point& operator=(const Point& rhs);
    
};

#endif /* defined(__InverseKinematics__point__) */






