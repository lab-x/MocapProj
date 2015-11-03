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
    float values[3];
    
public:
    Point() {}
    Point(float xx, float yy, float zz);
    
    Point(float point[]);
    Point(const Point& other );
    
    float* getValues();
    float getX();
    float getY();
    float getZ();

    float getDistance();
    float Distance(Point lhs, const Point& rhs);
    void normalize();
    
    
    Point& operator+=(const Point& rhs);
    Point& operator-=(const Point& rhs);
    Point& operator=(const Point& rhs);
    bool operator == (const Point &rhs) const;
    
    friend Point operator+(Point lhs, const Point& rhs){
        Point rst;
        rst.values[0] = lhs.values[0] + rhs.values[0];
        rst.values[1] = lhs.values[1] + rhs.values[1];
        rst.values[2] = lhs.values[2] + rhs.values[2];
        
        return rst;
    };

    friend Point operator-(Point lhs, const Point& rhs){
        Point rst;
        rst.values[0] = lhs.values[0] - rhs.values[0];
        rst.values[1] = lhs.values[1] - rhs.values[1];
        rst.values[2] = lhs.values[2] - rhs.values[2];
        
        return rst;
    };

    friend Point operator*(Point lhs, const float& rhs){
        Point rst;
        rst.values[0] = lhs.values[0] * rhs;
        rst.values[1] = lhs.values[1] * rhs;
        rst.values[2] = lhs.values[2] * rhs;
        
        return rst;
    };
    
    friend Point operator*(const float& lhs, Point rhs){
        Point rst;
        rst.values[0] = rhs.values[0] * lhs;
        rst.values[1] = rhs.values[1] * lhs;
        rst.values[2] = rhs.values[2] * lhs;
        
        return rst;
    };
    
    friend Point operator/(Point lhs, const float& rhs){
        Point rst;
        rst.values[0] = lhs.values[0] / rhs;
        rst.values[1] = lhs.values[1] / rhs;
        rst.values[2] = lhs.values[2] / rhs;
        
        return rst;
    };

};

#endif /* defined(__InverseKinematics__point__) */






