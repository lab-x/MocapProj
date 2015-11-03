//
//  position.h
//  InverseKinematics
//
//  Created by MengTsao on 3/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__position__
#define __InverseKinematics__position__

#include <stdio.h>
#include "vector3.h"

class Position {
    float values[3];
    
public:
    Position() {}
    Position(float xx, float yy, float zz);
    
    Position(float position[]);
    Position(const Position& other );
    
    float* getValues();
    float getX();
    float getY();
    float getZ();
    
    float getDistance();
    float Distance(Position lhs, Position rhs);
    void normalize();
    
    
    Position& operator+=(const Position& rhs);
    Position& operator-=(const Position& rhs);
    Position& operator=(const Position& rhs);
    bool operator == (const Position &rhs) const;
    
    friend Position operator+(Position lhs, const Position& rhs){
        Position rst;
        rst.values[0] = lhs.values[0] + rhs.values[0];
        rst.values[1] = lhs.values[1] + rhs.values[1];
        rst.values[2] = lhs.values[2] + rhs.values[2];
        
        return rst;
    };
    
    friend Position operator-(Position lhs, Position rhs){
        Position rst;
        rst.values[0] = lhs.values[0] - rhs.values[0];
        rst.values[1] = lhs.values[1] - rhs.values[1];
        rst.values[2] = lhs.values[2] - rhs.values[2];
        
        return rst;
    };
    
    friend Position operator*(Position lhs, float rhs){
        Position rst;
        rst.values[0] = lhs.values[0] * rhs;
        rst.values[1] = lhs.values[1] * rhs;
        rst.values[2] = lhs.values[2] * rhs;
        
        return rst;
    };
    
    friend Position operator*(float lhs, Position rhs){
        Position rst;
        rst.values[0] = rhs.values[0] * lhs;
        rst.values[1] = rhs.values[1] * lhs;
        rst.values[2] = rhs.values[2] * lhs;
        
        return rst;
    };
    
    friend Position operator/(Position lhs, float rhs){
        Position rst;
        rst.values[0] = lhs.values[0] / rhs;
        rst.values[1] = lhs.values[1] / rhs;
        rst.values[2] = lhs.values[2] / rhs;
        
        return rst;
    };
    
};

#endif /* defined(__InverseKinematics__position__) */
