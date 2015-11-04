//
//  position.h
//  InverseKinematics
//
//  Created by MengTsao on 3/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__position__
#define __InverseKinematics__position__

#include <cmath>
#include <stdlib.h>
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
        return Position(lhs.values[0] + rhs.values[0],lhs.values[1] + rhs.values[1], lhs.values[2] + rhs.values[2]);
    };
    
    friend Position operator-(Position lhs, Position rhs){
        return Position(lhs.values[0] - rhs.values[0], lhs.values[1] - rhs.values[1],lhs.values[2] - rhs.values[2]);
    };
    
    friend Position operator*(Position lhs, float rhs){
        return Position(lhs.values[0] * rhs, lhs.values[1] * rhs, lhs.values[2] * rhs);
    };
    
    friend Position operator*(float lhs, Position rhs){
        return Position(rhs.values[0] * lhs, rhs.values[1] * lhs, rhs.values[2] * lhs);
    };
    
    friend Position operator/(Position lhs, float rhs){
        return Position(lhs.values[0] / rhs, lhs.values[1] / rhs, lhs.values[2] / rhs);
    };
    
};

#endif /* defined(__InverseKinematics__position__) */
