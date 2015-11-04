//
//  vector3.h
//  InverseKinematics
//
//  Created by MengTsao on 3/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#ifndef __InverseKinematics__vector3__
#define __InverseKinematics__vector3__

#include <stdio.h>
#include <cmath> 

class Vector3
{
     float values[3];
    
public:
    Vector3();
    Vector3(float x, float y, float z);
    Vector3& operator=(const Vector3& rhs);
    
    float getX();
    float getY();
    float getZ();

    
    
    friend Vector3 operator+(const Vector3& lhs, Vector3 rhs){
        return Vector3( lhs.values[0] - rhs.values[0], lhs.values[1] - rhs.values[1], lhs.values[2] - rhs.values[2]);
    };
    
    friend Vector3 operator-(const Vector3& lhs){
        return Vector3(-lhs.values[0], -lhs.values[1], -lhs.values[2]);
    };
    
    friend Vector3 operator*(Vector3 lhs, const float& num){
         return Vector3(lhs.values[0] * num, lhs.values[1] * num, lhs.values[2] * num);
    };
    
    friend Vector3 operator/(Vector3 lhs, const float& num){
        return Vector3(lhs.values[0] / num, lhs.values[1] / num, lhs.values[2] / num);
    };
    
    public :
    static Vector3 Normalize(Vector3 vect);
    
    static float Getlen(Vector3 vect);
    
    static void SetLength(Vector3 vect, float length);
    
    static float Dot(Vector3 vec1, Vector3 vec2);

    static Vector3 cross(Vector3 vec1, Vector3 vec2);
    

};


#endif /* defined(__InverseKinematics__vector3__) */
