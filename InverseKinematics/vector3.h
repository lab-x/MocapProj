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
    Vector3() {};
    Vector3(float x, float y, float z);

    
public :
    static Vector3 Normalize(Vector3& vect){
        Vector3 rst;
        rst = vect / Getlen(vect);
        return rst;
    }
    
    static float Getlen(const Vector3& vect){
        return sqrt(vect.values[0]*vect.values[0] + vect.values[1]*vect.values[1] + vect.values[2]*vect.values[2]);
    }
    static void SetLength(Vector3& vect, float length)
    {
        vect = vect * (length / Getlen(vect));
    }
    
    static float Dot(Vector3& vec1, Vector3& vec2)
    {
        return vec1.values[0]*vec2.values[0] + vec1.values[1]*vec2.values[1] + vec1.values[2]*vec2.values[2];
    }
    static Vector3 cross(Vector3& vec1, Vector3& vec2)
    {
        return Vector3(vec1.values[1]*vec2.values[2] - vec1.values[2]*vec2.values[1],
                       vec1.values[2]*vec2.values[0] - vec1.values[0]*vec2.values[2],
                       vec1.values[0]*vec2.values[1] - vec1.values[1]*vec2.values[0]);
    }
    
    
public:
    
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

};


#endif /* defined(__InverseKinematics__vector3__) */
