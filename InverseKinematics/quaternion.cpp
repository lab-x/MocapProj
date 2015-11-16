//
//  quaternion.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include "quaternion.h"
#include "vector3.h"
Quaternion::Quaternion(){
    values[0] = 1;
    values[1] = 0;
    values[2] = 0;
    values[3] = 0;

}
Quaternion::Quaternion(float w, float x, float y, float z) {
    values[0] = w;
    values[1] = x;
    values[2] = y;
    values[3] = z;
}

Quaternion::Quaternion(float quaternion[]) {
    values[0] = quaternion[0];
    values[1] = quaternion[1];
    values[2] = quaternion[2];
    values[3] = quaternion[3];
}

Quaternion::Quaternion( const Quaternion& other ) {
    values[0] = other.values[0];
    values[1] = other.values[1];
  	values[2] = other.values[2];
  	values[3] = other.values[3];
}


Quaternion& Quaternion::operator=(const Quaternion& rhs) {
    this->values[0] = rhs.values[0];
  	this->values[1] = rhs.values[1];
    this->values[2] = rhs.values[2];
    this->values[3] = rhs.values[3];
  	return *this;
}



float Quaternion::getW() {
    return values[0];
}

float Quaternion::getX() {
    return values[1];
}

float Quaternion::getY() {
    return values[2];
}

float Quaternion::getZ() {
    return values[3];
}
void Quaternion::normalize() {
    float length = sqrt(pow(values[0], 2)
                        + pow(values[1], 2)
                        + pow(values[2], 2)
                        + pow(values[3], 2));
    values[0] = values[0] / length;
    values[1] = values[1] / length;
    values[2] = values[2] / length;
    values[3] = values[3] / length;
}

/* --------- --------- ---------
 Known Bug Case:
 Vector3 A(1,0,0);
 Vector3 B(-1,0,0);
 result in (0.00 nan nan nan)!
 --------- --------- ---------*/
Quaternion Quaternion::conjugate(Quaternion q1)
{
    return Quaternion(q1.getW(), -q1.getX(), -q1.getY(), -q1.getZ());
}

Quaternion Quaternion::v2q(Vector3 vec1, Vector3 vec2)
{
    Vector3 V1 = Vector3::Normalize(vec1);
    Vector3 V2 = Vector3::Normalize(vec2);
    float cos_theta = Vector3::Dot(V1,V2);
    float half_cos = sqrt(0.5f * (1.f + cos_theta));
    float half_sin = sqrt(0.5f * (1.f - cos_theta));
    Vector3 Cross = Vector3::cross(V1, V2);
    Vector3 w = Vector3::Normalize(Cross);
    return Quaternion(half_cos, half_sin * w.getX(), half_sin * w.getY(), half_sin * w.getZ());
}

Vector3 Quaternion::rotVbyQ(Vector3 v, Quaternion q)
{
    //Shall be same as the following
    // vector part of the quaternion
    /*Vector3 u(q.getX(),q.getY(),q.getZ());
     // scalar part of the quaternion
     float s = q.getW();
     Vector3 vprime(u * 2.0f * Vector3::Dot(u, v) + v*(s*s - Vector3::Dot(u, u)) + Vector3::cross(u, v) * 2.0f * s);
     return vprime;
     */
    Quaternion invq = Quaternion::conjugate(q);
    Quaternion V(0, v.getX(), v.getY(), v.getZ());
    Quaternion tmp1 = q*V;
    Quaternion tmp = tmp1*invq;
    Vector3 ret(tmp.getX(), tmp.getY(), tmp.getZ());
    return ret;
}

Vector3 Quaternion::Quat2Angle(Quaternion q){
    //Type 3 'YXZ'
    float r11 = 2 * (q.getX() * q.getZ() + q.getW() * q.getY());
    float r12 = powf(q.getW(), 2) - powf(q.getX(), 2) - powf(q.getY(), 2) + powf(q.getZ(), 2);
    float r21 = -2 * (q.getY()*q.getZ() - q.getW() * q.getX());
    float r31 = 2*(q.getX() * q.getY() + q.getW()*q.getZ());
    float r32 =powf(q.getW(), 2) - powf(q.getX(), 2) + powf(q.getY(), 2) - powf(q.getZ(), 2);
    Vector3 ret(atan2f(r11, r12), asinf(r21), atan2f(r31,r32));
    return ret;
}

void Quaternion::Quat2AxisAngle(Quaternion q, Vector3 *axis, float *angle){
    float x,y,z;
    
    float theta = 2 * acos(q.getW());
    *angle = theta*57.3;
    
    float s = sqrt(1-q.getW()*q.getW());
    // assuming quaternion normalised then w is less than 1, so term always positive.
    if (s < 0.01) { // test to avoid divide by zero, s is always positive due to sqrt
        // if s close to zero then direction of axis not important
        x = q.getX(); // if it is important that axis is normalised then replace with x=1; y=z=0;
        y = q.getY();
        z = q.getZ();
    } else {
        x = q.getX() / s; // normalise axis
        y = q.getY() / s;
        z = q.getZ() / s;
    }
    
    Vector3 A = Vector3(x,y,z);
    A.norm();
    *axis = A;
}
void Quaternion::AxisAngle2Quat(Vector3 axis, float angle, Quaternion *q ){
    float RadAng = angle/57.3;
    float qw = cos(RadAng/2);
    float qx = axis.getX() * sin(RadAng/2);
    float qy = axis.getY() * sin(RadAng/2);
    float qz = axis.getZ() * sin(RadAng/2);
    *q = Quaternion(qw,qx,qy,qz);
}


void Quaternion::test(int a, int b, int *c){
    int rst = a*b;
    *c = rst;
}