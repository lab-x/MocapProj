//
//  quaternion.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include "quaternion.h"
#include "vector3.h"
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

Quaternion Quaternion::v2q(Vector3& vec1, Vector3& vec2)
{
    Vector3 V1 = Vector3::Normalize(vec1);
    Vector3 V2 = Vector3::Normalize(vec2);
    float cos_theta = Vector3::Dot(V1,V2);
    float half_cos = sqrt(0.5f * (1.f + cos_theta));
    float half_sin = sqrt(0.5f * (1.f - cos_theta));
    Vector3 Cross = Vector3::cross(V1, V2);
    Vector3 w = Vector3::Normalize(Cross);
    return Quaternion(half_cos,
                      half_sin * w.getX(),
                      half_sin * w.getY(),
                      half_sin * w.getZ());
}
Quaternion Quaternion::conjugate(Quaternion q1)
{
    return Quaternion(q1.getW(), -q1.getX(), -q1.getY(), -q1.getZ());
}