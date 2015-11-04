//
//  main.cpp
//  InverseKinematics
//
//  Created by MengTsao on 2/11/15.
//  Copyright (c) 2015 MengTsao. All rights reserved.
//

#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "point.h"
#include "fabrik.h"
#include "quaternion.h"
#include "vector3.h"

#define PI 3.14159265
float posX = 2.0f;
float posY = 2.0f;
float posZ = 0.0f;
float fov = -40.0f;
float rotx = 168;
float roty = 180;

int jointNum = 0;
float tolerance = 0.01;
float moveStep = 0.2;
float epsilon = moveStep * 10;
float goal[3];
std::vector<float> endPosition[3];

//****************************************************
// Global Variables
//****************************************************
Fabrik fabrik(tolerance, epsilon);

void generateLinks() {
    Position p0(0.0001, 0.0001, 10);
    Position p1(0, 10, 10);
    Position p2(0, 20, 10);
    Position p3(0, 30, 10);
    Position goal(0, 3, 68);
    
    Axis AxisInit;

    Point P0(p0, AxisInit);
    Point P1(p1, AxisInit);
    Point P2(p2, AxisInit);
    Point P3(p3, AxisInit);
    Point GOAL(goal, AxisInit);
    
    fabrik.setGoal(GOAL);
    fabrik.setJoints(P0,P1,P2,P3);
    jointNum = 4;
}

int main(int argc, const char * argv[]) {
    // insert code here...
    
    generateLinks();
    
    fabrik.compute();
    
    
    Point* joints = fabrik.getJoints();
    
    for (int i = 0; i < jointNum; i++) {
        
        printf("%lf,%lf,%lf\n",joints[i].getPosition().getValues()[0],joints[i].getPosition().getValues()[1],joints[i].getPosition().getValues()[2]);
    }
    Quaternion q1(0.707,0.707,0,0);
    Quaternion q2(0.707,0, 0.707,0);
    Quaternion q3 = q1*q2;
    printf("%lf  %lf  %lf  %lf\n",q3.getW(),q3.getX(),q3.getY(),q3.getZ());
    
    Vector3 A(1,0,0);
    Vector3 B(0,1,0);
    //Vector3 C(0,0,1);
    //C = Vector3::cross(A, B);    //float b = 4;
    //Vector3 a =cs A*b;
    //printf("%lf  %lf  %lf \n",a.getX(),a.getY(),a.getZ());
    //q3 = Quaternion::v2q(A,B);
    
    
    
    return 0;
}
